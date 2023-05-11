#include "dev/sga_dma_controller.hh"

#include "base/trace.hh"
#include "debug/AddrRanges.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

namespace cat
{

SgaDmaController::SgaDmaController(const Params &params) :
    SgaDmaDevice(params),
    pioAddr(params.pio_addr),
    pioSize(SGA_DMA_IOREG_ITEMS * sizeof(uint64_t)),
    pioDelay(params.pio_latency),
    responseReg(0),
    statusReg(0),
    running(false),
    scattering(false),
    conflict(false)
{
    eocEvent = new EventFunctionWrapper([this]{ eocCallback(); }, name());
    eotEvent = new EventFunctionWrapper([this]{ eotCallback(); }, name());
    dmaFifo  = new SgaDmaCbFifo(this, dmaPort, 1024, 64, 8,
                                Request::UNCACHEABLE, eocEvent, eotEvent);
}

SgaDmaController::~SgaDmaController()
{
    delete(dmaFifo);
    delete(eotEvent);
    delete(eocEvent);
}

AddrRangeList
SgaDmaController::getAddrRanges() const
{
    assert(pioSize != 0);
    AddrRangeList ranges;
    DPRINTF(AddrRanges, "registering range: %#x-%#x\n", pioAddr, pioSize);
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

bool
SgaDmaController::decodeDir(uint64_t req) const
{
    bool _scattering;
    switch ((req >> 8) & 0xff) {
      default:
      case 0x00:
        _scattering = false;
      break;
      case 0xf0:
        _scattering = true;
      break;
    }
    return _scattering;
}

void
SgaDmaController::setCmdResponse(bool ack)
{
    uint8_t value;
    if (!ack) {
        value = 0b0000;
    } else {
        value = 0b1111;
    }
    responseReg &= ~((uint8_t)0b1111);
    responseReg |=  ((uint8_t)value);
}

void
SgaDmaController::setBusyResponse(bool busy)
{
    uint8_t value;
    if (!busy) {
        value = 0b0000;
    } else {
        value = 0b1111;
    }
    responseReg &= ~((uint8_t)0b1111 << 4);
    responseReg |=  ((uint8_t)value  << 4);
}

void
SgaDmaController::setStatus(bool running)
{
    uint8_t value;
    if (!running) {
        value = 0b0000;
    } else {
        value = 0b1111;
    }
    statusReg &= ~((uint8_t)0b1111);
    statusReg |=  ((uint8_t)value);
}

void
SgaDmaController::checkBusy(Addr addr_sz)
{
    // Note 1: Assume the access being checked is a not a strided one.
    // Note 2: The completion flags are set per chunk, which typically
    //         has a size equal or larger than the cache line.
    uint8_t size = (addr_sz >> 53) & 0xff;
    Addr addr = addr_sz & 0xffffffffffff;
    bool busy = false;

    // Check pending tranfers first
    for (auto op : pending_ops) {
        amap_t &op_lut = op.first;
        bool op_scat = op.second;
        if (op_scat) {
            auto it = op_lut.lower_bound(lkAlign(addr));
            if ((it != op_lut.end()) && (it->first - addr < size)) {
                busy = true;
                break;
            }
        } else {
            auto it = op_lut.to.lower_bound(lkAlign(addr));
            if ((it != op_lut.to.end()) && (it->second - addr < size)) {
                busy = true;
                break;
            }
        }
    }

    // Check current transfer
    if (!busy) {
        uint16_t index;
        Addr lk_addr = 0;
        if (scattering) {
            auto it = lut.lower_bound(lkAlign(addr));
            index = std::distance(lut.begin(), it);
            if (index < lut.size()) {
                lk_addr = it->first;
            }
        } else {
            auto it = lut.to.lower_bound(lkAlign(addr));
            index = std::distance(lut.to.begin(), it);
            if (index < lut.size()) {
                lk_addr = it->second;
            }
        }
        if (lk_addr && (lk_addr - addr < size) && !compFlags[index]) {
            busy = true;
        }
    }

    if (busy) {
        DPRINTF(SgaDma, "Address is BUSY\n");
        conflict = true;
        setBusyResponse(true);
        return;
    }

    DPRINTF(SgaDma, "Address is NOT BUSY\n");
    setBusyResponse(false);
}

uint64_t
SgaDmaController::regRead(Addr addr)
{
    uint64_t r = 0;
    switch (addr / sizeof(uint64_t)) {
      case SGA_DMA_REQ_REG:
        r = -1;
        panic("SGA_DMA_REQ_REG is write-only!");
        break;
      case SGA_DMA_RESP_REG:
        r = responseReg;
        //DPRINTF(SgaDma, "Reading SGA_DMA_RESP_REG: %#x\n", r);
        break;
      case SGA_DMA_STS_REG:
        r = statusReg;
        //DPRINTF(SgaDma, "Reading SGA_DMA_STS_REG: %#x\n", r);
        break;
      default:
        r = -1;
        panic("Unexpected read to SGA-DMA at address %lld!", addr);
        break;
    }
    return r;
}

void
SgaDmaController::regWrite(Addr addr, uint64_t data)
{
    bool process_req = false;
    switch (addr / sizeof(uint64_t)) {
      case SGA_DMA_REQ_REG:
        //DPRINTF(SgaDma, "Writing SGA_DMA_REQ_REG: %#x\n", data);
        process_req = true;
        break;
      case SGA_DMA_RESP_REG:
        panic("SGA_DMA_RESP_REG is read-only!");
        break;
      case SGA_DMA_STS_REG:
        panic("SGA_DMA_ST_REG is read-only!");
        break;
      default:
        panic("Unexpected write to SGA-DMA at address %lld!", addr);
        break;
    }

    if (!process_req) {
        // Nothing to do. Should never reach this point anyway.
        return;
    }

    cmd_t    cmd     = decodeCmd(data);
    uint64_t payload = getPayload(data);

    std::string cmd_name = "CMD_UNKNOWN";
    bool success = setParams(data, currentParams, cmd_name);
    if (success) {
        DPRINTF(SgaDma, "Received command SGA_DMA_%s\n", cmd_name);
        setCmdResponse(true);
        return;
    }

    switch(cmd) {
      case CAT_NO_COMMAND:
        if (running) {
            DPRINTF(SgaDma, "No command received, checking if requested "
                    "address %#x is available\n", payload);
            checkBusy(payload);
        }
        break;

      case CAT_START_STOP:
        DPRINTF(SgaDma, "Received command CAT_START_STOP\n");
        switch(decodeStartStop(payload)) {
          case CAT_SUB_START:
            DPRINTF(SgaDma, "Command is START\n");
            startTransfer(decodeDir(payload));
            setCmdResponse(true);
            break;

          case CAT_SUB_STOP:
            DPRINTF(SgaDma, "Command is STOP, interrupting transfers\n");
            stopTransfer();
            setCmdResponse(true);
            break;

          default:
            panic("Command in neither START or STOP!");
            setCmdResponse(false);
            break;
        }
        break;

      default:
        panic("Command is invalid!");
        setCmdResponse(false);
        break;
    }
}

bool
SgaDmaController::startTransfer(bool _scattering)
{
    if (!running) {
        DPRINTF(SgaDma, "Queue is free, starting a new transfer\n");
        generateLut(currentParams, lut);
        // (re-)initialize completion flags vector
        compFlags.clear();
        compFlags.resize(lut.size(), false);
        // start the dma transfer based on the lut and direction
        scattering = _scattering;
        dmaFifo->startFill(lut, _scattering, compFlags);
        setStatus(true);
        running = true;
    } else {
        DPRINTF(SgaDma, "Another transfer is ongoing, enqueuing\n");
        amap_t q_lut;
        generateLut(currentParams, q_lut);
        pending_ops.push_back(std::make_pair(q_lut, _scattering));
    }
    return true;
}

bool
SgaDmaController::stopTransfer()
{
    if (running) {
        dmaFifo->stopFill();
        pending_ops = {};
        running = false;
        return true;
    }
    return false;
}

void
SgaDmaController::eocCallback()
{
    if (conflict) {
        // Send a custom "retry packet" through dmaPort
        RequestPtr req = std::make_shared<Request>(0x8000000000000000,
            sizeof(uint64_t), 0, Request::funcRequestorId);
        PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
        DPRINTF(SgaDma, "DMA chunk complete, signaling TranslatingXBar\n");
        dmaPort.sendFunctional(pkt);
        delete pkt;
    }
    conflict = false;
}

void
SgaDmaController::eotCallback()
{
    DPRINTF(SgaDma, "The DMA transfer has been %s\n",
            running ? "completed" : "canceled");
    lut.clear();
    if (!pending_ops.empty()) {
        std::pair<amap_t, bool> op = pending_ops.front();
        DPRINTF(SgaDma, "Process next pending request\n");
        lut = amap_t(op.first);
        // (re-)initialize completion flags vector
        compFlags.clear();
        compFlags.resize(lut.size(), false);
        // start the dma transfer based on the lut and direction
        scattering = op.second;
        dmaFifo->startFill(lut, op.second, compFlags);
        pending_ops.pop_front();
    } else {
        setStatus(false);
        running = false;
    }
}

Tick
SgaDmaController::read(PacketPtr pkt)
{
    Addr pkt_addr = pkt->getAddr();
    if (pkt_addr < pioAddr || pkt_addr >= pioAddr + pioSize) {
        panic("Address %#x is out of SgaDma address range!", pkt_addr);
        return 1;
    }

    Addr sga_dma_addr = pkt_addr - pioAddr;

    //DPRINTF(SgaDma, "Read request - addr: %#x, size: %#x\n",
    //        sga_dma_addr, pkt->getSize());

    uint64_t read = regRead(sga_dma_addr);
    //DPRINTF(SgaDma, "Packet Read: %#x\n", read);
    pkt->setUintX(read, byteOrder);
    pkt->makeResponse();

    return pioDelay;
}

Tick
SgaDmaController::write(PacketPtr pkt)
{
    Addr pkt_addr = pkt->getAddr();
    if (pkt_addr < pioAddr || pkt_addr >= pioAddr + pioSize) {
        panic("Address %#x is out of SgaDma address range!", pkt_addr);
        return 1;
    }

    Addr sga_dma_addr = pkt_addr - pioAddr;

    //DPRINTF(SgaDma, "Write register %#x value %#x\n", sga_dma_addr,
    //        pkt->getUintX(byteOrder));

    regWrite(sga_dma_addr, pkt->getUintX(byteOrder));
    //DPRINTF(SgaDma, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));
    pkt->makeResponse();

    return pioDelay;
}

} // namespace cat

} // namespace gem5
