#include "dev/sga_dma_controller.hh"

#include "base/trace.hh"
#include "debug/AddrRanges.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

SgaDmaController::SgaDmaController(const Params &params) :
    SgaDmaDevice(params),
    pioAddr(params.pio_addr),
    pioSize(SGA_DMA_IOREG_ITEMS * sizeof(uint64_t)),
    pioDelay(params.pio_latency),
    responseReg(0),
    statusReg(0),
    running(false),
    direction(SGA_DMA_DIR_GATH)
{
    eotEvent = new EventFunctionWrapper([this]{ eotCallback(); }, name(),
                                        true);
    dmaFifo  = new SgaDmaCbFifo(this, dmaPort, 1024, 64, 8,
                                Request::UNCACHEABLE, eotEvent);
}

SgaDmaController::~SgaDmaController()
{
    delete(dmaFifo);
    delete(eotEvent);
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

SgaDmaController::dir_t
SgaDmaController::decodeDir(uint64_t req) const
{
    dir_t dir;
    switch ((req >> 8) & 0xff) {
      default:
      case 0x00:
        dir = SGA_DMA_DIR_GATH;
      break;
      case 0xf0:
        dir = SGA_DMA_DIR_SCAT;
      break;
    }
    return dir;
}

void
SgaDmaController::setResponse(resp_t resp)
{
    uint8_t value;
    switch (resp) {
      default:
      case SGA_DMA_CMD_NACK:
        value = 0b0000;
        break;
      case SGA_DMA_CMD_ACK:
        value = 0b1111;
        break;
    }
    responseReg &= ~((uint8_t)0b1111);
    responseReg |=  ((uint8_t)value);
}

void
SgaDmaController::setStatus(status_t status)
{
    uint8_t value;
    switch (status) {
      default:
      case SGA_DMA_STS_IDLE:
        value = 0b0000;
        break;
      case SGA_DMA_STS_RUNNING:
        value = 0b1111;
        break;
    }
    statusReg &= ~((uint8_t)0b1111);
    statusReg |=  ((uint8_t)value);
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
      case SGA_DMA_STS_REG:
        r = statusReg;
        DPRINTF(SgaDma, "Reading SGA_DMA_STS_REG: %#x\n", r);
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
        DPRINTF(SgaDma, "Writing SGA_DMA_REQ_REG: %#x\n", data);
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
        setResponse(SGA_DMA_CMD_ACK);
        return;
    }

    switch(cmd) {
      case CAT_NO_COMMAND:
        panic("No command received\n");
        setResponse(SGA_DMA_CMD_NACK);
        break;

      case CAT_START_STOP:
        DPRINTF(SgaDma, "Received command CAT_START_STOP\n");
        switch(decodeStartStop(payload)) {
          case CAT_SUB_START:
            DPRINTF(SgaDma, "Command is START, beginning transfer\n");
            startTransfer(decodeDir(payload));
            setResponse(SGA_DMA_CMD_ACK);
            break;

          case CAT_SUB_STOP:
            DPRINTF(SgaDma, "Command is STOP, interrupting transfer\n");
            stopTransfer();
            setResponse(SGA_DMA_CMD_ACK);
            break;

          default:
            panic("Command in neither START or STOP!");
            setResponse(SGA_DMA_CMD_NACK);
            break;
        }
        break;

      default:
        panic("Command is invalid!");
        setResponse(SGA_DMA_CMD_NACK);
        break;
    }
}

bool
SgaDmaController::startTransfer(dir_t dir)
{
    if (!running) {
        Addr src = currentParams.start_addr;
        Addr dst = currentParams.tr_b_addr;
        Addr len = currentParams.length;
        direction = dir;
        dmaFifo->startFill(src, dst, len);
        setStatus(SGA_DMA_STS_RUNNING);
        running = true;
        return true;
    }
    return false;
}

bool
SgaDmaController::stopTransfer()
{
    if (running) {
        dmaFifo->stopFill();
        running = false;
        return true;
    }
    return false;
}

void
SgaDmaController::eotCallback()
{
    DPRINTF(SgaDma, "The DMA transfer has been %s\n",
            running ? "completed" : "canceled");
    setStatus(SGA_DMA_STS_IDLE);
    running = false;
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

    DPRINTF(SgaDma,
        "Read request - addr: %#x, size: %#x\n", sga_dma_addr, pkt->getSize());

    uint64_t read = regRead(sga_dma_addr);
    DPRINTF(SgaDma, "Packet Read: %#x\n", read);
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

    DPRINTF(SgaDma, "Write register %#x value %#x\n", sga_dma_addr,
            pkt->getUintX(byteOrder));

    regWrite(sga_dma_addr, pkt->getUintX(byteOrder));
    DPRINTF(SgaDma, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));
    pkt->makeResponse();

    return pioDelay;
}

} // namespace gem5
