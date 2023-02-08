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
    running(false)
{
    dmaFifo = new SgaDmaReadFifo(dmaPort, 1024, 64, 8,
                                 Request::UNCACHEABLE);
}

SgaDmaController::~SgaDmaController()
{
    delete(dmaFifo);
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

void
SgaDmaController::setResponse(resp_t resp)
{
    uint8_t value;
    switch (resp) {
      default:
      case CAT_CMD_NACK:
        value = 0b0000;
        break;
      case CAT_CMD_ACK:
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
      case CAT_STS_IDLE:
        value = 0b0000;
        break;
      case CAT_STS_RUNNING:
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
        DPRINTF(SGA_DMA, "Reading SGA_DMA_STS_REG: %#x\n", r);
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
        DPRINTF(SGA_DMA, "Writing SGA_DMA_REQ_REG: %#x\n", data);
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
        DPRINTF(SGA_DMA, "Received command SGA_DMA_%s\n", cmd_name);
        setResponse(CAT_CMD_ACK);
        return;
    }

    switch(cmd) {
      case CAT_NO_COMMAND:
        panic("No command received\n");
        setResponse(CAT_CMD_NACK);
        break;

      case CAT_START_STOP:
        DPRINTF(SGA_DMA, "Received command CAT_START_STOP\n");
        switch(decodeStartStop(payload)) {
          case CAT_SUB_START:
            DPRINTF(SGA_DMA, "Command is START, beginning transfer\n");
            startTransfer();
            setResponse(CAT_CMD_ACK);
            break;

          case CAT_SUB_STOP:
            DPRINTF(SGA_DMA, "Command is STOP, interrupting transfer\n");
            stopTransfer();
            setResponse(CAT_CMD_ACK);
            break;

          default:
            panic("Command in neither START or STOP!");
            setResponse(CAT_CMD_NACK);
            break;
        }
        break;

      default:
        panic("Command is invalid!");
        setResponse(CAT_CMD_NACK);
        break;
    }
}

bool
SgaDmaController::startTransfer()
{
    bool success = false;
    if (!running) {
        running = true;
        Addr src = currentParams.start_addr;
        Addr dst = currentParams.tr_b_addr;
        Addr len = currentParams.length;
        dmaFifo->startFill(src, dst, len);
    }
    return success;
}

bool
SgaDmaController::stopTransfer()
{
    bool success = false;
    return success;
}

Tick
SgaDmaController::read(PacketPtr pkt)
{
    Addr pkt_addr = pkt->getAddr();
    if (pkt_addr < pioAddr || pkt_addr >= pioAddr + pioSize) {
        panic("Address %#x is out of SGA_DMA address range!", pkt_addr);
        return 1;
    }

    Addr sga_dma_addr = pkt_addr - pioAddr;

    DPRINTF(SGA_DMA,
        "Read request - addr: %#x, size: %#x\n", sga_dma_addr, pkt->getSize());

    uint64_t read = regRead(sga_dma_addr);
    DPRINTF(SGA_DMA, "Packet Read: %#x\n", read);
    pkt->setUintX(read, byteOrder);
    pkt->makeResponse();

    return pioDelay;
}

Tick
SgaDmaController::write(PacketPtr pkt)
{
    Addr pkt_addr = pkt->getAddr();
    if (pkt_addr < pioAddr || pkt_addr >= pioAddr + pioSize) {
        panic("Address %#x is out of SGA_DMA address range!", pkt_addr);
        return 1;
    }

    Addr sga_dma_addr = pkt_addr - pioAddr;

    DPRINTF(SGA_DMA, "Write register %#x value %#x\n", sga_dma_addr,
            pkt->getUintX(byteOrder));

    regWrite(sga_dma_addr, pkt->getUintX(byteOrder));
    DPRINTF(SGA_DMA, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));
    pkt->makeResponse();

    return pioDelay;
}

} // namespace gem5
