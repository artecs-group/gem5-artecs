/**
 * Scattering-Gathering Aware DMA Controller
 * -----------------------------------------
 *
 * Description of
 * SGA_DMA_REQ_REG:
 *
 * 52    47               0
 * +=====+================+
 * | CMD | DATA / ADDRESS |
 * +=====+================+
 *
 * Table of commands:
 * see cat_cmd_interface.hh
 *
 * -----------------------------------------
 *
 * Description of
 * SGA_DMA_RESP_REG:
 *
 * 3          0
 * +==========+
 * | ACK/NACK |
 * +==========+
 *
 * ACK/NACK codes:
 * [0b0000]: COMMAND NACK
 * [0b1111]: COMMAND ACK
 *
 * -----------------------------------------
 *
 * Description of
 * SGA_DMA_STS_REG:
 *
 * 3          0
 * +==========+
 * |  STATUS  |
 * +==========+
 *
 * STATUS codes:
 * [0b0000]: IDLE
 * [0b1111]: RUNNING
 **/

#ifndef __DEV_SGA_DMA_CONTROLLER_HH__
#define __DEV_SGA_DMA_CONTROLLER_HH__

#include "debug/SGA_DMA.hh"
#include "dev/cat_cmd_interface.hh"
#include "dev/sga_dma_device.hh"
#include "params/SgaDmaController.hh"

namespace gem5
{

class SgaDmaController : public SgaDmaDevice, public CatCmdInterface
{
  protected:
    const ByteOrder byteOrder = ByteOrder::little;

    /** Address that the device listens to. */
    Addr pioAddr;

    /** Size that the device's address range. */
    Addr pioSize;

    /** Delay that the device experiences on an access. */
    Cycles pioDelay;

  private:
    /* Register map */
    enum ioreg_t
    {
        SGA_DMA_REQ_REG,        // Request register, write-only
        SGA_DMA_RESP_REG,       // Response register, read-only
        SGA_DMA_STS_REG,        // Status register, read-only
        SGA_DMA_IOREG_ITEMS
    };

    /* Response type */
    enum resp_t
    {
        CAT_CMD_NACK,
        CAT_CMD_ACK
    };

    /* Running status */
    enum status_t
    {
        CAT_STS_IDLE,
        CAT_STS_RUNNING
    };

    /* DMA FIFO */
    SgaDmaReadFifo *dmaFifo;

    /* Response register */
    uint8_t responseReg;

    /* Status register */
    uint8_t statusReg;

    /* Is the DMA engine running? */
    bool running;

    /* Current parameters */
    params_t currentParams;

    /* Set the response to the corresponding register */
    void setResponse(resp_t resp);

    /* Set the status to the corresponding register  */
    void setStatus(status_t status);

    /* Methods to access SGA-DMA registers */
    uint64_t regRead(Addr addr);
    void regWrite(Addr addr, uint64_t data);

    /* Methods to start and stop the DMA transfer */
    bool startTransfer();
    bool stopTransfer();

  public:
    PARAMS(SgaDmaController);
    SgaDmaController(const Params &params);
    ~SgaDmaController();
    AddrRangeList getAddrRanges() const override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace gem5

#endif /* __DEV_SGA_DMA_CONTROLLER_HH__ */
