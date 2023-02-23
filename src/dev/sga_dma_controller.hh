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
 * Table of commands: see cat_common.hh
 * Modifications:
 *
 * 52     47   15           7              0
 * +=======+====+===========+==============+
 * | 11111 | ND | DIRECTION | START / STOP |
 * +=======+====+===========+==============+
 *
 * DIRECTION codes:
 *
 * [0x00]: GATHERING
 * [0xf0]: SCATTERING
 *
 * -----------------------------------------
 *
 * Description of
 * SGA_DMA_RESP_REG:
 *
 * 7           3            0
 * +===========+============+
 * | BUSY ADDR | ACK / NACK |
 * +===========+============+
 *
 * BUSY ADDR codes:
 *
 * [0b0000]: ADDRESS AVAILABLE
 * [0b1111]: ADDRESS BUSY
 *
 * ACK / NACK codes:
 *
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
 *
 * [0b0000]: IDLE
 * [0b1111]: RUNNING
 **/

#ifndef __DEV_SGA_DMA_CONTROLLER_HH__
#define __DEV_SGA_DMA_CONTROLLER_HH__

#include "debug/SgaDma.hh"
#include "dev/cat_common.hh"
#include "dev/sga_dma_device.hh"
#include "params/SgaDmaController.hh"

namespace gem5
{

namespace cat
{

class SgaDmaController : public SgaDmaDevice
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
    enum resp_cmd_t
    {
        SGA_DMA_CMD_NACK,
        SGA_DMA_CMD_ACK
    };

    enum resp_busy_t
    {
        SGA_DMA_ADDR_AVAIL,
        SGA_DMA_ADDR_BUSY
    };

    /* Running status */
    enum status_t
    {
        SGA_DMA_STS_IDLE,
        SGA_DMA_STS_RUNNING
    };

    /* Scattering direction type */
    enum dir_t
    {
        SGA_DMA_DIR_GATH,
        SGA_DMA_DIR_SCAT
    };

    /* DMA FIFO */
    SgaDmaCbFifo *dmaFifo;

    /* Response register */
    uint8_t responseReg;

    /* Status register */
    uint8_t statusReg;

    /* Is the DMA engine running? */
    bool running;

    /* Scattering direction (gathering or scattering) */
    dir_t direction;

    /* Current parameters */
    params_t currentParams;

    /* Address LUT */
    std::map<Addr, Addr> lut;

    /* Compacted address range (after gathering) */
    std::pair<Addr, Addr> compRange;

    /* Event triggered at the end of a DMA transfer */
    EventFunctionWrapper *eotEvent;

    /* Decode the direction of a start request */
    dir_t decodeDir(uint64_t req) const;

    /* Set the response to the corresponding register */
    void setCmdResponse(resp_cmd_t resp);
    void setBusyResponse(resp_busy_t resp);

    /* Set the status to the corresponding register  */
    void setStatus(status_t status);

    /* Method to check whether the requested address is busy */
    void checkBusy(Addr addr);

    /* Methods to access SGA-DMA registers */
    uint64_t regRead(Addr addr);
    void regWrite(Addr addr, uint64_t data);

    /* Methods to start and stop the DMA transfer */
    bool startTransfer(dir_t dir);
    bool stopTransfer();

    /* Callback of eobEvent to update the status */
    void eotCallback();

  public:
    PARAMS(SgaDmaController);
    SgaDmaController(const Params &params);
    ~SgaDmaController();
    AddrRangeList getAddrRanges() const override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace cat

} // namespace gem5

#endif /* __DEV_SGA_DMA_CONTROLLER_HH__ */
