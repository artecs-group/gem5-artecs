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

    /* DMA FIFO */
    SgaDmaCbFifo *dmaFifo;

    /* Response register */
    uint8_t responseReg;

    /* Status register */
    uint8_t statusReg;

    /* Is the DMA engine running? */
    bool running;

    /* Direction flag (true: scattering, false: gathering) */
    bool scattering;

    /* Current parameters */
    params_t currentParams;

    /* Address LUT */
    amap_t lut;

    /* Completion flags vector */
    std::vector<bool> compFlags;

    /* Pending operations */
    std::deque<std::pair<amap_t, bool>> pending_ops;

    /* Event triggered at the end of a DMA chunk transfer */
    EventFunctionWrapper *eocEvent;

    /* Event triggered at the end of a DMA transfer (all chunks) */
    EventFunctionWrapper *eotEvent;

    /* Decode the direction of a start request */
    bool decodeDir(uint64_t req) const;

    /* Set the response to the corresponding register */
    void setCmdResponse(bool ack);
    void setBusyResponse(bool busy);

    /* Set the status to the corresponding register  */
    void setStatus(bool running);

    /* Method to check whether the requested address is busy */
    void checkBusy(Addr addr);

    /* Methods to access SGA-DMA registers */
    uint64_t regRead(Addr addr);
    void regWrite(Addr addr, uint64_t data);

    /* Methods to start and stop the DMA transfer */
    bool startTransfer(bool _scattering);
    bool stopTransfer();

    /* Callback of eocEvent to retry conflicting requests */
    void eocCallback();

    /* Callback of eotEvent to update the running status */
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
