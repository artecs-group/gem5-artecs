/**
 * Compacting Address Translator (CAT)
 * -----------------------------------
 *
 * Description of
 * CAT_REQ_REG:
 *
 * 60         52    47               0
 * +==========+=====+================+
 * | ENTRY_ID | CMD | DATA / ADDRESS |
 * +==========+=====+================+
 *
 * Table of commands: see cat_common.hh
 *
 * -----------------------------------
 *
 * Description of
 * CAT_RESP_REG:
 *
 * 43       39        0
 * +========+=========+
 * | STATUS | ADDRESS |
 * +========+=========+
 *
 * List of CAT_RESP_REG statuses:
 *
 * [0b0000]: NOT MAPPED
 * [0b1010]: MAPPED, NOT FOUND
 * [0b1111]: MAPPED, FOUND
 * [0b0101]: COMMAND ACK
 * [0b1001]: COMMAND NACK
 */

#ifndef __DEV_CAT_HH__
#define __DEV_CAT_HH__

#include "dev/cat_common.hh"
#include "dev/io_device.hh"
#include "params/CAT.hh"

namespace gem5
{

namespace cat
{

class CAT : public BasicPioDevice
{
  protected:
    const ByteOrder byteOrder = ByteOrder::little;

  private:
    /* Register map */
    enum ioreg_t
    {
        CAT_REQ_REG,            // Request register, write-only
        CAT_RESP_REG,           // Response register, read-only
        CAT_RDY_REG,            // Ready time register, read-only
        CAT_IOREG_ITEMS
    };

    /* Response status */
    enum status_t
    {
        CAT_NOT_MAPPED,
        CAT_CMD_ACK,
        CAT_CMD_NACK,
        CAT_NOT_FOUND,
        CAT_FOUND
    };

    /* Information contained in a CAT entry */
    struct entry_t
    {
        params_t p;                       // Translation parameters
        Tick ready_time = 0;              // Ready time
        std::map<Addr, Addr> lut;         // Translation LUT
    };

    /* Address range with corresponding entry identifier */
    struct range_t
    {
        std::pair<Addr, Addr> addr_range;
        uint8_t entry_id;
    };

    /* Configuration latency (cycles) */
    Cycles configLat;
    /* Translation start latency (cycles) */
    Cycles startLat;
    /* Address lookup latency (cycles) */
    Cycles lookupLat;

    /* Vector of CAT entries */
    std::vector<entry_t> entries;

    /* Vector of mapped address ranges */
    std::vector<range_t> ranges;

    /* Response register */
    uint64_t responseReg;

    /* Ready time register */
    uint64_t readyTimeReg;

    /* Get the entry ID of a request */
    uint8_t getEntryID(uint64_t req) const;

    /* Sets the status to the response register */
    void setStatus(status_t status);

    /**
     * Method to perform an address lookup
     *
     * It returns void since the result is
     * saved in the response register.
     */
    void lookup(Addr addr);

    /**
     * Method to start the address translation
     *
     * - Computes and generates the address LUT
     * - Save the address range in the mapped ranges vector
     */
    void startTranslation(uint8_t entryID);

    /**
     * Method to start the address translation
     *
     * Complementary to startTranslation().
     */
    void stopTranslation(uint8_t entryID);

    /* Method to compute the mapped address range */
    std::pair<Addr, Addr> computeRange(entry_t entry);

    /* Methods to access CAT registers */
    uint64_t CATRegRead(Addr addr, Tick &delay);
    void CATRegWrite(Addr addr, uint64_t data, Tick &delay);

  public:
    PARAMS(CAT);
    CAT(const Params &params);

    /**
     * Implement BasicPioDevice virtual functions
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace cat

} // namespace gem5

#endif // __DEV_CAT_HH__
