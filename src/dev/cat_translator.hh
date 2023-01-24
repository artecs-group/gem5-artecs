/**
 * Compacting Address Translator (CAT)
 * -----------------------------------
 *
 * Description of CAT_REQ_REG:
 * 60         52    47               0
 * +==========+=====+================+
 * | ENTRY_ID | CMD | DATA / ADDRESS |
 * +==========+=====+================+
 *
 * Table of commands: see cat_cmd_interface.hh
 *
 * Description of CAT_RESP_REG:
 * 43       39        0
 * +========+=========+
 * | STATUS | ADDRESS |
 * +========+=========+
 *
 * List of CAT_RESP_REG statuses:
 * +======+====================+
 * | 0000 |     NOT MAPPED     |
 * +======+====================+
 * | 0101 |    COMMAND ACK     |
 * +======+====================+
 * | 1001 |    COMMAND NACK    |
 * +======+====================+
 * | 1010 |  MAPPED, NOT FOUND |
 * +======+====================+
 * | 1111 |   MAPPED, FOUND    |
 * +======+====================+
 */

#ifndef __DEV_CAT_HH__
#define __DEV_CAT_HH__

#include "debug/CAT.hh"
#include "dev/cat_cmd_interface.hh"
#include "dev/io_device.hh"
#include "params/CAT.hh"

namespace gem5
{

class CAT : public BasicPioDevice, CatCmdInterface
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
        Addr     start_addr = 0;          // Start address
        int16_t  seq_stride = 0;          // Sequential stride
        int16_t  iv2_stride = 0;          // Interleave 2 stride
        int16_t  iv2_offset = 0;          // Interleave 2 offset
        int16_t  iv3_stride = 0;          // Interleave 3 stride
        int16_t  iv3_offset = 0;          // Interleave 3 offset
        int16_t  ol_offset  = 0;          // Outer loop offset
        uint16_t ol_length  = 0;          // Outer loop length
        uint16_t length     = 0;          // Interval length
        Addr     tr_b_addr  = 0;          // Translated base address
        Tick     ready_time = 0;          // Ready time
        opmode_t mode = CAT_MOD_UNKNOWN;  // Operation mode
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
    uint64_t response;

    /* Ready time register */
    uint64_t ready_reg;

    /* Get the entry ID of a request */
    uint8_t getEntryID(uint64_t req) const {
        return (uint8_t)((req >> 53) & 0xff);
    };

    /* Get the payload of a request */
    uint64_t getPayload(uint64_t req) const {
        return req & 0xffffffffffff;
    }

    /**
     * Method to perform an address lookup
     *
     * It returns void since the result is
     * saved in the response register.
     */
    void lookup(uint64_t reg);

    /* Sets the status to the response register */
    void setStatus(status_t status);

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

} // namespace gem5

#endif // __DEV_CAT_HH__
