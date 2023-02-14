/**
 * Common command interface for Compacting Address Translator
 * and Scattering-Gathering Aware DMA Controller
 * ----------------------------------------------------------
 *
 * Description of request (least significant part):
 * 52    47               0
 * +=====+================+
 * | CMD | DATA / ADDRESS |
 * +=====+================+
 *
 * Table of commands:
 * +=======+====================+
 * | 11000 |      ST_ADDR       |
 * +=======+======+======+======+
 * | 11001 |  ND  | STR2 | OFS2 |
 * +=======+======+======+======+
 * | 11010 |  ND  | STR3 | OFS3 |
 * +=======+======+======+======+
 * | 11011 |  ND  | OOFS | OLEN |
 * +=======+======+======+======+
 * | 11100 | STR1 | MODE | LEN  |
 * +=======+======+======+======+
 * | 11101 |     TR_B_ADDR      |
 * +=======+====================+
 * | 11111 |   START / STOP     |
 * +=======+====================+
 */

#ifndef __DEV_CAT_CMD_INTERFACE_HH__
#define __DEV_CAT_CMD_INTERFACE_HH__

#include <map>

#include "base/types.hh"

namespace gem5
{

class CatCmdInterface
{
  public:
    /* Operation modes */
    enum opmode_t
    {
        CAT_MOD_UNKNOWN,
        CAT_SEQUENTIAL,
        CAT_INTLV_D2,
        CAT_INTLV_D3
    };

    /* Translation parameters */
    struct params_t
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
        opmode_t mode = CAT_MOD_UNKNOWN;  // Operation mode
    };

  protected:
    /* Request commands */
    enum cmd_t
    {
        CAT_NO_COMMAND,
        CAT_SET_ST_ADDR,
        CAT_SET_INTLV_D2,
        CAT_SET_INTLV_D3,
        CAT_SET_OLOOP,
        CAT_SET_STR_MODE,
        CAT_SET_TR_B_ADDR,
        CAT_START_STOP
    };

    /* Start/stop subcommands */
    enum subcmd_t
    {
        CAT_SUB_UNKNOWN,
        CAT_SUB_START,
        CAT_SUB_STOP
    };

    /* Decode the command of a request */
    cmd_t decodeCmd(uint64_t req) const;

    /* Decode the subcommand of a CAT_START_STOP request */
    subcmd_t decodeStartStop(uint64_t req) const;

    /* Decode the operation mode of a CAT_SET_STR_MODE request */
    opmode_t decodeMode(uint64_t req) const;

    /* Get the payload of a request */
    uint64_t getPayload(uint64_t req) const;

    /* Set parameters according to received request */
    bool setParams(uint64_t req, params_t &p, std::string &cmd_name);

    /**
     * Generate an address LUT given the translation parameters.
     * @return the number of bytes of the compacted elements
     */
    unsigned generateLut(params_t p, std::map<Addr, Addr> &lut);

    /* Protected constructor */
    CatCmdInterface() { };
};

} // namespace gem5

#endif /* __DEV_CAT_CMD_INTERFACE_HH__ */
