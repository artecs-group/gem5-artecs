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

#ifndef __DEV_CAT_COMMON_HH__
#define __DEV_CAT_COMMON_HH__

namespace gem5
{

class CatCmdInterface
{
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

    /* Operation modes */
    enum opmode_t
    {
        CAT_MOD_UNKNOWN,
        CAT_SEQUENTIAL,
        CAT_INTLV_D2,
        CAT_INTLV_D3
    };

    /* Decode the command of a request */
    cmd_t decodeCmd(uint64_t req) const {
        cmd_t cmd;
        switch ((req >> 48) & 0x1f) {
          case 0b11000:
            cmd = CAT_SET_ST_ADDR;
            break;
          case 0b11001:
            cmd = CAT_SET_INTLV_D2;
            break;
          case 0b11010:
            cmd = CAT_SET_INTLV_D3;
            break;
          case 0b11011:
            cmd = CAT_SET_OLOOP;
            break;
          case 0b11100:
            cmd = CAT_SET_STR_MODE;
            break;
          case 0b11101:
            cmd = CAT_SET_TR_B_ADDR;
            break;
          case 0b11111:
            cmd = CAT_START_STOP;
            break;
          default:
            cmd = CAT_NO_COMMAND;
        }
        return cmd;
    };

    /* Decode the subcommand of a CAT_START_STOP request */
    subcmd_t decodeStartStop(uint64_t req) const {
        subcmd_t subcmd;
        switch (req & 0xff) {
          case 0b10100101:
            subcmd = CAT_SUB_START;
            break;
          case 0b01011010:
            subcmd = CAT_SUB_STOP;
            break;
          default:
            subcmd = CAT_SUB_UNKNOWN;
        }
        return subcmd;
    };

    /* Decode the operation mode of a CAT_SET_STR_MODE request */
    opmode_t decodeMode(uint64_t req) const {
        opmode_t opmode;
        switch ((req >> 16) & 0xf) {
          case 0b1100:
            opmode = CAT_SEQUENTIAL;
            break;
          case 0b1101:
            opmode = CAT_INTLV_D2;
            break;
          case 0b1111:
            opmode = CAT_INTLV_D3;
            break;
          default:
            opmode = CAT_MOD_UNKNOWN;
        }
        return opmode;
    }
};

} // namespace gem5

#endif /* __DEV_CAT_COMMON_HH__ */
