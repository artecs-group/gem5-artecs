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
 * | 11001 | STR2 |    OFS2     |   STR2: 16bit, OFS2: 32bit
 * +=======+======+======+======+
 * | 11010 | STR3 |    OFS3     |   STR3: 16bit, OFS3: 32bit
 * +=======+======+======+======+
 * | 11011 |    OOFS     | OLEN |   OOFS: 32bit, OLEN: 16bit
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

#include <map>

#include "base/bimap.hh"
#include "base/types.hh"

namespace gem5
{

namespace  cat
{

typedef std::pair<Addr, Addr> apair_t;
typedef codeproject::bimap<Addr, Addr> amap_t;
typedef amap_t::const_iterator amap_it_t;

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
    int32_t  iv2_offset = 0;          // Interleave 2 offset
    int16_t  iv3_stride = 0;          // Interleave 3 stride
    int32_t  iv3_offset = 0;          // Interleave 3 offset
    int32_t  ol_offset  = 0;          // Outer loop offset
    uint16_t ol_length  = 0;          // Outer loop length
    uint16_t length     = 0;          // Interval length
    Addr     tr_b_addr  = 0;          // Translated base address
    opmode_t mode = CAT_MOD_UNKNOWN;  // Operation mode
};

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
cmd_t decodeCmd(uint64_t req);

/* Decode the subcommand of a CAT_START_STOP request */
subcmd_t decodeStartStop(uint64_t req);

/* Decode the operation mode of a CAT_SET_STR_MODE request */
opmode_t decodeMode(uint64_t req);

/* Get the payload of a request */
uint64_t getPayload(uint64_t req);

/* Set parameters according to received request */
bool setParams(uint64_t req, params_t &p, std::string &cmd_name);

/* Generate an address LUT given the translation parameters. */
void generateLut(params_t p, amap_t &lut, uint32_t limit = 0);

/* Align the address to word size for lut lookup */
Addr lkAlign(Addr addr);

/* Get the offset within the word for smaller accesses */
uint8_t lkOfs(Addr addr);

} // namespace cat

} // namespace gem5

#endif /* __DEV_CAT_COMMON_HH__ */
