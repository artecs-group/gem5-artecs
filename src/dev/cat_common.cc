#include "dev/cat_common.hh"

#include "base/logging.hh"

namespace gem5
{

namespace cat
{

cmd_t decodeCmd(uint64_t req) {
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
}

subcmd_t decodeStartStop(uint64_t req) {
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

opmode_t decodeMode(uint64_t req) {
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

uint64_t getPayload(uint64_t req) {
    return req & 0xffffffffffff;
}

bool setParams(uint64_t req, params_t &p, std::string &cmd_name) {
    cmd_t    cmd     = decodeCmd(req);
    uint64_t payload = getPayload(req);

    bool success;
    switch(cmd) {
      case CAT_SET_ST_ADDR:
        p.start_addr = (Addr)payload;
        cmd_name     = "SET_ST_ADDR";
        success      = true;
        break;

      case CAT_SET_INTLV_D2:
        p.iv2_stride = (payload >> 16) & 0xffff;
        p.iv2_offset = payload & 0xffff;
        cmd_name     = "SET_INTLV_D2";
        success      = true;
        break;

      case CAT_SET_INTLV_D3:
        p.iv3_stride = (payload >> 16) & 0xffff;
        p.iv3_offset = payload & 0xffff;
        cmd_name     = "SET_INTLV_D3";
        success      = true;
        break;

      case CAT_SET_OLOOP:
        p.ol_offset  = (payload >> 16) & 0xffff;
        p.ol_length  = payload & 0xffff;
        cmd_name     = "SET_OLOOP";
        success      = true;
        break;

      case CAT_SET_STR_MODE:
        p.seq_stride = (payload >> 32) & 0xffff;
        p.mode       = decodeMode(payload);
        p.length     = payload & 0xffff;
        cmd_name     = "SET_STR_MODE";
        success      = true;
        break;

      case CAT_SET_TR_B_ADDR:
        p.tr_b_addr  = (Addr)payload;
        cmd_name     = "SET_TR_B_ADDR";
        success      = true;
        break;

      default:
        success      = false;
        break;
    }
    return success;
}

unsigned generateLut(params_t p, std::map<Addr, Addr> &lut) {
    lut.clear();

    std::array<int16_t, 3> strides = { p.seq_stride,
                                       p.iv2_stride,
                                       p.iv3_stride };
    std::array<Addr, 3> start_addr = { p.start_addr,
                                       p.start_addr + p.iv2_offset,
                                       p.start_addr + p.iv3_offset };
    std::array<Addr, 3> cur_addr = start_addr;
    Addr out_addr = p.tr_b_addr;

    uint16_t length = p.length;
    if (!length) {
        panic("The length of the interval must not be 0!");
        return 0;
    }

    uint8_t intlv = 0;
    switch (p.mode) {
      case CAT_SEQUENTIAL:
        intlv = 1;
        break;
      case CAT_INTLV_D2:
        intlv = 2;
        break;
      case CAT_INTLV_D3:
        intlv = 3;
        break;
      default:
        intlv = 0;
        panic("Specified mode is not valid!");
        return 0;
        break;
    }
    uint16_t ol_length = (p.ol_length < 1 ? 1 : p.ol_length);

    /* Create the LUT with every possible combination of addresses */
    for (uint16_t ol_iter = 0; ol_iter < ol_length; ol_iter++) {
        uint16_t elem = 0;
        // Inner loop
        while (elem < length) {
            // Interleaving control
            for (uint8_t i = 0; (i < intlv && elem++ < length); i++) {
                if (!lut.count(cur_addr[i])) {
                    lut[cur_addr[i]] = out_addr;
                    // Keep granularity fixed to 64 bits for now
                    out_addr += sizeof(uint64_t);
                }
                cur_addr[i] = int64_t(cur_addr[i]) + strides[i];
            }
        }
        // End of inner loop, add outer loop offset
        for (uint8_t i = 0; i < intlv; i++) {
            start_addr[i] = int64_t(start_addr[i]) + p.ol_offset;
        }
        cur_addr = start_addr;
    }

    return out_addr;
}

} // namespace cat

} // namespace gem5
