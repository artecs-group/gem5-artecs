#include "dev/cat_cmd_interface.hh"

namespace gem5
{

CatCmdInterface::cmd_t
CatCmdInterface::decodeCmd(uint64_t req) const
{
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

CatCmdInterface::subcmd_t
CatCmdInterface::decodeStartStop(uint64_t req) const
{
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

CatCmdInterface::opmode_t
CatCmdInterface::decodeMode(uint64_t req) const
{
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

uint64_t
CatCmdInterface::getPayload(uint64_t req) const {
    return req & 0xffffffffffff;
}

bool
CatCmdInterface::setParams(uint64_t req, params_t &p, std::string &cmd_name)
{
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

} // namespace gem5
