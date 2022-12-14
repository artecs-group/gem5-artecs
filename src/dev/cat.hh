/*
 * Copyright (c) 2021 The Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Description of CAT_REQ_REG:
 *
 * 60         52    47               0
 * +==========+=====+================+
 * | ENTRY_ID | CMD | DATA / ADDRESS |
 * +==========+=====+================+
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
 *
 *
 * Description of CAT_REQ_RESP:
 *
 * 43       39        0
 * +========+=========+
 * | STATUS | ADDRESS |
 * +========+=========+
 *
 *
 * List of CAT_RESP_REG statuses:
 * +======+====================+
 * | 0000 |     NOT MAPPED     |
 * +======+====================+
 * | 0101 |    COMMAND ACK     |
 * +======+====================+
 * | 1010 |  MAPPED, NOT FOUND |
 * +======+====================+
 * | 1111 |   MAPPED, FOUND    |
 * +======+====================+
 */

#ifndef __DEV_CAT_HH__
#define __DEV_CAT_HH__

#include "debug/CAT.hh"
#include "dev/io_device.hh"
#include "params/CAT.hh"

namespace gem5
{

/**
 * Compacted Address Translator
 */
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
        CAT_SUB_START,
        CAT_SUB_STOP,
        CAT_SUB_UNKNOWN
    };

    /* Operation modes */
    enum opmode_t
    {
        CAT_SEQUENTIAL,
        CAT_INTLV_D2,
        CAT_INTLV_D3,
        CAT_MOD_UNKNOWN
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
        Addr     start_addr = 0;        // Start address
        int16_t  seq_stride = 0;        // Sequential stride
        int16_t  iv2_stride = 0;        // Interleave 2 stride
        int16_t  iv2_offset = 0;        // Interleave 2 offset
        int16_t  iv3_stride = 0;        // Interleave 3 stride
        int16_t  iv3_offset = 0;        // Interleave 3 offset
        int16_t  ol_offset  = 0;        // Outer loop offset
        uint16_t ol_length  = 0;        // Outer loop length
        uint16_t mode       = 0;        // Operation mode
        uint16_t length     = 0;        // Interval length
        Addr     tr_b_addr  = 0;        // Translated base address
        Tick     ready_time = 0;        // Ready time
        std::map<Addr, Addr> lut;       // Translation LUT
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

    /* Get the entry ID from a request */
    uint8_t getEntryID(uint64_t req) const {
        return (uint8_t)((req >> 53) & 0xff);
    };

    uint64_t getPayload(uint64_t req) const {
        return req & 0xffffffffffff;
    }

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

    /* Decode the subcommand of a start/stop request */
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

    opmode_t decodeMode(uint16_t mode) const {
        opmode_t opmode;
        switch (mode & 0xf) {
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
