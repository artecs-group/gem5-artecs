#include "dev/cat_translator.hh"

#include "base/trace.hh"
#include "debug/CAT.hh"
#include "mem/packet_access.hh"

namespace gem5
{

namespace cat
{

CAT::CAT(const Params &params) :
    BasicPioDevice(params, CAT_IOREG_ITEMS * sizeof(uint64_t)),
    configLat(params.config_lat),
    startLat(params.start_lat),
    lookupLat(params.lookup_lat),
    entries(params.entries),
    responseReg(0),
    readyTimeReg(0)
{ }

uint8_t
CAT::getEntryID(uint64_t req) const
{
    return (uint8_t)((req >> 53) & 0xff);
}

void
CAT::setStatus(status_t status)
{
    uint8_t value;
    switch (status) {
      case CAT_NOT_MAPPED:
        value = 0b0000;
        break;
      case CAT_CMD_ACK:
        value = 0b0101;
        break;
      case CAT_CMD_NACK:
        value = 0b1001;
        break;
      case CAT_NOT_FOUND:
        value = 0b1010;
        break;
      case CAT_FOUND:
        value = 0b1111;
        break;
      default:
        value = 0b0000;
        break;
    }
    responseReg &= ~((uint64_t)0b1111 << 40);
    responseReg |=  ((uint64_t)value  << 40);
}

void
CAT::lookup(Addr addr)
{
    responseReg = 0;

    // First lookup: check if address is in a mapped range
    auto it = std::find_if (
        ranges.begin(), ranges.end(),
        [&addr](const range_t& r) {
            return (addr >= r.addr_range.first &&
                    addr <= r.addr_range.second);
        });
    if (it != ranges.end()) {
        DPRINTF(CAT, "Found a corresponding range for address %#x -> "
                "entry %d\n", addr, 0);
        uint8_t entry_id = it->entry_id;
        const amap_t &lut = entries[entry_id].lut;
        readyTimeReg = entries[entry_id].ready_time;

        // Second lookup: check if there is an entry in the LUT
        auto ait = lut.find(lkAlign(addr));
        if (ait != lut.end()) {
            DPRINTF(CAT, "Address found in LUT, returning new address\n");
            responseReg = ait->second + lkOfs(addr);
            setStatus(CAT_FOUND);
        } else {
            DPRINTF(CAT, "Address not found in LUT, won't translate\n");
            setStatus(CAT_NOT_FOUND);
        }
    } else {
        //DPRINTF(CAT, "Address %#x does not belong to any mapped range\n",
        //        addr);
        setStatus(CAT_NOT_MAPPED);
    }
}

void
CAT::startTranslation(uint8_t entry_id) {
    entry_t& tgt = entries[entry_id];

    generateLut(tgt.p, tgt.lut);

    Addr min_addr = tgt.lut.begin()->first;
    Addr max_addr = tgt.lut.rbegin()->first;

    // Add address range to mapped ranges vector
    range_t r;
    r.addr_range = std::make_pair(min_addr, max_addr);
    r.entry_id   = entry_id;
    ranges.emplace_back(r);

    // Set entry ready time (to account for registers preparation)
    tgt.ready_time = curTick() + cyclesToTicks(startLat);
}

void
CAT::stopTranslation(uint8_t entry_id) {
    auto it = std::find_if (
        ranges.begin(), ranges.end(),
        [&entry_id](const range_t& r) {
            return (entry_id == r.entry_id);
        });
    if (it == ranges.end()) {
        panic("Entry %d of CAT is not active, cannot stop!", entry_id);
        return;
    }
    // Remove range from the list
    ranges.erase(it);

    // Clear CAT entry information
    entries[entry_id] = {};
}

uint64_t
CAT::CATRegRead(Addr addr, Tick &delay)
{
    // During a lookup, a read operation always follows a write.
    // Do not account for the delay twice.
    delay = 0;

    uint64_t r = 0;
    switch (addr / sizeof(uint64_t)) {
      case CAT_REQ_REG:
        r = -1;
        panic("CAT_REQ_REG is write-only!");
        break;
      case CAT_RESP_REG:
        r = responseReg;
        //DPRINTF(CAT, "Reading CAT_RESP_REG: %#x\n", r);
        break;
      case CAT_RDY_REG:
        r = readyTimeReg;
        //DPRINTF(CAT, "Reading CAT_RDY_REG: %#x\n", r);
        break;
      default:
        r = -1;
        panic("Unexpected read to CAT at address %lld!", addr);
        break;
    }
    return r;
}

void
CAT::CATRegWrite(Addr addr, uint64_t data, Tick &delay)
{
    bool process_req = false;
    switch (addr / sizeof(uint64_t)) {
      case CAT_REQ_REG:
        //DPRINTF(CAT, "Writing CAT_REQ_REG: %#x\n", data);
        process_req = true;
        break;
      case CAT_RESP_REG:
        panic("CAT_RESP_REG is read-only!");
        break;
      default:
        panic("Unexpected write to CAT at address %lld!", addr);
        break;
    }

    if (!process_req) {
        // Nothing to do. Should never reach this point anyway.
        return;
    }

    uint8_t  entry   = getEntryID(data);
    cmd_t    cmd     = decodeCmd(data);
    uint64_t payload = getPayload(data);

    delay = cyclesToTicks(configLat);

    std::string cmd_name = "CMD_UNKNOWN";
    bool success = setParams(data, entries[entry].p, cmd_name);
    if (success) {
        DPRINTF(CAT, "Received command CAT_%s, entry %d\n", cmd_name, entry);
        setStatus(CAT_CMD_ACK);
        return;
    }

    switch(cmd) {
      case CAT_NO_COMMAND:
        //DPRINTF(CAT, "No command received, performing normal lookup\n");
        lookup((Addr)payload);
        // Warning: lookup latency is ignored for now: using functional access
        delay = cyclesToTicks(lookupLat);
        break;

      case CAT_START_STOP:
        DPRINTF(CAT, "Received command CAT_START_STOP, entry %d\n", entry);
        switch(decodeStartStop(payload)) {
          case CAT_SUB_START:
            DPRINTF(CAT, "Command is START, beginning translation\n");
            startTranslation(entry);
            setStatus(CAT_CMD_ACK);
            break;

          case CAT_SUB_STOP:
            DPRINTF(CAT, "Command is STOP, interrupting translation\n");
            stopTranslation(entry);
            setStatus(CAT_CMD_ACK);
            break;

          default:
            panic("Command in neither START or STOP!");
            setStatus(CAT_CMD_NACK);
            break;
        }
        break;

      default:
        panic("Command is invalid!");
        setStatus(CAT_CMD_NACK);
        break;
    }
}

Tick
CAT::read(PacketPtr pkt)
{
    Addr pkt_addr = pkt->getAddr();
    if (pkt_addr < pioAddr || pkt_addr >= pioAddr + pioSize) {
        panic("Address %#x is out of CAT address range!", pkt_addr);
        return 1;
    }

    Addr cat_addr = pkt_addr - pioAddr;

    //DPRINTF(CAT,
    //    "Read request - addr: %#x, size: %#x\n", cat_addr, pkt->getSize());

    Tick delay = 0;
    uint64_t read = CATRegRead(cat_addr, delay);
    //DPRINTF(CAT, "Packet Read: %#x\n", read);
    pkt->setUintX(read, byteOrder);
    pkt->makeResponse();

    return delay;
}

Tick
CAT::write(PacketPtr pkt)
{
    Addr pkt_addr = pkt->getAddr();
    if (pkt_addr < pioAddr || pkt_addr >= pioAddr + pioSize) {
        panic("Address %#x is out of CAT address range!", pkt_addr);
        return 1;
    }

    Addr cat_addr = pkt_addr - pioAddr;

    //DPRINTF(CAT, "Write register %#x value %#x\n", cat_addr,
    //        pkt->getUintX(byteOrder));

    Tick delay = 0;
    CATRegWrite(cat_addr, pkt->getUintX(byteOrder), delay);
    //DPRINTF(CAT, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));

    pkt->makeResponse();

    return delay;
}

} // namespace cat

} // namespace gem5
