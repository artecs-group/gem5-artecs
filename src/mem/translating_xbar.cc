/*
 * Copyright (c) 2011-2015, 2018-2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * @file
 * Definition of a translating crossbar object.
 */

#include "mem/translating_xbar.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/TranslatingXBar.hh"
#include "debug/XBar.hh"
#include "mem/packet_access.hh"

namespace gem5
{

TranslatingXBar::TranslatingXBar(const TranslatingXBarParams &p)
    : BaseXBar(p)
{
    // create the ports based on the size of the memory-side port and
    // CPU-side port vector ports, and the presence of the default port,
    // the ports are enumerated starting from zero
    for (int i = 0; i < p.port_mem_side_ports_connection_count; ++i) {
        std::string portName = csprintf("%s.mem_side_port[%d]", name(), i);
        RequestPort* bp = new TranslatingXBarRequestPort(portName, *this, i);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this,
                                         csprintf("reqLayer%d", i)));
    }

    // see if we have a default CPU-side-port device connected and if so add
    // our corresponding memory-side port
    if (p.port_default_connection_count) {
        defaultPortID = memSidePorts.size();
        std::string portName = name() + ".default";
        RequestPort* bp = new TranslatingXBarRequestPort(portName, *this,
                                                      defaultPortID);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this, csprintf("reqLayer%d",
                                                              defaultPortID)));
    }

    // create the address translator port
    {
        translatorPortID = memSidePorts.size();
        std::string portName = csprintf("%s.translator_port", name());
        RequestPort* bp = new TranslatingXBarRequestPort(portName, *this,
                                                         translatorPortID);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this, csprintf("reqLayer%d",
                                                     translatorPortID)));
    }

    // create the DMA controller port
    {
        dmacPortID = memSidePorts.size();
        std::string portName = csprintf("%s.dmac_port", name());
        RequestPort* bp = new TranslatingXBarRequestPort(portName, *this,
                                                         dmacPortID);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this, csprintf("reqLayer%d",
                                                              dmacPortID)));
    }

    // create the CPU-side ports, once again starting at zero
    for (int i = 0; i < p.port_cpu_side_ports_connection_count; ++i) {
        std::string portName = csprintf("%s.cpu_side_ports[%d]", name(), i);
        QueuedResponsePort* bp = new TranslatingXBarResponsePort(portName,
                                                                *this, i);
        cpuSidePorts.push_back(bp);
        respLayers.push_back(new RespLayer(*bp, *this,
                                           csprintf("respLayer%d", i)));
    }
}

TranslatingXBar::~TranslatingXBar()
{
    for (auto l: reqLayers)
        delete l;
    for (auto l: respLayers)
        delete l;
}

Port &
TranslatingXBar::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side_ports" && idx < memSidePorts.size()) {
        // the memory-side ports index translates directly to the vector
        // position
        return *memSidePorts[idx];
    } else  if (if_name == "default") {
        return *memSidePorts[defaultPortID];
    } else  if (if_name == "translator_port") {
        return *memSidePorts[translatorPortID];
    } else  if (if_name == "dmac_port") {
        return *memSidePorts[dmacPortID];
    } else if (if_name == "cpu_side_ports" && idx < cpuSidePorts.size()) {
        // the CPU-side ports index translates directly to the vector position
        return *cpuSidePorts[idx];
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

PacketPtr
TranslatingXBar::createPacket(MemCmd cmd, Addr addr, uint64_t data)
{
    RequestPtr req;
    PacketPtr  pkt;
    req = std::make_shared<Request>(
        addr,
        sizeof(uint64_t),
        0,
        Request::funcRequestorId);
    pkt = new Packet(req, cmd);
    pkt->allocate();
    if (cmd == MemCmd::WriteReq) {
        pkt->set(data, ByteOrder::little);
    }
    return pkt;
}

bool
TranslatingXBar::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id)
{
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    // we should never see express snoops on a translating crossbar
    assert(!pkt->isExpressSnoop());

    // write request to addresses >= 2^61 are used to program the
    // address translator and dma controller. The address itself
    // is used as data after clearing the first 3 bits. This workaround
    // allows to program the translator when using elastic traces, which
    // do not contain data
    Addr pkt_addr = pkt->getAddr();
    AddrRangeList translator_range =
        memSidePorts[translatorPortID]->getAddrRanges();
    AddrRangeList dmac_range = memSidePorts[dmacPortID]->getAddrRanges();
    Addr at_base_addr = translator_range.front().start();
    Addr dmac_base_addr = dmac_range.front().start();

    uint8_t high_bits = pkt_addr >> 61;
    if (high_bits && pkt->isWrite()) {
        uint64_t data = pkt_addr & 0x1fffffffffffffff;
        if (high_bits == 0b001) {
            DPRINTF(TranslatingXBar, "Found translator programming command\n");
            pkt->setAddr(at_base_addr);
            pkt->set(data, ByteOrder::little);
        } else if (high_bits == 0b010) {
            DPRINTF(TranslatingXBar, "Found DMAC programming command\n");
            pkt->setAddr(dmac_base_addr);
            pkt->set(data, ByteOrder::little);
        }
    }

    // determine the destination based on the address
    PortID mem_side_port_id = findPort(pkt->getAddrRange());

    // check if we need an address translation
    Tick trans_delay = 0;
    if (mem_side_port_id != translatorPortID &&
        mem_side_port_id != dmacPortID &&
        pkt->req->taskId() != context_switch_task_id::DMA) {
        RequestPtr q_req;
        PacketPtr  q_pkt;

        // write address to the first register of the translator
        q_pkt = createPacket(MemCmd::WriteReq, at_base_addr, pkt_addr);
        memSidePorts[translatorPortID]->sendFunctional(q_pkt);
        delete q_pkt;

        // read response from the second register of the translator
        q_pkt = createPacket(MemCmd::ReadReq, at_base_addr + sizeof(uint64_t));
        memSidePorts[translatorPortID]->sendFunctional(q_pkt);
        uint64_t q_ans = q_pkt->getUintX(ByteOrder::little);
        delete q_pkt;

        // read ready time from the third register of the translator
        q_pkt = createPacket(MemCmd::ReadReq,
                             at_base_addr + (2 * sizeof(uint64_t)));
        memSidePorts[translatorPortID]->sendFunctional(q_pkt);
        uint64_t q_rdy_time = q_pkt->getUintX(ByteOrder::little);
        delete q_pkt;
        Tick tick = curTick() + pkt->headerDelay;
        if (q_rdy_time > tick) {
            DPRINTF(TranslatingXBar, "Entry will be ready at tick %lld, "
                    "adding extra delay\n", q_rdy_time);
            trans_delay = q_rdy_time - tick;
        }

        // to-do: make this information less hardcoded
        uint8_t status = (q_ans >> 40) & 0xf;
        Addr tr_addr = q_ans & 0xffffffffff;
        switch (status) {
          case 0b1111:
            DPRINTF(TranslatingXBar, "Translating address %#x to %#x\n",
                    pkt_addr, tr_addr);
            pkt->setAddr(tr_addr);
            // update destination according to the new address
            mem_side_port_id = findPort(pkt->getAddrRange());
            break;
          case 0b1010:
            DPRINTF(TranslatingXBar, "Address %#x belongs to a mapped range "
                    "but is not translatable\n", pkt_addr);
            // to-do: uncached access
            break;
          case 0b0000:
            DPRINTF(TranslatingXBar, "Address %#x does not belong to any "
                    "mapped range, not translating\n", pkt_addr);
            break;
          default:
            panic("Not a valid after-lookup status");
            break;
        }
    }

    // test if the layer should be considered occupied for the current
    // port
    if (!reqLayers[mem_side_port_id]->tryTiming(src_port)) {
        DPRINTF(TranslatingXBar, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(TranslatingXBar, "recvTimingReq: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // store the old header delay so we can restore it if needed
    Tick old_header_delay = pkt->headerDelay;

    // a request sees the frontend and forward latency
    Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // add the extra translation delay, if the entry was not ready
    pkt->headerDelay += trans_delay;

    // determine how long to be crossbar layer is busy
    //Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // before forwarding the packet (and possibly altering it),
    // remember if we are expecting a response
    const bool expect_response = pkt->needsResponse() &&
        !pkt->cacheResponding();

    // since it is a normal request, attempt to send the packet
    bool success = memSidePorts[mem_side_port_id]->sendTimingReq(pkt);

    if (!success)  {
        DPRINTF(TranslatingXBar, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        // restore the header delay as it is additive
        pkt->headerDelay = old_header_delay;

        // occupy until the header is sent
        reqLayers[mem_side_port_id]->failedTiming(src_port, 0);

        return false;
    }

    // remember where to route the response to
    if (expect_response) {
        assert(routeTo.find(pkt->req) == routeTo.end());
        routeTo[pkt->req] = cpu_side_port_id;
    }

    reqLayers[mem_side_port_id]->succeededTiming(0);

    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

bool
TranslatingXBar::recvTimingResp(PacketPtr pkt, PortID mem_side_port_id)
{
    // determine the source port based on the id
    RequestPort *src_port = memSidePorts[mem_side_port_id];

    // determine the destination
    const auto route_lookup = routeTo.find(pkt->req);
    assert(route_lookup != routeTo.end());
    const PortID cpu_side_port_id = route_lookup->second;
    assert(cpu_side_port_id != InvalidPortID);
    assert(cpu_side_port_id < respLayers.size());

    // test if the layer should be considered occupied for the current
    // port
    if (!respLayers[cpu_side_port_id]->tryTiming(src_port)) {
        DPRINTF(TranslatingXBar, "recvTimingResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(TranslatingXBar, "recvTimingResp: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // a response sees the response latency
    Tick xbar_delay = responseLatency * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    //Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // send the packet through the destination CPU-side port, and pay for
    // any outstanding latency
    Tick latency = pkt->headerDelay;
    pkt->headerDelay = 0;
    cpuSidePorts[cpu_side_port_id]->schedTimingResp(pkt,
                                        curTick() + latency);

    // remove the request from the routing table
    routeTo.erase(route_lookup);

    respLayers[cpu_side_port_id]->succeededTiming(0);

    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

void
TranslatingXBar::recvReqRetry(PortID mem_side_port_id)
{
    // responses never block on forwarding them, so the retry will
    // always be coming from a port to which we tried to forward a
    // request
    reqLayers[mem_side_port_id]->recvRetry();
}

Tick
TranslatingXBar::recvAtomicBackdoor(PacketPtr pkt, PortID cpu_side_port_id,
                                    MemBackdoorPtr *backdoor)
{
    DPRINTF(TranslatingXBar, "recvAtomic: packet src %s addr 0x%x cmd %s\n",
            cpuSidePorts[cpu_side_port_id]->name(), pkt->getAddr(),
            pkt->cmdString());

    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // determine the destination port
    PortID mem_side_port_id = findPort(pkt->getAddrRange());

    // stats updates for the request
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    // forward the request to the appropriate destination
    auto mem_side_port = memSidePorts[mem_side_port_id];
    Tick response_latency = backdoor ?
        mem_side_port->sendAtomicBackdoor(pkt, *backdoor) :
        mem_side_port->sendAtomic(pkt);

    // add the response data
    if (pkt->isResponse()) {
        pkt_size = pkt->hasData() ? pkt->getSize() : 0;
        pkt_cmd = pkt->cmdToIndex();

        // stats updates
        pktCount[cpu_side_port_id][mem_side_port_id]++;
        pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
        transDist[pkt_cmd]++;
    }

    // @todo: Not setting first-word time
    pkt->payloadDelay = response_latency;
    return response_latency;
}

void
TranslatingXBar::recvFunctional(PacketPtr pkt, PortID cpu_side_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(TranslatingXBar,
                "recvFunctional: packet src %s addr 0x%x cmd %s\n",
                cpuSidePorts[cpu_side_port_id]->name(), pkt->getAddr(),
                pkt->cmdString());
    }

    // since our CPU-side ports are queued ports we need to check them as well
    for (const auto& p : cpuSidePorts) {
        // if we find a response that has the data, then the
        // downstream caches/memories may be out of date, so simply stop
        // here
        if (p->trySatisfyFunctional(pkt)) {
            if (pkt->needsResponse())
                pkt->makeResponse();
            return;
        }
    }

    // determine the destination port
    PortID dest_id = findPort(pkt->getAddrRange());

    // forward the request to the appropriate destination
    memSidePorts[dest_id]->sendFunctional(pkt);
}

} // namespace gem5
