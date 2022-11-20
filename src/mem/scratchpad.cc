/*
 * Copyright (c) 2010-2013, 2015 ARM Limited
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
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

#include "mem/scratchpad.hh"

#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/Scratchpad.hh"

namespace gem5
{

namespace memory
{

Scratchpad::Scratchpad(const ScratchpadParams &p) :
    AbstractMemory(p),
    port(name() + ".port", *this), readLatency(p.read_latency),
    writeLatency(p.write_latency), banksEnable(p.banks_enable),
    banksNumber(p.banks_number), banksIntlv(p.banks_intlv),
    banks(p.banks_number), isBusy(false), retryReq(false), retryResp(false),
    releaseEvent([this]{ release(); }, name()),
    dequeueEvent([this]{ dequeue(); }, name())
{
}

void
Scratchpad::init()
{
    AbstractMemory::init();

    // allow unconnected memories as this is used in several ruby
    // systems at the moment
    if (port.isConnected()) {
        port.sendRangeChange();
    }

    if (banksEnable)
    {
        if (!isPowerOf2(banksNumber))
            fatal("%s: number of banks is not a power of 2", __func__);
        if (!banksIntlv || banksIntlv > ceilLog2(size()) - 6)
            fatal("%s: interleave granularity is 0 or too big", __func__);

        for (unsigned i = 0; i < banks.size(); ++i) {
            banks[i] = new SimpleBank();
        }
    }
}

Tick
Scratchpad::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    access(pkt);
    return getLatency(pkt);
}

Tick
Scratchpad::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &_backdoor)
{
    Tick latency = recvAtomic(pkt);
    getBackdoor(_backdoor);
    return latency;
}

void
Scratchpad::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    functionalAccess(pkt);

    bool done = false;
    auto p = packetQueue.begin();
    // potentially update the packets in our packet queue as well
    while (!done && p != packetQueue.end()) {
        done = pkt->trySatisfyFunctional(p->pkt);
        ++p;
    }

    pkt->popLabel();
}

bool
Scratchpad::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(Scratchpad, "%s: receiving packet %s\n", __func__, pkt->print());

    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at memory controller, "
             "saw %s to %#llx\n", pkt->cmdString(), pkt->getAddr());

    // we should not get a new request after committing to retry the
    // current one, but unfortunately the CPU violates this rule, so
    // simply ignore it for now
    if (retryReq)
        return false;

    // if we are busy with a read or write, remember that we have to
    // retry
    if (isBusy) {
        retryReq = true;
        return false;
    }

    // technically the packet only reaches us after the header delay,
    // and since this is a memory controller we also need to
    // deserialise the payload before performing any write operation
    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;

    // save bank write delay to account in successive operations
    if (pkt->isWrite() && pkt->hasData() && banksEnable) {
        Tick endTick = curTick() + receive_delay + getLatency(pkt, false);
        unsigned bankID = getBankID(pkt->getAddr());
        Tick when_ready = banks[bankID]->getWhenReady();
        Tick tick = curTick() + pkt->headerDelay;
        if (when_ready > tick) {
            Tick new_ready = endTick + when_ready - tick;
            DPRINTF(Scratchpad, "%s: write request to bank %u (busy), "
                    "ending at tick %llu\n", __func__, bankID, new_ready);
            banks[bankID]->setWhenReady(new_ready);
        } else {
            DPRINTF(Scratchpad, "%s: write request to bank %u (idle), "
                    "ending at tick %llu\n", __func__, bankID, endTick);
            banks[bankID]->setWhenReady(endTick);
        }
    }

    // go ahead and deal with the packet and put the response in the
    // queue if there is one
    bool needsResponse = pkt->needsResponse();
    Tick latency = recvAtomic(pkt);
    // turn packet around to go back to requestor if response expected
    if (needsResponse) {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());

        Tick when_to_send = curTick() + receive_delay + latency;
        DPRINTF(Scratchpad, "%s: packet needs response, will be sent at tick "
                "%llu\n", __func__, when_to_send);

        // typically this should be added at the end, so start the
        // insertion sort with the last element, also make sure not to
        // re-order in front of some existing packet with the same
        // address, the latter is important as this memory effectively
        // hands out exclusive copies (shared is not asserted)
        auto i = packetQueue.end();
        --i;
        while (i != packetQueue.begin() && when_to_send < i->tick &&
               !i->pkt->matchAddr(pkt))
            --i;

        // emplace inserts the element before the position pointed to by
        // the iterator, so advance it one step
        packetQueue.emplace(++i, pkt, when_to_send);

        if (!retryResp && !dequeueEvent.scheduled())
            schedule(dequeueEvent, packetQueue.back().tick);
    } else {
        pendingDelete.reset(pkt);
    }

    return true;
}

unsigned
Scratchpad::getBankID(Addr addr)
{
    unsigned bankID = (addr >> 6) & ((1ULL << banksIntlv) - 1);
    DPRINTF(Scratchpad, "%s: mapping address %llu to bank %u\n",
            __func__, addr, bankID);
    return bankID;
}

void
Scratchpad::release()
{
    assert(isBusy);
    isBusy = false;
    if (retryReq) {
        retryReq = false;
        port.sendRetryReq();
    }
}

void
Scratchpad::dequeue()
{
    assert(!packetQueue.empty());
    DeferredPacket deferred_pkt = packetQueue.front();

    retryResp = !port.sendTimingResp(deferred_pkt.pkt);

    if (!retryResp) {
        packetQueue.pop_front();

        // if the queue is not empty, schedule the next dequeue event,
        // otherwise signal that we are drained if we were asked to do so
        if (!packetQueue.empty()) {
            // if there were packets that got in-between then we
            // already have an event scheduled, so use re-schedule
            reschedule(dequeueEvent,
                       std::max(packetQueue.front().tick, curTick()), true);
        } else if (drainState() == DrainState::Draining) {
            DPRINTF(Drain, "Draining of Scratchpad complete\n");
            signalDrainDone();
        }
    }
}

Tick
Scratchpad::getLatency(PacketPtr pkt, bool deferredWrite)
{
    Tick latency = 0;

    if (pkt->isWrite() && (!banksEnable || !deferredWrite)) {
        latency = cyclesToTicks(writeLatency);
    } else if (pkt->isRead()) {
        latency = cyclesToTicks(readLatency);
        if (banksEnable) {
            unsigned bankID = getBankID(pkt->getAddr());
            Tick when_ready = banks[bankID]->getWhenReady();
            Tick tick = curTick() + pkt->headerDelay;
            if (when_ready > tick) {
                // an additional latency is added to the next read operation
                // if some write is in progress
                latency += when_ready - tick;
            }
        }
    }

    return latency;
}

void
Scratchpad::recvRespRetry()
{
    assert(retryResp);

    dequeue();
}

Port &
Scratchpad::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "port") {
        return AbstractMemory::getPort(if_name, idx);
    } else {
        return port;
    }
}

DrainState
Scratchpad::drain()
{
    if (!packetQueue.empty()) {
        DPRINTF(Drain, "Scratchpad Queue has requests, waiting to drain\n");
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

Scratchpad::MemoryPort::MemoryPort(const std::string& _name,
                                     Scratchpad& _memory)
    : ResponsePort(_name, &_memory), mem(_memory)
{ }

AddrRangeList
Scratchpad::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(mem.getAddrRange());
    return ranges;
}

Tick
Scratchpad::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return mem.recvAtomic(pkt);
}

Tick
Scratchpad::MemoryPort::recvAtomicBackdoor(
        PacketPtr pkt, MemBackdoorPtr &_backdoor)
{
    return mem.recvAtomicBackdoor(pkt, _backdoor);
}

void
Scratchpad::MemoryPort::recvFunctional(PacketPtr pkt)
{
    mem.recvFunctional(pkt);
}

bool
Scratchpad::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    return mem.recvTimingReq(pkt);
}

void
Scratchpad::MemoryPort::recvRespRetry()
{
    mem.recvRespRetry();
}

} // namespace memory
} // namespace gem5
