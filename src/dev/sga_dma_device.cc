/*
 * Copyright (c) 2012, 2015, 2017, 2019 ARM Limited
 * All rights reserved.
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

#include "dev/sga_dma_device.hh"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <utility>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/SgaDma.hh"
#include "sim/clocked_object.hh"
#include "sim/system.hh"

namespace gem5
{

SgaDmaPort::SgaDmaPort(ClockedObject *dev, System *s,
                 uint32_t sid, uint32_t ssid)
    : RequestPort(dev->name() + ".dma", dev),
      device(dev), sys(s), requestorId(s->getRequestorId(dev)),
      sendEvent([this]{ sendDma(); }, dev->name()),
      defaultSid(sid), defaultSSid(ssid), cacheLineSize(s->cacheLineSize())
{ }

void
SgaDmaPort::handleRespPacket(PacketPtr pkt, Tick delay)
{
    // Should always see a response with a sender state.
    assert(pkt->isResponse());

    // Get the DMA sender state.
    auto *state = dynamic_cast<SgaDmaReqState*>(pkt->senderState);
    assert(state);

    handleResp(state, pkt->getAddr(), pkt->req->getSize(), delay, pkt);

    delete pkt;
}

void
SgaDmaPort::handleResp(SgaDmaReqState *state, Addr addr, Addr size, Tick delay,
                       PacketPtr pkt)
{
    if (pkt && pkt->isRead()) {
        // To-do: generalize and use createPacket()
        const uint8_t *data = pkt->getConstPtr<uint8_t>();
        Addr dst_addr = state->dst + (addr - state->src);
        RequestPtr dst_req = std::make_shared<Request>(
            dst_addr, size, 0, state->id);
        dst_req->setStreamId(state->sid);
        dst_req->setSubstreamId(state->ssid);
        dst_req->taskId(context_switch_task_id::DMA);
        PacketPtr dst_pkt = new Packet(dst_req, MemCmd::WriteReq);
        dst_pkt->senderState = state;
        dst_pkt->allocate();
        dst_pkt->setData(data);

        DPRINTF(SgaDma, "Received response for addr %#x (src), forwarding " \
                "data to addr %#x (dst)\n", addr, dst_addr);

        if (!sendTimingReq(dst_pkt)) {
            assert(!inRetryDst);
            DPRINTF(SgaDma, "Data forwarding to addr %#x failed, adding to " \
                    "retry list\n", dst_addr);
            inRetryDst = dst_pkt;
        }

        return;
    }

    DPRINTF(SgaDma, "Received response for addr %#x (dst), size: %d, nb: %d," \
            " tot: %d, sched %d\n",
            addr, size,
            state->numBytes, state->totBytes,
            state->completionEvent ?
            state->completionEvent->scheduled() : 0);

    // Update the number of bytes received based on the request rather
    // than the packet as the latter could be rounded up to line sizes.
    state->numBytes += size;
    assert(state->totBytes >= state->numBytes);

    // If we have reached the total number of bytes for this DMA request,
    // then signal the completion and delete the sate.
    if (state->totBytes == state->numBytes) {
        assert(pendingCount != 0);
        pendingCount--;
        if (state->completionEvent) {
            delay += state->delay;
            device->schedule(state->completionEvent, curTick() + delay);
        }
        delete state;
    }

    // We might be drained at this point, if so signal the drain event.
    if (pendingCount == 0)
        signalDrainDone();
}

PacketPtr
SgaDmaPort::SgaDmaReqState::createPacket()
{
    RequestPtr req = std::make_shared<Request>(
            gen.addr(), gen.size(), flags, id);
    req->setStreamId(sid);
    req->setSubstreamId(ssid);
    req->taskId(context_switch_task_id::DMA);

    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

    if (data)
        pkt->dataStatic(data + gen.complete());

    pkt->senderState = this;
    return pkt;
}

bool
SgaDmaPort::recvTimingResp(PacketPtr pkt)
{
    // We shouldn't ever get a cacheable block in Modified state.
    assert(pkt->req->isUncacheable() ||
           !(pkt->cacheResponding() && !pkt->hasSharers()));

    handleRespPacket(pkt);

    return true;
}

SgaDmaDevice::SgaDmaDevice(const Params &p)
    : PioDevice(p), dmaPort(this, sys, p.sid, p.ssid)
{ }

void
SgaDmaDevice::init()
{
    panic_if(!dmaPort.isConnected(),
             "DMA port of %s not connected to anything!", name());
    PioDevice::init();
}

DrainState
SgaDmaPort::drain()
{
    if (pendingCount == 0) {
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "SgaDmaPort not drained\n");
        return DrainState::Draining;
    }
}

void
SgaDmaPort::recvReqRetry()
{
    if (inRetryDst) {
        DPRINTF(SgaDma, "Retrying data forward to address %#x\n",
                inRetryDst->getAddr());
        if (sendTimingReq(inRetryDst)) {
            inRetryDst = nullptr;
        }
    } else {
        assert(transmitList.size());
        trySendTimingReq();
    }
}

void
SgaDmaPort::dmaAction(Addr src, Addr dst, int size, Event *event,
                      uint8_t *data, uint32_t sid, uint32_t ssid, Tick delay,
                      Request::Flags flag)
{
    DPRINTF(SgaDma, "Starting DMA for addr: %#x size: %d sched: %d\n", src,
            size, event ? event->scheduled() : -1);

    // One DMA request sender state for every action, that is then
    // split into many requests and packets based on the block size,
    // i.e. cache line size.
    transmitList.push_back(
            new SgaDmaReqState(src, dst, cacheLineSize, size,
                data, flag, requestorId, sid, ssid, event, delay));
    pendingCount++;

    // In zero time, also initiate the sending of the packets for the request
    // we have just created. For atomic this involves actually completing all
    // the requests.
    sendDma();
}

void
SgaDmaPort::dmaAction(Addr src, Addr dst, int size, Event *event,
                      uint8_t *data, Tick delay, Request::Flags flag)
{
    dmaAction(src, dst, size, event, data,
              defaultSid, defaultSSid, delay, flag);
}

void
SgaDmaPort::trySendTimingReq()
{
    // Send the next packet for the first DMA request on the transmit list,
    // and schedule the following send if it is successful
    SgaDmaReqState *state = transmitList.front();

    PacketPtr pkt = inRetry ? inRetry : state->createPacket();
    inRetry = nullptr;

    DPRINTF(SgaDma, "Trying to send %s addr %#x\n", pkt->cmdString(),
            pkt->getAddr());

    // Check if this was the last packet now, since hypothetically the packet
    // response may come immediately, and state may be deleted.
    bool last = state->gen.last();
    if (!sendTimingReq(pkt))
        inRetry = pkt;
    if (!inRetry) {
        // If that was the last packet from this request, pop it from the list.
        if (last)
            transmitList.pop_front();
        else
            state->gen.next();
        DPRINTF(SgaDma, "-- Done\n");
        // If there is more to do, then do so.
        if (!transmitList.empty()) {
            // This should ultimately wait for as many cycles as the device
            // needs to send the packet, but currently the port does not have
            // any known width so simply wait a single cycle.
            device->schedule(sendEvent, device->clockEdge(Cycles(1)));
        }
    } else {
        DPRINTF(SgaDma, "-- Failed, waiting for retry\n");
    }

    DPRINTF(SgaDma, "TransmitList: %d, inRetry: %d\n",
            transmitList.size(), inRetry ? 1 : 0);
}

bool
SgaDmaPort::sendAtomicReq(SgaDmaReqState *state)
{
    panic("Only timing mode is supported");
}

bool
SgaDmaPort::sendAtomicBdReq(SgaDmaReqState *state)
{
    panic("Only timing mode is supported");
}

void
SgaDmaPort::sendDma()
{
    // Some kind of selection between access methods. More work is going to
    // have to be done to make switching actually work.
    assert(transmitList.size());

    if (sys->isTimingMode()) {
        // If we are either waiting for a retry or are still waiting after
        // sending the last packet, then do not proceed.
        if (inRetry || sendEvent.scheduled()) {
            DPRINTF(SgaDma, "Can't send immediately, waiting to send\n");
            return;
        }

        trySendTimingReq();
    } else if (sys->isAtomicMode()) {
        const bool bypass = sys->bypassCaches();

        // Send everything there is to send in zero time.
        while (!transmitList.empty()) {
            SgaDmaReqState *state = transmitList.front();
            transmitList.pop_front();

            bool done = state->gen.done();
            while (!done)
                done = bypass ? sendAtomicBdReq(state) : sendAtomicReq(state);
        }
    } else {
        panic("Unknown memory mode.");
    }
}

Port &
SgaDmaDevice::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "dma") {
        return dmaPort;
    }
    return PioDevice::getPort(if_name, idx);
}

SgaDmaFifo::SgaDmaFifo(SgaDmaPort &_port, size_t size,
                       unsigned max_req_size,
                       unsigned max_pending,
                       Request::Flags flags)
    : maxReqSize(max_req_size), fifoSize(size),
      reqFlags(flags), port(_port), cacheLineSize(port.sys->cacheLineSize()),
      buffer(size)
{
    freeRequests.resize(max_pending);
    for (auto &e : freeRequests)
        e.reset(new SgaDmaDoneEvent(this, max_req_size));

}

SgaDmaFifo::~SgaDmaFifo()
{
    for (auto &p : pendingRequests) {
        SgaDmaDoneEvent *e(p.release());

        if (e->done()) {
            delete e;
        } else {
            // We can't kill in-flight DMAs, so we'll just transfer
            // ownership to the event queue so that they get freed
            // when they are done.
            e->kill();
        }
    }
}

void
SgaDmaFifo::serialize(CheckpointOut &cp) const
{
    assert(pendingRequests.empty());

    SERIALIZE_CONTAINER(buffer);
    SERIALIZE_SCALAR(endAddr);
    SERIALIZE_SCALAR(nextAddr);
    SERIALIZE_SCALAR(dstAddr);
}

void
SgaDmaFifo::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_CONTAINER(buffer);
    UNSERIALIZE_SCALAR(endAddr);
    UNSERIALIZE_SCALAR(nextAddr);
    UNSERIALIZE_SCALAR(dstAddr);
}

void
SgaDmaFifo::startFill(Addr start, Addr dst, size_t size)
{
    assert(atEndOfBlock());

    nextAddr = start;
    endAddr = start + size;
    dstAddr = dst;
    resumeFill();
}

void
SgaDmaFifo::stopFill()
{
    // Prevent new DMA requests by setting the next address to the end
    // address. Pending requests will still complete.
    nextAddr = endAddr;

    // Flag in-flight accesses as canceled. This prevents their data
    // from being written to the FIFO.
    for (auto &p : pendingRequests)
        p->cancel();
}

void
SgaDmaFifo::resumeFill()
{
    // Don't try to fetch more data if we are draining. This ensures
    // that the DMA engine settles down before we checkpoint it.
    if (drainState() == DrainState::Draining)
        return;

    const bool old_eob(atEndOfBlock());

    if (port.sys->bypassCaches())
        resumeFillBypass();
    else
        resumeFillTiming();

    if (!old_eob && atEndOfBlock())
        onEndOfBlock();
}

void
SgaDmaFifo::resumeFillBypass()
{
    const size_t fifo_space = buffer.capacity() - buffer.size();
    if (fifo_space >= cacheLineSize || buffer.capacity() < cacheLineSize) {
        const size_t block_remaining = endAddr - nextAddr;
        const size_t xfer_size = std::min(fifo_space, block_remaining);
        std::vector<uint8_t> tmp_buffer(xfer_size);

        assert(pendingRequests.empty());
        DPRINTF(SgaDma, "Direct bypass startAddr=%#x xfer_size=%#x " \
                "fifo_space=%#x block_remaining=%#x\n",
                nextAddr, xfer_size, fifo_space, block_remaining);

        port.dmaAction(nextAddr, dstAddr, xfer_size, nullptr,
                       tmp_buffer.data(), 0, reqFlags);

        buffer.write(tmp_buffer.begin(), xfer_size);
        nextAddr += xfer_size;
        dstAddr  += xfer_size;
    }
}

void
SgaDmaFifo::resumeFillTiming()
{
    size_t size_pending(0);
    for (auto &e : pendingRequests)
        size_pending += e->requestSize();

    while (!freeRequests.empty() && !atEndOfBlock()) {
        const size_t req_size(std::min(maxReqSize, endAddr - nextAddr));
        if (buffer.size() + size_pending + req_size > fifoSize)
            break;

        SgaDmaDoneEventUPtr event(std::move(freeRequests.front()));
        freeRequests.pop_front();
        assert(event);

        event->reset(req_size);
        port.dmaAction(nextAddr, dstAddr, req_size, event.get(),
                       event->data(), 0, reqFlags);
        nextAddr += req_size;
        dstAddr  += req_size;
        size_pending += req_size;

        pendingRequests.emplace_back(std::move(event));
    }
}

void
SgaDmaFifo::dmaDone()
{
    const bool old_active(isActive());

    handlePending();
    resumeFill();

    if (old_active && !isActive())
        onIdle();
}

void
SgaDmaFifo::handlePending()
{
    while (!pendingRequests.empty() && pendingRequests.front()->done()) {
        // Get the first finished pending request
        SgaDmaDoneEventUPtr event(std::move(pendingRequests.front()));
        pendingRequests.pop_front();

        //if (!event->canceled())
        //    buffer.write(event->data(), event->requestSize());

        // Move the event to the list of free requests
        freeRequests.emplace_back(std::move(event));
    }

    if (pendingRequests.empty())
        signalDrainDone();
}

DrainState
SgaDmaFifo::drain()
{
    return pendingRequests.empty() ?
        DrainState::Drained : DrainState::Draining;
}


SgaDmaFifo::SgaDmaDoneEvent::SgaDmaDoneEvent(SgaDmaFifo *_parent,
                                             size_t max_size)
    : parent(_parent), _data(max_size, 0)
{
}

void
SgaDmaFifo::SgaDmaDoneEvent::kill()
{
    parent = nullptr;
    setFlags(AutoDelete);
}

void
SgaDmaFifo::SgaDmaDoneEvent::cancel()
{
    _canceled = true;
}

void
SgaDmaFifo::SgaDmaDoneEvent::reset(size_t size)
{
    assert(size <= _data.size());
    _done = false;
    _canceled = false;
    _requestSize = size;
}

void
SgaDmaFifo::SgaDmaDoneEvent::process()
{
    if (!parent)
        return;

    assert(!_done);
    _done = true;
    parent->dmaDone();
}

SgaDmaCbFifo::SgaDmaCbFifo(ClockedObject *device,
                           SgaDmaPort &port, size_t size,
                           unsigned max_req_size,
                           unsigned max_pending,
                           Request::Flags flags,
                           Event *eot_event)
    : SgaDmaFifo(port, size, max_req_size, max_pending, flags),
      owner(device),
      eotEvent(eot_event)
{ }

void
SgaDmaCbFifo::onIdle()
{
    if (eotEvent) {
        owner->schedule(eotEvent, curTick() + 1);
    }
}

} // namespace gem5
