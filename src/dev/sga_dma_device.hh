/*
 * Modifications to DmaDevice class:
 * - Auto-write data from FIFO to destination
 * - Data scattering-gathering
 */

#ifndef __DEV_SGA_DMA_DEVICE_HH__
#define __DEV_SGA_DMA_DEVICE_HH__

#include <deque>
#include <memory>

#include "base/addr_range_map.hh"
#include "base/chunk_generator.hh"
#include "base/circlebuf.hh"
#include "dev/io_device.hh"
#include "mem/backdoor.hh"
#include "params/SgaDmaDevice.hh"
#include "sim/drain.hh"
#include "sim/system.hh"

namespace gem5
{

class ClockedObject;

class SgaDmaPort : public RequestPort, public Drainable
{
  private:
    AddrRangeMap<MemBackdoorPtr, 1> memBackdoors;

    /**
     * Take the first request on the transmit list and attempt to send a timing
     * packet from it. If it is successful, schedule the sending of the next
     * packet. Otherwise remember that we are waiting for a retry.
     */
    void trySendTimingReq();

    /**
     * For timing, attempt to send the first item on the transmit
     * list, and if it is successful and there are more packets
     * waiting, then schedule the sending of the next packet. For
     * atomic, simply send and process everything on the transmit
     * list.
     */
    void sendDma();

    struct SgaDmaReqState : public Packet::SenderState
    {
        /** Event to call on the device when this transaction (all packets)
         * complete. */
        Event *completionEvent;

        /** Total number of bytes that this transaction involves. */
        const Addr totBytes;

        /** Number of bytes that have been acked for this transaction. */
        Addr numBytes = 0;

        /** Amount to delay completion of dma by */
        const Tick delay;

        /** Object to track what chunks of bytes to send at a time. */
        ChunkGenerator gen;

        /** Pointer to a buffer for the data. */
        uint8_t *const data = nullptr;

        /** The flags to use for requests. */
        const Request::Flags flags;

        /** The requestor ID to use for requests. */
        const RequestorID id;

        /** Stream IDs. */
        const uint32_t sid;
        const uint32_t ssid;

        /** -- SGA-DMA Additions --. */
        Addr src;
        Addr dst;
        /** -----------------------. */

        SgaDmaReqState(Addr _src, Addr _dst, Addr chunk_sz,
                       Addr tb, uint8_t *_data, Request::Flags _flags,
                       RequestorID _id, uint32_t _sid, uint32_t _ssid,
                       Event *ce, Tick _delay)
            : completionEvent(ce), totBytes(tb), delay(_delay),
              gen(_src, tb, chunk_sz), data(_data), flags(_flags),
              id(_id), sid(_sid), ssid(_ssid), src(_src), dst(_dst)
        {}

        PacketPtr createPacket();
    };

    /**
     * Handle a response packet by updating the corresponding DMA
     * request state to reflect the bytes received, and also update
     * the pending request counter. If the DMA request that this
     * packet is part of is complete, then signal the completion event
     * if present, potentially with a delay added to it.
     *
     * @param pkt Response packet to handler
     * @param delay Additional delay for scheduling the completion event
     */
    void handleRespPacket(PacketPtr pkt, Tick delay=0);
    void handleResp(SgaDmaReqState *state, Addr addr, Addr size, Tick delay=0,
                    PacketPtr pkt = nullptr);

  public:
    /** The device that owns this port. */
    ClockedObject *const device;

    /** The system that device/port are in. This is used to select which mode
     * we are currently operating in. */
    System *const sys;

    /** Id for all requests */
    const RequestorID requestorId;

  protected:
    /** Use a deque as we never do any insertion or removal in the middle */
    std::deque<SgaDmaReqState *> transmitList;

    /** Event used to schedule a future sending from the transmit list. */
    EventFunctionWrapper sendEvent;

    /** Number of outstanding packets the dma port has. */
    uint32_t pendingCount = 0;

    /** The packet (if any) waiting for a retry to send. */
    PacketPtr inRetry = nullptr;
    PacketPtr inRetryDst = nullptr;

    /** Default streamId */
    const uint32_t defaultSid;

    /** Default substreamId */
    const uint32_t defaultSSid;

    const int cacheLineSize;

  protected:

    bool recvTimingResp(PacketPtr pkt) override;
    void recvReqRetry() override;

  public:

    SgaDmaPort(ClockedObject *dev, System *s, uint32_t sid=0, uint32_t ssid=0);

    void
    dmaAction(Addr src, Addr dst, int size, Event *event, uint8_t *data,
              Tick delay, Request::Flags flag=0);

    void
    dmaAction(Addr src, Addr dst, int size, Event *event, uint8_t *data,
              uint32_t sid, uint32_t ssid, Tick delay,
              Request::Flags flag=0);

    bool dmaPending() const { return pendingCount > 0; }

    DrainState drain() override;
};

class SgaDmaDevice : public PioDevice
{
   protected:
    SgaDmaPort dmaPort;

  public:
    typedef SgaDmaDeviceParams Params;
    SgaDmaDevice(const Params &p);
    virtual ~SgaDmaDevice() = default;

    bool dmaPending() const { return dmaPort.dmaPending(); }

    void init() override;

    unsigned int cacheBlockSize() const { return sys->cacheLineSize(); }

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

};

/**
 * Buffered DMA engine helper class
 *
 * This class implements a simple DMA engine that feeds a FIFO
 * buffer. The size of the buffer, the maximum number of pending
 * requests and the maximum request size are all set when the engine
 * is instantiated.
 *
 * An <i>asynchronous</i> transfer of a <i>block</i> of data
 * (designated by a start address and a size) is started by calling
 * the startFill() method. The DMA engine will aggressively try to
 * keep the internal FIFO full. As soon as there is room in the FIFO
 * for more data <i>and</i> there are free request slots, a new fill
 * will be started.
 *
 * The DMA engine allows new blocks to be requested as soon as the
 * last request for a block has been sent (i.e., there is no need to
 * wait for pending requests to complete). This can be queried with
 * the atEndOfBlock() method and more advanced implementations may
 * override the onEndOfBlock() callback.
 */
class SgaDmaFifo : public Drainable, public Serializable
{
  public:
    SgaDmaFifo(SgaDmaPort &port, size_t size,
               unsigned max_req_size,
               unsigned max_pending,
               Request::Flags flags = 0);

    ~SgaDmaFifo();

  public: // Serializable
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: // Drainable
    DrainState drain() override;

  public: // FIFO fill control
    /**
     * @{
     * @name FIFO fill control
     */
    /**
     * Start filling the FIFO.
     *
     * @warn It's considered an error to call start on an active DMA
     * engine unless the last request from the active block has been
     * sent (i.e., atEndOfBlock() is true).
     *
     * @param start Physical address to copy from.
     * @param dst Physical address to copy to.
     * @param size Size of the block to copy.
     */
    void startFill(Addr start, Addr dst, size_t size);

    /**
     * Stop the DMA engine.
     *
     * Stop filling the FIFO and ignore incoming responses for pending
     * requests. The onEndOfBlock() callback will not be called after
     * this method has been invoked. However, once the last response
     * has been received, the onIdle() callback will still be called.
     */
    void stopFill();

    /**
     * Has the DMA engine sent out the last request for the active
     * block?
     */
    bool atEndOfBlock() const { return nextAddr == endAddr; }

    /**
     * Is the DMA engine active (i.e., are there still in-flight
     * accesses)?
     */
    bool
    isActive() const
    {
        return !(pendingRequests.empty() && atEndOfBlock());
    }

    /** @} */
  protected: // Callbacks
    /**
     * @{
     * @name Callbacks
     */
    /**
     * End of block callback
     *
     * This callback is called <i>once</i> after the last access in a
     * block has been sent. It is legal for a derived class to call
     * startFill() from this method to initiate a transfer.
     */
    virtual void onEndOfBlock() {};

    /**
     * Last response received callback
     *
     * This callback is called when the DMA engine becomes idle (i.e.,
     * there are no pending requests).
     *
     * It is possible for a DMA engine to reach the end of block and
     * become idle at the same tick. In such a case, the
     * onEndOfBlock() callback will be called first. This callback
     * will <i>NOT</i> be called if that callback initiates a new DMA transfer.
     */
    virtual void onIdle() {};

    /** @} */
  private: // Configuration
    /** Maximum request size in bytes */
    const Addr maxReqSize;
    /** Maximum FIFO size in bytes */
    const size_t fifoSize;
    /** Request flags */
    const Request::Flags reqFlags;

    SgaDmaPort &port;

    const int cacheLineSize;

  private:
    class SgaDmaDoneEvent : public Event
    {
      public:
        SgaDmaDoneEvent(SgaDmaFifo *_parent, size_t max_size);

        void kill();
        void cancel();
        bool canceled() const { return _canceled; }
        void reset(size_t size);
        void process();

        bool done() const { return _done; }
        size_t requestSize() const { return _requestSize; }
        const uint8_t *data() const { return _data.data(); }
        uint8_t *data() { return _data.data(); }

      private:
        SgaDmaFifo *parent;
        bool _done = false;
        bool _canceled = false;
        size_t _requestSize;
        std::vector<uint8_t> _data;
    };

    typedef std::unique_ptr<SgaDmaDoneEvent> SgaDmaDoneEventUPtr;

    /**
     * DMA request done, handle incoming data and issue new
     * request.
     */
    void dmaDone();

    /** Handle pending requests that have been flagged as done. */
    void handlePending();

    /** Try to issue new DMA requests or bypass DMA requests*/
    void resumeFill();

    /** Try to issue new DMA requests during normal execution*/
    void resumeFillTiming();

    /** Try to bypass DMA requests in non-caching mode */
    void resumeFillBypass();

  private: // Internal state
    Fifo<uint8_t> buffer;

    Addr nextAddr = 0;
    Addr endAddr = 0;

    /** -- SGA-DMA Additions --. */
    Addr dstAddr = 0;
    /** -----------------------. */

    std::deque<SgaDmaDoneEventUPtr> pendingRequests;
    std::deque<SgaDmaDoneEventUPtr> freeRequests;
};

class SgaDmaCbFifo : public SgaDmaFifo
{
  private:
    ClockedObject *owner;
    Event *eotEvent;
    void onIdle() override;

  public:
    SgaDmaCbFifo(ClockedObject *device,
                 SgaDmaPort &port, size_t size,
                 unsigned max_req_size,
                 unsigned max_pending,
                 Request::Flags flags = 0,
                 Event *eot_event = nullptr);
};

} // namespace gem5

#endif // __DEV_SGA_DMA_DEVICE_HH__
