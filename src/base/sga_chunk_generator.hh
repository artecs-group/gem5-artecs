#ifndef __BASE_SGA_CHUNK_GENERATOR_HH__
#define __BASE_SGA_CHUNK_GENERATOR_HH__

/**
 * @file
 * Declaration and inline definition of SgaChunkGenerator object.
 */

#include <algorithm>
#include <cassert>

#include "base/intmath.hh"
#include "base/types.hh"
#include "dev/cat_common.hh"

namespace gem5
{

namespace cat
{

/**
 * Modified ChunkGenerator for scattering/gathering support.
 */
class SgaChunkGenerator
{
  private:
    /** The iterator corresponding to the current chunk entry point. */
    amap_it_t curEntry;
    /** The iterator corresponding to the next chunk entry point. */
    amap_it_t nextEntry;
    /** The iterator corresponding to the limit entry point. */
    amap_it_t endEntry;
    /** The size of the current chunk (in bytes). */
    Addr curSize;
    /** The total number of bytes of the region. */
    Addr totalSize;
    /** The number of bytes remaining in the region after the current chunk. */
    Addr sizeLeft;
    /** The address translation lut. */
    const amap_t &lut;
    /** The iterator corresponding to the first entry point. */
    const amap_it_t start;
    /** The number of entries to process. */
    const uint16_t length;
    /** Direction flag (true: scattering, false: gathering) */
    const bool scattering;
    /** The maximum chunk size, e.g., the cache block size. */
    const Addr chunkSize;
    /** The chunk aligment, e.g., the dram page size. */
    const Addr chunkAlign;

    Addr _getAddr(amap_it_t it) const {
        if (scattering) {
            return it->second;
        }
        return it->first;
    }

    void _calcNext() {
        assert(nextEntry != endEntry);
        curEntry = nextEntry;
        curSize = 0;
        Addr pageNum = (curEntry->first / chunkAlign);
        while (nextEntry != endEntry && curSize < chunkSize &&
               (nextEntry->first / chunkAlign) == pageNum) {
            curSize += sizeof(uint64_t);
            nextEntry++;
        }
        sizeLeft -= curSize;
    }

  public:
    /**
     * Constructor.
     * @param _lut (Reference to) the address translation lut.
     * @param _start The iterator corresponding to the first entry point
     * @param _length The number of entries to process
     * @param _scattering The scattering direction flag.
     * @param _chunkSize The size of chunks into which
     *    the region should be decomposed.
     * @param _chunkAlign The alignment of chunks into which
     *    the region should be decomposed.
     *
     * @ingroup api_chunk_generator
     */
    SgaChunkGenerator(const amap_t &_lut, amap_it_t _start, uint16_t _length,
                      bool _scattering, Addr _chunkSize, Addr _chunkAlign) :
        lut(_lut), start(_start), length(_length), scattering(_scattering),
        chunkSize(_chunkSize), chunkAlign(_chunkAlign)
    {
        // chunkSize must be a power of two
        assert(chunkSize == 0 || isPowerOf2(chunkSize));

        // chunkAlign must be a power of two
        assert(chunkAlign == 0 || isPowerOf2(chunkAlign));

        // calculate the total size
        assert(std::distance(start, lut.end()) >= length);
        totalSize = length * sizeof(uint64_t);

        // initial values
        nextEntry = start;
        endEntry  = std::next(start, length);
        sizeLeft  = totalSize;

        // place iterator at beginning of LUT

        if (chunkSize == 0) { // Special Case, if we see 0, assume no chunking.
            curEntry  = start;
            curSize   = totalSize;
            nextEntry = endEntry;
            sizeLeft  = 0;
        } else {
            _calcNext();
        }
    }

    /**
     * Return starting address of current chunk.
     *
     * @ingroup api_chunk_generator
     */
    Addr addr() const { return _getAddr(curEntry); }

    /**
     * Return size in bytes of current chunk.
     *
     * @ingroup api_chunk_generator
     */
    Addr size() const { return curSize; }

    /**
     * Number of bytes we have already chunked up.
     *
     * @ingroup api_chunk_generator
     */
    Addr complete() const { return totalSize - sizeLeft; }

    /**
     * Are we done?  That is, did the last call to next() advance
     * past the end of the region?
     * @return True if yes, false if more to go.
     *
     * @ingroup api_chunk_generator
     */
    bool done() const { return curSize == 0; }

    /**
     * Is this the last chunk?
     * @return True if yes, false if more to go.
     *
     * @ingroup api_chunk_generator
     */
    bool last() const { return sizeLeft == 0; }

    /**
     * Advance generator to next chunk.
     * @return True if successful, false if unsuccessful
     * (because we were at the last chunk).
     *
     * @ingroup api_chunk_generator
     */
    bool
    next()
    {
        if (last()) {
            curSize = 0;
            return false;
        }

        _calcNext();
        return true;
    }
};

} // namespace cat

} // namespace gem5

#endif // __BASE_SGA_CHUNK_GENERATOR_HH__
