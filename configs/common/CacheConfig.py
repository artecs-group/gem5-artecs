# Copyright (c) 2012-2013, 2015-2016 ARM Limited
# Copyright (c) 2020 Barkhausen Institut
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2010 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Configure the M5 cache hierarchy config in one place
#

import m5
from m5.objects import *
from common.Caches import *
from common import ObjectList

def _get_hwp(hwp_option, params):
    if hwp_option == None:
        return NULL

    hwpClass = ObjectList.hwp_list.get(hwp_option)
    return hwpClass(**params)

def _get_cache_opts(level, options):
    opts = {}
    hwp_opts = {}

    size_attr = '{}_size'.format(level)
    if getattr(options, size_attr, None) is not None:
        opts['size'] = getattr(options, size_attr)

    assoc_attr = '{}_assoc'.format(level)
    if getattr(options, assoc_attr, None) is not None:
        opts['assoc'] = getattr(options, assoc_attr)

    enable_banks_attr = '{}_enable_banks'.format(level)
    if getattr(options, enable_banks_attr, None) is not None:
        opts['enable_banks'] = getattr(options, enable_banks_attr)

    num_banks_attr = '{}_num_banks'.format(level)
    if getattr(options, num_banks_attr, None) is not None:
        opts['num_banks'] = getattr(options, num_banks_attr)

    intlv_bit_attr = '{}_intlv_bit'.format(level)
    if getattr(options, intlv_bit_attr, None) is not None:
        opts['bank_intlv_high_bit'] = getattr(options, intlv_bit_attr)

    tag_latency_attr = '{}_tag_lat'.format(level)
    if getattr(options, tag_latency_attr, None) is not None:
        opts['tag_latency'] = getattr(options, tag_latency_attr)

    data_latency_attr = '{}_data_lat'.format(level)
    if getattr(options, data_latency_attr, None) is not None:
        opts['data_latency'] = getattr(options, data_latency_attr)

    write_latency_attr = '{}_write_lat'.format(level)
    if getattr(options, write_latency_attr, None) is not None:
        opts['write_latency'] = getattr(options, write_latency_attr)

    response_latency_attr = '{}_resp_lat'.format(level)
    if getattr(options, response_latency_attr, None) is not None:
        opts['response_latency'] = getattr(options, response_latency_attr)

    prefetcher_degree_attr = '{}_hwp_deg'.format(level)
    if getattr(options, prefetcher_degree_attr, None) is not None:
        hwp_opts['degree'] = getattr(options, prefetcher_degree_attr)

    prefetcher_latency_attr = '{}_hwp_lat'.format(level)
    if getattr(options, prefetcher_latency_attr, None) is not None:
        hwp_opts['latency'] = getattr(options, prefetcher_latency_attr)

    prefetcher_qs_attr = '{}_hwp_qs'.format(level)
    if getattr(options, prefetcher_qs_attr, None) is not None:
        hwp_opts['queue_size'] = getattr(options, prefetcher_qs_attr)

    prefetcher_attr = '{}_hwp_type'.format(level)
    if getattr(options, prefetcher_attr, None) is not None:
        opts['prefetcher'] = \
            _get_hwp(getattr(options, prefetcher_attr), hwp_opts)
    elif getattr(options, 'hwp_override', None):
        opts['prefetcher'] = m5.params.NULL

    return opts

def config_cache(options, system):
    if options.external_memory_system and (options.caches or options.l2cache):
        print("External caches and internal caches are exclusive options.\n")
        sys.exit(1)

    if options.external_memory_system:
        ExternalCache = ExternalCacheFactory(options.external_memory_system)

    if options.cpu_type == "O3_ARM_v7a_3":
        try:
            import cores.arm.O3_ARM_v7a as core
        except:
            print("O3_ARM_v7a_3 is unavailable. Did you compile the O3 model?")
            sys.exit(1)

        dcache_class, icache_class, l2_cache_class, walk_cache_class, \
            l3_cache_class = \
            core.O3_ARM_v7a_DCache, core.O3_ARM_v7a_ICache, \
            core.O3_ARM_v7aL2, \
            None, None
    elif options.cpu_type == "HPI":
        try:
            import cores.arm.HPI as core
        except:
            print("HPI is unavailable.")
            sys.exit(1)

        dcache_class, icache_class, l2_cache_class, walk_cache_class, \
            l3_cache_class = \
            core.HPI_DCache, core.HPI_ICache, core.HPI_L2, \
            None, None
    else:
        dcache_class, icache_class, l2_cache_class, walk_cache_class, \
            l3_cache_class = \
            L1_DCache, L1_ICache, L2Cache, None, L3Cache

        if buildEnv['TARGET_ISA'] in ['x86', 'riscv']:
            walk_cache_class = PageTableWalkerCache

    # Set the cache line size of the system
    system.cache_line_size = options.cacheline_size

    # If elastic trace generation is enabled, make sure the memory system is
    # minimal so that compute delays do not include memory access latencies.
    # Configure the compulsory L1 caches for the O3CPU, do not configure
    # any more caches.
    if options.l2cache and options.elastic_trace_en:
        fatal("When elastic trace is enabled, do not configure L2 caches.")

    num_l2caches = options.num_l2caches

    if options.l2cache:
        if options.num_cpus % num_l2caches != 0:
            fatal("The number of L2 caches must be a submultiple of the ",
                  "number of cores.")
        # Provide a clock for the L2 and the L1-to-L2 bus here as they
        # are not connected using addTwoLevelCacheHierarchy. Use the
        # same clock as the CPUs.
        system.l2 = [l2_cache_class(clk_domain=system.cpu_clk_domain,
            **_get_cache_opts('l2', options)) for i in range(num_l2caches)]
        system.tol2bus = [L2XBar(clk_domain = system.cpu_clk_domain)
            for i in range(num_l2caches)]

        for i in range(num_l2caches):
            system.l2[i].cpu_side = system.tol2bus[i].mem_side_ports
            if not options.l3cache:
                system.l2[i].mem_side = system.membus.cpu_side_ports

    if options.l3cache:
        if not options.l2cache:
            fatal("It is not possible to have a L3 cache without a L2.")

        if l3_cache_class == None:
            fatal("No valid L3 configuration found.")

        system.l3 = l3_cache_class(clk_domain=system.cpu_clk_domain,
                                   **_get_cache_opts('l3', options))

        system.tol3bus = L3XBar(clk_domain = system.cpu_clk_domain)
        system.l3.cpu_side = system.tol3bus.mem_side_ports
        system.l3.mem_side = system.membus.cpu_side_ports

        # Change some stuff in L2 since it is not the LLC anymore
        for i in range(num_l2caches):
            system.l2[i].clusivity = 'mostly_incl'
            system.l2[i].prefetch_on_access = False
            system.l2[i].mem_side = system.tol3bus.slave

    if options.memchecker:
        system.memchecker = MemChecker()

    for i in range(options.num_cpus):
        if options.caches:
            icache = icache_class(**_get_cache_opts('l1i', options))
            dcache = dcache_class(**_get_cache_opts('l1d', options))

            # If we have a walker cache specified, instantiate two
            # instances here
            if walk_cache_class:
                iwalkcache = walk_cache_class()
                dwalkcache = walk_cache_class()
            else:
                iwalkcache = None
                dwalkcache = None

            if options.memchecker:
                dcache_mon = MemCheckerMonitor(warn_only=True)
                dcache_real = dcache

                # Do not pass the memchecker into the constructor of
                # MemCheckerMonitor, as it would create a copy; we require
                # exactly one MemChecker instance.
                dcache_mon.memchecker = system.memchecker

                # Connect monitor
                dcache_mon.mem_side = dcache.cpu_side

                # Let CPU connect to monitors
                dcache = dcache_mon

            # When connecting the caches, the clock is also inherited
            # from the CPU in question
            system.cpu[i].addPrivateSplitL1Caches(icache, dcache,
                                                  iwalkcache, dwalkcache)

            if options.memchecker:
                # The mem_side ports of the caches haven't been connected yet.
                # Make sure connectAllPorts connects the right objects.
                system.cpu[i].dcache = dcache_real
                system.cpu[i].dcache_mon = dcache_mon

        elif options.external_memory_system:
            # These port names are presented to whatever 'external' system
            # gem5 is connecting to.  Its configuration will likely depend
            # on these names.  For simplicity, we would advise configuring
            # it to use this naming scheme; if this isn't possible, change
            # the names below.
            if buildEnv['TARGET_ISA'] in ['x86', 'arm', 'riscv']:
                system.cpu[i].addPrivateSplitL1Caches(
                        ExternalCache("cpu%d.icache" % i),
                        ExternalCache("cpu%d.dcache" % i),
                        ExternalCache("cpu%d.itb_walker_cache" % i),
                        ExternalCache("cpu%d.dtb_walker_cache" % i))
            else:
                system.cpu[i].addPrivateSplitL1Caches(
                        ExternalCache("cpu%d.icache" % i),
                        ExternalCache("cpu%d.dcache" % i))

        system.cpu[i].createInterruptController()
        if options.l2cache:
            bus = i % num_l2caches
            system.cpu[i].connectAllPorts(
                system.tol2bus[bus].cpu_side_ports,
                system.membus.cpu_side_ports, system.membus.mem_side_ports)
        elif options.external_memory_system:
            system.cpu[i].connectUncachedPorts(
                system.membus.cpu_side_ports, system.membus.mem_side_ports)
        else:
            system.cpu[i].connectBus(system.membus)

    return system

# ExternalSlave provides a "port", but when that port connects to a cache,
# the connecting CPU SimObject wants to refer to its "cpu_side".
# The 'ExternalCache' class provides this adaptation by rewriting the name,
# eliminating distracting changes elsewhere in the config code.
class ExternalCache(ExternalSlave):
    def __getattr__(cls, attr):
        if (attr == "cpu_side"):
            attr = "port"
        return super(ExternalSlave, cls).__getattr__(attr)

    def __setattr__(cls, attr, value):
        if (attr == "cpu_side"):
            attr = "port"
        return super(ExternalSlave, cls).__setattr__(attr, value)

def ExternalCacheFactory(port_type):
    def make(name):
        return ExternalCache(port_data=name, port_type=port_type,
                             addr_ranges=[AllMemory])
    return make
