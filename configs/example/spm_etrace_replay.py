# Copyright (c) 2015 ARM Limited
# All rights reserved.
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

# Basic elastic traces replay script that configures a Trace CPU

import argparse

from m5.util import addToPath, fatal

addToPath('../')

from common import Options
from common import Simulation
from common import CacheConfig
from common import MemConfig
from common.Caches import *

def addSPMOptions(parser):
    """parser.add_argument("--scratchpad-size", type=str,
                           default="4kB")"""
    parser.add_argument("--scratchpad-read-latency", type=str,
                        default="1ns")
    """parser.add_argument("--scratchpad-write-latency", type=str,
                           default="1ns")"""
    parser.add_argument("--scratchpad-bandwidth", type=str,
                        default='64GB/s')

def buildSPM(args, cpuid):
    scratchpad = SimpleMemory()
    spm_size  = MemorySize(args.mem_size)
    spm_start = MemorySize(args.mem_size).getValue() + (
        spm_size.getValue() * cpuid)
    scratchpad.range = AddrRange(spm_start, size = spm_size)
    scratchpad.latency = Latency(args.scratchpad_read_latency)
    scratchpad.bandwidth = MemoryBandwidth(args.scratchpad_bandwidth)
    return scratchpad

parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
addSPMOptions(parser)

if '--ruby' in sys.argv:
    print("This script does not support Ruby configuration, mainly"
    " because Trace CPU has been tested only with classic memory system")
    sys.exit(1)

args = parser.parse_args()

numThreads = 1

if args.cpu_type != "TraceCPU":
    fatal("This is a script for elastic trace replay simulation, use "\
            "--cpu-type=TraceCPU\n");

if args.num_cpus > 1:
    fatal("This script does not support multi-processor trace replay.\n")

# In this case FutureClass will be None as there is not fast forwarding or
# switching
(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(args)
CPUClass.numThreads = numThreads

system = System(cpu = CPUClass(cpu_id=0),
                mem_mode = test_mem_mode,
                mem_ranges = [AddrRange(args.mem_size)],
                cache_line_size = args.cacheline_size)

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage = args.sys_voltage)

# Create a source clock for the system. This is used as the clock period for
# xbar and memory
system.clk_domain = SrcClockDomain(clock =  args.sys_clock,
                                   voltage_domain = system.voltage_domain)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs. In case of Trace CPUs this clock
# is actually used only by the caches connected to the CPU.
system.cpu_clk_domain = SrcClockDomain(clock = args.cpu_clock,
                                       voltage_domain =
                                       system.cpu_voltage_domain)

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
for cpu in system.cpu:
    cpu.clk_domain = system.cpu_clk_domain

# BaseCPU no longer has default values for the BaseCPU.isa
# createThreads() is needed to fill in the cpu.isa
for cpu in system.cpu:
    cpu.createThreads()

# Assign input trace files to the Trace CPU
system.cpu.instTraceFile=args.inst_trace_file
system.cpu.dataTraceFile=args.data_trace_file

# Configure the classic memory system args
MemClass = Simulation.setMemClass(args)
system.cache_line_size = args.cacheline_size
# L1D-SPM Hub
system.hub = NoncoherentXBar()
system.hub.width = 8
system.hub.frontend_latency = 1
system.hub.forward_latency  = 0
system.hub.response_latency = 0
system.cpu.icache = L1_ICache()
system.cpu.dcache = L1_DCache()
# L1 Cache Parameters
system.cpu.dcache.size = args.l1d_size
system.cpu.icache.size = args.l1i_size
system.cpu.dcache.assoc = args.l1d_assoc
system.cpu.icache.assoc = args.l1i_assoc
system.cpu.dcache.data_latency = args.l1d_data_lat
system.cpu.icache.data_latency = args.l1i_data_lat
system.cpu.dcache.write_latency = args.l1d_write_lat
system.cpu.icache.write_latency = args.l1i_write_lat
system.cpu.dcache.tag_latency = args.l1d_tag_lat
system.cpu.icache.tag_latency = args.l1i_tag_lat
system.cpu.dcache.response_latency = args.l1d_resp_lat
system.cpu.icache.response_latency = args.l1i_resp_lat
system.cpu.dcache.enable_banks = args.l1d_enable_banks
system.cpu.icache.enable_banks = args.l1i_enable_banks
system.cpu.dcache.num_banks = args.l1d_num_banks
system.cpu.icache.num_banks = args.l1i_num_banks
system.cpu.dcache.bank_intlv_high_bit = args.l1d_intlv_bit
system.cpu.icache.bank_intlv_high_bit = args.l1i_intlv_bit
system.cpu.dcache.addr_ranges = [AddrRange(0, args.mem_size)]
system.cpu.dcache_port = system.hub.cpu_side_ports
system.cpu.icache.cpu_side = system.cpu.icache_port
# L2 Bus
system.l2bus = L2XBar()
system.cpu.icache.mem_side = system.l2bus.cpu_side_ports
system.cpu.dcache.mem_side = system.l2bus.cpu_side_ports
system.l2cache = L2Cache()
system.l2cache.cpu_side = system.l2bus.mem_side_ports
# L2 Cache Parameters
system.l2cache.size = args.l2_size
system.l2cache.assoc = args.l2_assoc
system.l2cache.data_latency = args.l2_data_lat
system.l2cache.write_latency = args.l2_write_lat
system.l2cache.tag_latency = args.l2_tag_lat
system.l2cache.response_latency = args.l2_resp_lat
system.l2cache.enable_banks = args.l2_enable_banks
system.l2cache.num_banks = args.l2_num_banks
system.l2cache.bank_intlv_high_bit = args.l2_intlv_bit
system.membus = SystemXBar()
system.l2cache.mem_side = system.membus.cpu_side_ports
# SPM Definition
system.scratchpad = buildSPM(args, 0)
system.hub.mem_side_ports = [system.scratchpad.port,
                             system.cpu.dcache.cpu_side]
system.cpu.createInterruptController()
if buildEnv['TARGET_ISA'] == "x86":
    system.cpu.interrupts[0].pio = system.membus.mem_side_ports
    system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports
MemConfig.config_mem(args, system)

root = Root(full_system = False, system = system)
Simulation.run(args, root, system, FutureClass)
