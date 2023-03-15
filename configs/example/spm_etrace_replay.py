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
    parser.add_argument("--spm1-read-latency", type=str,
                        default="2")
    parser.add_argument("--spm1-write-latency", type=str,
                        default="2")


def buildSPM(args, cpuid):
    scratchpad = Scratchpad()
    spm_size  = MemorySize(args.mem_size)
    spm_start = MemorySize(args.mem_size).getValue() + (
        spm_size.getValue() * cpuid)
    scratchpad.range = AddrRange(spm_start, size = spm_size)
    scratchpad.read_latency = Latency(args.spm1_read_latency)
    scratchpad.write_latency = Latency(args.spm1_write_latency)
    scratchpad.banks_enable = False
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
    fatal("This script does not support multi-processor trace replay\n")

if not args.caches:
    fatal("Caches must be used with this script\n")

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

MemClass = Simulation.setMemClass(args)
system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports
CacheConfig.config_cache(args, system, True)
MemConfig.config_mem(args, system)

mem_ctrl = system.mem_ctrls[0]
if getattr(mem_ctrl, "dram", False):
    rb_size = mem_ctrl.dram.device_rowbuffer_size
elif getattr(mem_ctrl, "nvm", False):
    rb_size = mem_ctrl.nvm.device_rowbuffer_size
else:
    fatal("Memory interface type not supported\n")

# Override dcache address range
system.cpu.dcache.addr_ranges = [AddrRange(0, args.mem_size)]

# SPM-related configuration
system.translator = CAT(pio_addr = 0x2000000000)
system.dmac = SgaDmaController(pio_addr = 0x3000000000,
                               mmem_rowbuffer_size = rb_size)
system.hub = TranslatingXBar(width = 8,
                             translator_port = system.translator.pio,
                             dmac_port = system.dmac.pio)
system.scratchpad = buildSPM(args, 0)
system.mem_ranges.append(system.scratchpad.range)
system.hub.mem_side_ports = [system.scratchpad.port,
                             system.cpu.dcache.cpu_side]
system.hub.cpu_side_ports = [system.dmac.dma,
                             system.cpu.dcache_port]

root = Root(full_system = False, system = system)
Simulation.run(args, root, system, FutureClass)
