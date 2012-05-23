# Copyright (c) 2010 ARM Limited
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
# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
#
# Authors: Ali Saidi

import optparse
import os
import sys
import re

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

if not buildEnv['FULL_SYSTEM']:
    fatal("This script requires full-system mode (*_FS).")

addToPath('../common')

from FSConfig import *
from SysPaths import *
from Benchmarks import *
import Simulation
import CacheConfig
from Caches import *

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)

parser = optparse.OptionParser()

# Simulation options
parser.add_option("--timesync", action="store_true",
        help="Prevent simulated time from getting ahead of real time")

# System options
parser.add_option("--kernel", action="store", type="string")
parser.add_option("--script", action="store", type="string")
parser.add_option("--frame-capture", action="store_true",
        help="Stores changed frame buffers from the VNC server to compressed "\
        "files in the gem5 output directory")

if buildEnv['TARGET_ISA'] == "arm":
    parser.add_option("--bare-metal", action="store_true",
               help="Provide the raw system without the linux specific bits")
    parser.add_option("--machine-type", action="store", type="choice",
            choices=ArmMachineType.map.keys(), default="RealView_PBX")
# Benchmark options
parser.add_option("--dual", action="store_true",
                  help="Simulate two systems attached with an ethernet link")
parser.add_option("-b", "--benchmark", action="store", type="string",
                  dest="benchmark",
                  help="Specify the benchmark to run. Available benchmarks: %s"\
                  % DefinedBenchmarks)

# Metafile options
parser.add_option("--etherdump", action="store", type="string", dest="etherdump",
                  help="Specify the filename to dump a pcap capture of the" \
                  "ethernet traffic")

# New options for 453 project 
parser.add_option("--asymmetric", action="store_true",
		  help="Specifies asymmetric mode. Default symmetric")
parser.add_option("--num-bce", type="int", default=16,
		  help="Specify the number of Base Core Equivalents (BCE)")
parser.add_option("--num-r", type="int", default=1,
		  help="Specify the number of BCEs per core")

execfile(os.path.join(config_root, "common", "Options.py"))

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# driver system CPU is always simple... note this is an assignment of
# a class, not an instance.

DriveCPUClass = AtomicSimpleCPU
drive_mem_mode = 'atomic'

# system under test can be any CPU
(TestCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)

TestCPUClass.clock = '2GHz'
DriveCPUClass.clock = '2GHz'

if options.benchmark:
    try:
        bm = Benchmarks[options.benchmark]
    except KeyError:
        print "Error benchmark %s has not been defined." % options.benchmark
        print "Valid benchmarks are: %s" % DefinedBenchmarks
        sys.exit(1)
else:
    if options.dual:
        bm = [SysConfig(), SysConfig()]
    else:
        bm = [SysConfig()]

# options.num_cpus will not be used. We would need to compute it
# according the num_bce and num_rsc
np = options.num_cpus

# Number of BCE in total
num_bce = options.num_bce
# Number of resources/BCE per core
num_rsc = options.num_r

if buildEnv['TARGET_ISA'] == "alpha":
    test_sys = makeLinuxAlphaSystem(test_mem_mode, bm[0])
elif buildEnv['TARGET_ISA'] == "mips":
    test_sys = makeLinuxMipsSystem(test_mem_mode, bm[0])
elif buildEnv['TARGET_ISA'] == "sparc":
    test_sys = makeSparcSystem(test_mem_mode, bm[0])
elif buildEnv['TARGET_ISA'] == "x86":
    test_sys = makeLinuxX86System(test_mem_mode, options.num_cpus, bm[0])
    setWorkCountOptions(test_sys, options)
elif buildEnv['TARGET_ISA'] == "arm":
    test_sys = makeArmSystem(test_mem_mode,
            options.machine_type, bm[0],
            bare_metal=options.bare_metal)
    setWorkCountOptions(test_sys, options)
else:
    fatal("incapable of building non-alpha or non-sparc full system!")

if options.kernel is not None:
    test_sys.kernel = binary(options.kernel)

if options.script is not None:
    test_sys.readfile = options.script

# Project code
if options.asymmetric:
	test_sys.cpu = [TestCPUClass(cpu_id=i) for i in xrange(1+num_bce-num_rsc)]

 	# In an asymmetric system, we can have one big-core
	# and several small cores depending on the num_bce
	# We have simulated one 'num_r' BCE big core and remaining
	# one BCE cores. Accordingly we must change the below
	# parameters for the architecture to be constructed properly
	# CPU 0 is the big core
	print "Asymmetric mode"

	# TODO: I could not find a way to create an asymmetric system with 
	# the configuration we want. I think we need a system with one big
	# superscalar core with width 'num_bce' and all the other cores to 
	# be baseline. As of now, I am not sure if there is a way to do it.
	# The workaround is to make all the cores as O3CPU and change the 
	# widths of all other cores to 1. We still do not know the config 
	# of the base core so I am assuming the base core is like a 
	# uniprocessor.
	TestCPUClass.issueWidth = 1	# Default: 8
	TestCPUClass.fetchWidth = 1	# Default: 8
	TestCPUClass.decodeWidth = 1	# Default: 8
	TestCPUClass.dispatchWidth = 1	# Default: 8
	TestCPUClass.renameWidth = 1	# Default: 8
	TestCPUClass.issueWidth = 1	# Default: 8
	TestCPUClass.commitWidth = 1	# Default: 8
	TestCPUClass.wbWidth = 1	# Default: 8
	TestCPUClass.RASSize = 2	# Default: 16
	TestCPUClass.LQEntries = 4	# Default: 32
	TestCPUClass.SQEntries = 4	# Default: 32
	TestCPUClass.numIQEntries = 8	# Default: 64
	TestCPUClass.numROBEntries = 24	# Default: 192

	test_sys.cpu[0].issueWidth = num_rsc	# Default: 8
	test_sys.cpu[0].fetchWidth = num_rsc	# Default: 8
	test_sys.cpu[0].decodeWidth = num_rsc	# Default: 8
	test_sys.cpu[0].dispatchWidth = num_rsc	# Default: 8
	test_sys.cpu[0].renameWidth = num_rsc	# Default: 8
	test_sys.cpu[0].issueWidth = num_rsc	# Default: 8
	test_sys.cpu[0].commitWidth = num_rsc	# Default: 8
	test_sys.cpu[0].wbWidth = num_rsc	# Default: 8
	test_sys.cpu[0].RASSize = num_rsc * 2	# Default: 16
	test_sys.cpu[0].LQEntries = num_rsc * 4	# Default: 32
	test_sys.cpu[0].SQEntries = num_rsc * 4	# Default: 32
	test_sys.cpu[0].numIQEntries = num_rsc * 8 # Default: 64
	test_sys.cpu[0].numROBEntries = num_rsc * 24 # Default: 192
else:
	test_sys.cpu = [TestCPUClass(cpu_id=i) for i in xrange(int(num_bce/num_rsc))]
	
    	# In an symmetric system, we need to have all the cores
	# with the same configuration
    	print "Symmetric mode"
	rsc = num_bce / num_rsc
	TestCPUClass.issueWidth = rsc		# Default: 8
	TestCPUClass.fetchWidth = rsc		# Default: 8
	TestCPUClass.decodeWidth = rsc		# Default: 8
	TestCPUClass.dispatchWidth = rsc	# Default: 8
	TestCPUClass.renameWidth = rsc		# Default: 8
	TestCPUClass.issueWidth = rsc		# Default: 8
	TestCPUClass.commitWidth = rsc		# Default: 8
	TestCPUClass.wbWidth = rsc		# Default: 8
	TestCPUClass.RASSize = rsc * 2		# Default: 16
	TestCPUClass.LQEntries = rsc * 4	# Default: 32
	TestCPUClass.SQEntries = rsc * 4	# Default: 32
	TestCPUClass.numIQEntries = rsc * 8  	# Default: 64
	TestCPUClass.numROBEntries = rsc * 24 	# Default: 192    

CacheConfig.new_config_cache(options, test_sys, num_bce, num_rsc)

if options.caches or options.l2cache:
    if bm[0]:
        mem_size = bm[0].mem()
    else:
        mem_size = SysConfig().mem()
    # For x86, we need to poke a hole for interrupt messages to get back to the
    # CPU. These use a portion of the physical address space which has a
    # non-zero prefix in the top nibble. Normal memory accesses have a 0
    # prefix.
    if buildEnv['TARGET_ISA'] == 'x86':
        test_sys.bridge.filter_ranges_a=[AddrRange(0, Addr.max >> 4)]
    else:
        test_sys.bridge.filter_ranges_a=[AddrRange(0, Addr.max)]
    test_sys.bridge.filter_ranges_b=[AddrRange(mem_size)]
    test_sys.iocache = IOCache(addr_range=mem_size)
    test_sys.iocache.cpu_side = test_sys.iobus.port
    test_sys.iocache.mem_side = test_sys.membus.port

#for i in xrange(np):
for i in xrange(num_bce):
    if options.fastmem:
        test_sys.cpu[i].physmem_port = test_sys.physmem.port

if buildEnv['TARGET_ISA'] == 'mips':
    setMipsOptions(TestCPUClass)

if len(bm) == 2:
    if buildEnv['TARGET_ISA'] == 'alpha':
        drive_sys = makeLinuxAlphaSystem(drive_mem_mode, bm[1])
    elif buildEnv['TARGET_ISA'] == 'mips':
        drive_sys = makeLinuxMipsSystem(drive_mem_mode, bm[1])
    elif buildEnv['TARGET_ISA'] == 'sparc':
        drive_sys = makeSparcSystem(drive_mem_mode, bm[1])
    elif buildEnv['TARGET_ISA'] == 'x86':
	drive_sys = makeX86System(drive_mem_mode, num_bce, bm[1])
    elif buildEnv['TARGET_ISA'] == 'arm':
        drive_sys = makeArmSystem(drive_mem_mode,
                machine_options.machine_type, bm[1])
    drive_sys.cpu = DriveCPUClass(cpu_id=0)
    drive_sys.cpu.connectAllPorts(drive_sys.membus)
    if options.fastmem:
        drive_sys.cpu.physmem_port = drive_sys.physmem.port
    if options.kernel is not None:
        drive_sys.kernel = binary(options.kernel)

    root = makeDualRoot(test_sys, drive_sys, options.etherdump)
elif len(bm) == 1:
    root = Root(system=test_sys)
else:
    print "Error I don't know how to create more than 2 systems."
    sys.exit(1)

if options.timesync:
    root.time_sync_enable = True

if options.frame_capture:
    VncServer.frame_capture = True

Simulation.run(options, root, test_sys, FutureClass)
