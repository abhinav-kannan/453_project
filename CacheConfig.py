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
#
# Authors: Lisa Hsu

# Configure the M5 cache hierarchy config in one place
#

import re
import m5
from m5.objects import *
from Caches import *

def config_cache(options, system):
    if options.l2cache:
        system.l2 = L2Cache(size = options.l2_size, assoc = options.l2_assoc,
                            block_size=options.cacheline_size)
        system.tol2bus = Bus()
        system.l2.cpu_side = system.tol2bus.port
        system.l2.mem_side = system.membus.port
        system.l2.num_cpus = options.num_cpus

    for i in xrange(options.num_cpus):
        if options.caches:
            icache = L1Cache(size = options.l1i_size, assoc = options.l1i_assoc,
                             block_size=options.cacheline_size)
            dcache = L1Cache(size = options.l1d_size, assoc = options.l1d_assoc,
                             block_size=options.cacheline_size)
            if buildEnv['TARGET_ISA'] == 'x86':
                system.cpu[i].addPrivateSplitL1Caches(icache, dcache,
                                                      PageTableWalkerCache(),
                                                      PageTableWalkerCache())
            else:
                system.cpu[i].addPrivateSplitL1Caches(icache, dcache)
        if options.l2cache:
            system.cpu[i].connectAllPorts(system.tol2bus, system.membus)
        else:
            system.cpu[i].connectAllPorts(system.membus)

    return system

# Project
def new_config_cache(options, system, num_bce=4, num_r=1):
    # Number of processors to create
    if options.asymmetric:
        np = 1 + num_bce - num_r
    else:
        np = int(num_bce / num_r)

    if options.l2cache:
        # Change the size of the l2 cache
        old_l2_size = re.findall(r'[0-9]+', options.l2_size)

        if options.asymmetric:
            new_l2_size = str(int(old_l2_size[0]) * num_r) + "MB"
        else:
            new_l2_size = str(int(old_l2_size[0]) * int(num_bce/num_r)) + "MB"
        
        if options.asymmetric:
            new_l2_assoc = num_r
        else:
            new_l2_assoc = int(num_bce / num_r)
        
        system.l2 = L2Cache(size = new_l2_size, assoc = new_l2_assoc,
                                block_size=options.cacheline_size)

        system.tol2bus = Bus()
        system.l2.cpu_side = system.tol2bus.port
        system.l2.mem_side = system.membus.port
        system.l2.num_cpus = np

    for i in xrange(np):
        if options.caches:
            new_l1i_size = options.l1i_size
            new_l1d_size = options.l1d_size
            new_l1i_assoc = 8
            new_l1d_assoc = 8

            old_l1i_size = re.findall(r'[0-9]+', options.l1i_size)
            old_l1d_size = re.findall(r'[0-9]+', options.l1d_size)

            if options.asymmetric:
                if i is 0:
                    new_l1i_size = str(int(old_l1i_size[0]) * num_r) + "kB"
                    new_l1d_size = str(int(old_l1d_size[0]) * num_r) + "kB"
                    new_l1i_assoc = num_r
                    new_l1d_assoc = num_r
            else:       
                new_l1i_size = str(int(old_l1i_size[0]) * int(num_bce / num_r)) + "kB"
                new_l1d_size = str(int(old_l1d_size[0]) * int(num_bce / num_r)) + "kB"
                new_l1i_assoc = int(num_bce / num_r)
                new_l1d_assoc = int(num_bce / num_r)

            print "CPU " + str(i) + " -> L1i Size: " + str(new_l1i_size) + " L1d Size: " + str(new_l1d_size)
            icache = L1Cache(size = new_l1i_size, assoc = new_l1i_assoc,
                            block_size=options.cacheline_size)
            dcache = L1Cache(size = new_l1d_size, assoc = new_l1d_assoc,
                            block_size=options.cacheline_size)

            if buildEnv['TARGET_ISA'] == 'x86':
                system.cpu[i].addPrivateSplitL1Caches(icache, dcache,
                                                    PageTableWalkerCache(),
                                                    PageTableWalkerCache())
            else:
                system.cpu[i].addPrivateSplitL1Caches(icache, dcache)
        if options.l2cache:
            system.cpu[i].connectAllPorts(system.tol2bus, system.membus)
        else:
            system.cpu[i].connectAllPorts(system.membus)

    return system
