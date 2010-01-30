/*
 * Copyright (c) 2008 Princeton University
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
 *
 * Authors: Niket Agarwal
 */

/*
 * This header file is used to define all configuration parameters
 * required by the interconnection network.
 */

#ifndef NETWORKCONFIG_H
#define NETWORKCONFIG_H

#include "mem/ruby/network/garnet-fixed-pipeline/NetworkHeader.hh"
#include "mem/gems_common/util.hh"

class NetworkConfig {
        private:
                int m_flit_size;
                int m_number_of_pipe_stages;
                int m_vcs_per_class;
                int m_buffer_size;
                bool m_using_network_testing;
        public:
                NetworkConfig(){}
                void init() {
                  for (size_t i=0; i<argv.size(); i+=2) {
                   if (argv[i] == "flit_size")
                     m_flit_size = atoi(argv[i+1].c_str());
                   else if (argv[i] == "number_of_pipe_stages")
                     m_number_of_pipe_stages = atoi(argv[i+1].c_str());
                   else if (argv[i] == "vcs_per_class")
                     m_vcs_per_class = atoi(argv[i+1].c_str());
                   else if (argv[i] == "buffer_size")
                     m_buffer_size = atoi(argv[i+1].c_str());
                   else if (argv[i] == "using_network_testing")
                     m_using_network_testing = atoi(argv[i+1].c_str());
                  }
                }
                bool isNetworkTesting() {return m_using_network_testing; }
                int getFlitSize() {return m_flit_size; }
                int getNumPipeStages() {return m_number_of_pipe_stages; }
                int getVCsPerClass() {return m_vcs_per_class; }
                int getBufferSize() {return m_buffer_size; }
};


#endif
