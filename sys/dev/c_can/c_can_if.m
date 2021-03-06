#-
# Copyright (c) 2019, 2021 Henning Matyschok, DARPA/AFRL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
#

#include <sys/bus.h>

INTERFACE c_can;

#
# Read register.
#
METHOD uint8_t read_1 {
    device_t    dev;
    int     port;
};

METHOD uint16_t read_2 {
    device_t    dev;
    int     port;
};

METHOD uint32_t read_4 {
    device_t    dev;
    int     port;
};

#
# Write register.
#
METHOD void write_1 {
    device_t    dev;
    int     port;
    uint8_t     val;
};


METHOD void write_2 {
    device_t    dev;
    int     port;
    uint16_t    val;
};

METHOD void write_4 {
    device_t    dev;
    int     port;
    uint32_t    val;
};

#
# Software reset.
#
METHOD void reset {
    device_t    dev;
};
