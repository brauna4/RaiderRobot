#     _____
#    /  _  \
#   / _/ \  \
#  / / \_/   \
# /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
# \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
#  \ \_/ \_/ /  | |  | |  | +-+| |_| || ++ / | ++_/| |_| |  | |  | +-+ |
#   \  \_/  /   | |_ | |_ | +-+|  _  || |\ \ | |   |  _  |  | |  | +-+ |
#    \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
#            ROBOTICS�
#
#  File: makefile
#  Desc: Makefile for demo C++ interface code
#  Auth: R. Gariepy
#
#  Copyright � 2010 Clearpath Robotics, Inc. 
#  All Rights Reserved
#  
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Clearpath Robotics, Inc. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Please send comments, questions, or patches to skynet@clearpathrobotics.com
#

.SECONDEXPANSION:

# Define ARCH as linux_x86 or win_x86
ARCH ?= linux_x86_64
INCLUDES = -I.
CPP = g++
CFLAGS += -g -Wall -Wextra -D$(ARCH) -DTRANSPORT_AVAIL -DLOGGING_AVAIL
OPENCV_LIBS= -L -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_ts -lopencv_video -lopencv_videostab -lgd -lrt -lpthread -lcurl -lopencv_gpu -L/usr/local/cuda -L/home/bnair002/NVIDIA_GPU_Computing_SDK/C/lib -L/usr/local/lib
OPENCV_CFLAGS=`pkg-config --cflags opencv libcurl` -I/usr/local/include/opencv -I/usr/local/include -I/usr/local/include/opencv2 -I/usr/local/cuda/include 

BINS = person_follower keyboard_ctrl
OBJDIR ?= ../obj
DEPDIR ?= ../dep
BINDIR ?= ../bin

BIN_OUTPUTS:= $(patsubst %,$(BINDIR)/%,$(BINS))

all: $(BIN_OUTPUTS)

# Sources to compile and link into bins 
SRCS = crc.cpp \
        cwrap.cpp \
        Logger.cpp \
        Message.cpp \
        Message_data.cpp \
        Message_request.cpp \
        Message_cmd.cpp \
        Transport.cpp \
        Number.cpp \
        linux_serial.cpp \
        windows_serial.cpp \
        face_detector.cpp \
        body_detector.cpp \
        descriptor.cpp

BIN_SRCS:= $(patsubst %,%.cpp,$(BINS))

# Generate deps files for sources
DEPS:= $(patsubst %.cpp, $(DEPDIR)/%.d, $(SRCS) $(BIN_SRCS))

$(DEPS): $$(patsubst $$(DEPDIR)/%.d, %.cpp, $$@)
	$(CPP) $(INCLUDES) $(OPENCV_CFLAGS) $< -MM -MF $@ -MT $(patsubst $(DEPDIR)/%.d,$(OBJDIR)/%.o,$@)

# Include any depsfiles generated last time around
include $(DEPS)

# Compilation rules 
OBJS:= $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRCS))
BIN_OBJS:= $(patsubst %.cpp,$(OBJDIR)/%.o,$(BIN_SRCS))

$(OBJS) $(BIN_OBJS): $$(patsubst $$(OBJDIR)/%.o, %.cpp, $$@)
	$(CPP) $(CFLAGS) $(INCLUDES) $(OPENCV_CFLAGS) -c $< -o $@

$(BIN_OUTPUTS): $(OBJS) $$(patsubst $$(BINDIR)/%,$$(OBJDIR)/%.o,$$@)
	$(CPP) $^ -o $@ $(OPENCV_LIBS)

clean:
	rm -rf $(DEPS)
	rm -rf $(OBJS) $(BIN_OBJS)
	rm -rf $(BIN_OUTPUTS)

tags: $(SRCS)
	ctags $(SRCS)

debugvars:
	@echo "BINS: ${BINS}"
	@echo "SRCS: ${SRCS}"
	@echo "BIN_SRCS: ${BIN_SRCS}"
	@echo "OBJS: ${OBJS}"
	@echo "BIN_OBJS: ${BIN_OBJS}"

