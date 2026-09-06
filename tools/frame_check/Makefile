# Host-side verification of the frame layer against captured traffic.
# d101_frame.cpp has no ESPHome dependencies, so it builds and runs on a PC.
#
#   make check
#
#   frame_check - envelope, DL/T 645 layer and the reference request/response
#                 frames from the protocol notes
#   f202_check  - the 15 records of a captured DI 0xF202 page, which is what
#                 established that instantaneous values are 4-byte BCD and not
#                 the 4-byte binary the DLMS data types had suggested, plus the
#                 DI 0xF203 replies: the assembled continuation forms and the one
#                 captured off the air, which turned out to be a status block
#   f1xx_check  - the fixed blocks DI 0xF101 and DI 0xF102: length as the only
#                 identification of the layout, and the map from their positional
#                 fields onto TAGs
COMP := ../../components/nartis_rf_2_meter
CXX ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -I$(COMP)
DEPS := $(COMP)/d101_frame.cpp $(COMP)/d101_frame.h
CHECKS := frame_check f202_check f1xx_check

all: $(CHECKS)

$(CHECKS): %: %.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMP)/d101_frame.cpp

.PHONY: all check clean
check: $(CHECKS)
	$(foreach c,$(CHECKS),./$(c) &&) true

clean:
	rm -f $(CHECKS)
