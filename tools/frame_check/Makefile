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
#                 captured off the air, which turned out to be a status block.
#                 Also the captured DI 0xF102 block, where the two sum checks on
#                 the per-phase powers are what pin the positional order down
COMP := ../../components/nartis_rf_2_meter
CXXFLAGS := -std=c++17 -Wall -Wextra -I$(COMP)
DEPS := $(COMP)/d101_frame.cpp $(COMP)/d101_frame.h

all: frame_check f202_check

frame_check: frame_check.cpp $(DEPS)
	g++ $(CXXFLAGS) -o $@ frame_check.cpp $(COMP)/d101_frame.cpp

f202_check: f202_check.cpp $(DEPS)
	g++ $(CXXFLAGS) -o $@ f202_check.cpp $(COMP)/d101_frame.cpp

.PHONY: all check clean
check: frame_check f202_check
	./frame_check
	./f202_check

clean:
	rm -f frame_check f202_check
