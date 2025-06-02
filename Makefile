CXX	        = g++
CCFLAGS	    = -g -Wall -Wextra -I. -MMD -MP
LDFLAGS     = 

CCSRC       = RSTL_simulator.cpp

OBJS = $(CCSRC:.cpp=.o)

all:	RSTL_simulator

RSTL_simulator:	$(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

%.o:	%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

clean:
	rm -fr RSTL_simulator $(OBJS)
