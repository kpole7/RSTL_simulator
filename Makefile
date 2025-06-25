CXX	        = g++
CCFLAGS	    = -g -Wall -Wextra -I. -MMD -MP
LDFLAGS     = 

CCSRC       = RSTL_master.cpp

OBJS = $(CCSRC:.cpp=.o)

all:	RSTL_master

RSTL_master:	$(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

%.o:	%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

clean:
	rm -fr RSTL_master $(OBJS)
