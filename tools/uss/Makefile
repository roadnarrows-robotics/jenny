CDEBUG = -g
CFLAGS = -I. -I/prj/include
CFLAGS = -I. -I/prj/include -I/prj/ros/indigo/src/jenny/include
LDFLAGS = -g

SRCS_CXX = uss.cxx main.cxx
OBJS    = $(SRCS_CXX:.cxx=.o)

# $(warning $(OBJS))
$(OBJS): uss.h

all: uss

uss: $(OBJS)
	g++ -L/prj/lib/rnr $(OBJS) -lrnr_serial -lrnr -o $@

clean:
	rm -f *.o

%.o : %.cxx
	g++ $(CXXFLAGS) $(CPPFLAGS) $(CFLAGS) -o $(@) -c $(<)
