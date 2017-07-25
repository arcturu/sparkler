TARGET = sparkler
CXX = g++-6
CXXFLAGS = -Wall -std=c++14 -O3 -fopenmp
SDIR = src
ODIR = build
SRCS = $(patsubst $(SDIR)/%, %, $(wildcard $(SDIR)/*.cc))
OBJS = $(addprefix $(ODIR)/, $(patsubst %.cc, %.o, $(SRCS)))

.PHONY: all clean
all: $(TARGET)

$(ODIR)/%.o: $(SDIR)/%.cc
	@ test -d $(ODIR) || mkdir $(ODIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	cd lib/json11 && cmake -DCMAKE_CXX_COMPILER=$(CXX) . && make && cd ../../
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS) $(LIBDIRS) lib/json11/libjson11.a

test: $(filter-out $(ODIR)/main.o, $(OBJS))
	$(CXX) $(CXXFLAGS) -o test-main test/test.cc $^ $(LIBS) $(LIBDIRS)

clean:
	cd lib/json11 && make clean && rm CMakeCache.txt && rm -rf CMakeFiles && cd ../../
	rm -rf build sparkler sparkler.dtps
