TARGET = sparkler
CXX = g++
CXXFLAGS = -Wall -std=c++14 -O3
SDIR = src
ODIR = build
SRCS = $(patsubst $(SDIR)/%, %, $(wildcard $(SDIR)/*.cc))
OBJS = $(addprefix $(ODIR)/, $(patsubst %.cc, %.o, $(SRCS))) lib/json11/json11.o

.PHONY: all clean
all: $(TARGET)

$(ODIR)/%.o: $(SDIR)/%.cc
	@ test -d $(ODIR) || mkdir $(ODIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS) $(LIBDIRS)

clean:
	cd lib/json11 && make clean && cd ../../
	rm -rf build sparkler sparkler.dtps
