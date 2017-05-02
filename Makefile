TARGET = sparkler
CXX = g++
CXXFLAGS = -Wall -std=c++14
SDIR = src
ODIR = build
SRCS = $(patsubst $(SDIR)/%, %, $(wildcard $(SDIR)/*.cc))
OBJS = $(addprefix $(ODIR)/, $(patsubst %.cc, %.o, $(SRCS)))

.PHONY: all clean
all: $(TARGET)

$(ODIR)/%.o: $(SDIR)/%.cc
	test -d $(ODIR) || mkdir $(ODIR)
	$(CXX) $(CXXFLAGS) -o $@ -c $<

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -rf build sparkler
