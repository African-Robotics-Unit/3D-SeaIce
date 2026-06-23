CXX      := g++
CXXFLAGS := -std=c++14 -O2 -Wall -Wextra -Iinclude
LIBS     := -lrealsense2 -lpthread

# ── Add new programs here ──────────────────────────────────────────────────────
PROGRAMS := rs_record rgb_viewer rec_viewer rs_clean test_res

rgb_viewer_EXTRA_CFLAGS := $(shell pkg-config --cflags opencv4)
rgb_viewer_EXTRA_LIBS   := $(shell pkg-config --libs   opencv4)

rec_viewer_EXTRA_CFLAGS := $(shell pkg-config --cflags opencv4)
rec_viewer_EXTRA_LIBS   := $(shell pkg-config --libs   opencv4)
# ──────────────────────────────────────────────────────────────────────────────

# On Ubuntu 22.04+ with snap installed, Qt's platform plugin can accidentally
# pull in snap core20's libpthread (built for glibc 2.31) which is missing
# __libc_pthread_init in glibc 2.35+. Preloading the system pthread wins the race.
PTHREAD_FIX := LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

LIB_SRCS := src/d456_recorder.cpp
LIB_OBJS := $(LIB_SRCS:src/%.cpp=build/lib/%.o)

.PHONY: all clean check-deps $(PROGRAMS)
all: check-deps $(PROGRAMS)

build/lib/%.o: src/%.cpp | build/lib
	$(CXX) $(CXXFLAGS) -c $< -o $@

build/lib:
	mkdir -p $@

define PROGRAM_template
build/$(1):
	mkdir -p $$@

$(1): $(LIB_OBJS) build/$(1)/main.o
	$(CXX) $(CXXFLAGS) $$^ -o $$@ $(LIBS) $$($(1)_EXTRA_LIBS)

build/$(1)/main.o: programs/$(1)/main.cpp | build/$(1)
	$(CXX) $(CXXFLAGS) $$($(1)_EXTRA_CFLAGS) -c $$< -o $$@
endef

$(foreach prog,$(PROGRAMS),$(eval $(call PROGRAM_template,$(prog))))

clean:
	rm -rf build $(PROGRAMS)

check-deps:
	@pkg-config --exists realsense2 2>/dev/null && echo "librealsense2 found via pkg-config" || \
	  (ldconfig -p | grep -q librealsense2 && echo "librealsense2 found in ldconfig") || \
	  (echo "WARNING: librealsense2 not found." && exit 1)

# If librealsense2 is in a non-standard path, override like:
#   make CXXFLAGS="-std=c++14 -O2 -Iinclude -I/opt/librealsense2/include" \
#        LIBS="-L/opt/librealsense2/lib -lrealsense2 -lpthread"
