# include Rules file with architecture dependent definitions of 
# VRMSDK_INSTALL_DIR, CROSS_COMPILE_PREFIX, VRM_INCPATH, VRM_LIBPATH, VRM_LFLAGS
include ../Rules.make

CXX      = $(CROSS_COMPILE_PREFIX)g++
CXXFLAGS = -pipe -O2 -fmessage-length=0 -fPIC -Wno-deprecated -Wall -W -Wno-unused

# append your additional includes with -I<path_to_new_dir>
INCPATH  = $(VRM_INCPATH)
# append your additional library search dirs with -L<path_to_new_dir>
LIBPATH  = $(VRM_LIBPATH)
# add additional linker flags as needed 
LFLAGS= $(VRM_LFLAGS)

LIBS     = -lvrmusbcam2 -lpthread
DEL_FILE = rm -f
####### Files

OBJECTS = main.o
TARGET  = build/$(_TARGET_ARCH)/deviceinfo

first: all
####### Implicit rules

.SUFFIXES: .o .cpp

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

####### Build rules

all: Makefile $(TARGET) clean-obj

$(TARGET):  $(OBJECTS)
	$(MKDIR) build
	$(MKDIR) build/$(_TARGET_ARCH)
	$(CXX) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBPATH) $(LIBS)

clean-obj:
	-$(DEL_FILE) $(OBJECTS)

clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core
	-$(DEL_FILE) $(TARGET)
	-$(RMDIR) build/$(_TARGET_ARCH)
