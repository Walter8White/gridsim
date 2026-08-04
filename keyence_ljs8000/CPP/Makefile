VPATH=$(ALL_SRC_DIR)
#------------------------------------------------------------
# Makefile for LJS communication library and sample application
#------------------------------------------------------------
# Compiler
CC		= g++

# Target program
TARGET		= main
EXTRA_TARGETS	= check_status keyence_cmd keyence_setting
LIB_TARGET	= libljscom

# Directories
SRCDIR		= .
INCDIR		= ../include
BINDIR		= bin

LIBSRC_DIR	= ../libsrc
OBJDIR		= obj

# Option
USE_OPENCV	= false
USE_VIZ		= false
USE_PNG		= true

#------------------------------------------------------------
# Compile/Link the sample application
#------------------------------------------------------------
OBJS = $(SRCDIR)/$(TARGET).cpp $(OBJDIR)/$(LIB_TARGET).a

CFLAGS		= -g -Wall -I$(INCDIR) -I$(SRCDIR)
LDFLAGS		= -L/usr/local/lib -lpthread

# Optional include files and libraries
ifeq ($(USE_OPENCV),true)
	CFLAGS		+= -DOPENCV_EN $(shell pkg-config --cflags opencv4)
	LDFLAGS		+= -lopencv_highgui -lopencv_core
endif

ifeq ($(USE_PNG),true)
	CFLAGS		+= -DPNG_EN $(shell pkg-config --cflags opencv4)
	LDFLAGS		+= -lopencv_imgcodecs -lopencv_core
endif

ifeq ($(USE_VIZ),true)
	CFLAGS		+= -DVIZ_EN
	LDFLAGS		+= -lopencv_viz
endif


all: directories $(BINDIR)/$(TARGET) $(addprefix $(BINDIR)/,$(EXTRA_TARGETS))

directories:
	@mkdir -p -v $(BINDIR)
	@mkdir -p -v $(OBJDIR)

.PHONY: clean
clean:
	rm -rf $(OBJDIR)
	rm -f $(BINDIR)/$(TARGET) $(addprefix $(BINDIR)/,$(EXTRA_TARGETS))

$(BINDIR)/$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)

$(BINDIR)/check_status: $(SRCDIR)/check_status.cpp $(OBJDIR)/$(LIB_TARGET).a
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BINDIR)/keyence_cmd: $(SRCDIR)/keyence_cmd.cpp $(OBJDIR)/$(LIB_TARGET).a
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BINDIR)/keyence_setting: $(SRCDIR)/keyence_setting.cpp $(OBJDIR)/$(LIB_TARGET).a
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

#------------------------------------------------------------
# Compile/Archive library
#------------------------------------------------------------
AR		= ar
LIB_ARFLAGS	= r
LIB_CFLAGS	= -fPIC -g -Wall -pthread -I$(INCDIR)
LIB_SUB1	= LJS8_IF_Linux
LIB_SUB2	= LJS_ACQ

LIB_OBJS	= $(OBJDIR)/$(LIB_SUB1).o $(OBJDIR)/$(LIB_SUB2).o

.PHONY: lib
lib: directories $(OBJDIR)/$(LIB_TARGET).a

$(OBJDIR)/$(LIB_TARGET).a: $(LIB_OBJS)
	$(AR) $(LIB_ARFLAGS) $@ $(LIB_OBJS)

$(OBJDIR)/$(LIB_SUB1).o: $(LIBSRC_DIR)/$(LIB_SUB1).cpp
	$(CC) $(LIB_CFLAGS) -c $(LIBSRC_DIR)/$(LIB_SUB1).cpp -o $@

$(OBJDIR)/$(LIB_SUB2).o: $(LIBSRC_DIR)/$(LIB_SUB2).cpp
	$(CC) $(LIB_CFLAGS) -c $(LIBSRC_DIR)/$(LIB_SUB2).cpp -o $@
