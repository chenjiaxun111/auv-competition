CC = gcc

CFLAGS := -Wall -Wextra -O2 -fPIC
CPPFLAGS := -I. -Iinc -Iplugins
LDLIBS := -pthread

TARGET := libeasylogger.so
SOURCES := \
	src/elog.c \
	src/elog_async.c \
	src/elog_buf.c \
	src/elog_utils.c \
	port/elog_port.c \
	plugins/file/elog_file.c \
	plugins/file/elog_file_port.c
OBJECTS := $(SOURCES:.c=.o)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) -shared -o $@ $(OBJECTS) $(LDLIBS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJECTS) $(TARGET)
