# Adjust if IgH is installed elsewhere:  make ETHERCAT_PREFIX=/usr/local
ETHERCAT_PREFIX ?= /opt/etherlab
CFLAGS  ?= -O2 -Wall -Wextra -Wno-missing-field-initializers
CFLAGS  += -I$(ETHERCAT_PREFIX)/include
LDFLAGS += -L$(ETHERCAT_PREFIX)/lib -Wl,-rpath,$(ETHERCAT_PREFIX)/lib
LDLIBS  += -lethercat -lpthread -lm

all: myact_ctl
myact_ctl: myact_ctl.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

# Variant that forces PDO mapping 0x1600/0x1A00 (only if the drive supports it)
custom: myact_ctl.c
	$(CC) $(CFLAGS) -DUSE_CUSTOM_PDO $< -o myact_ctl $(LDFLAGS) $(LDLIBS)

clean:
	rm -f myact_ctl
