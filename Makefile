# Makefile -- makefile for DCF77/MSF/WWVB time signal decoding
#
#

VERSION=1.0

CC = /usr/bin/gcc
INSTALL = /usr/bin/install
DESTDIR = /usr/local
MANDESTDIR = /usr/local/
CFLAGS= -Wall
INSTALL-BIN = $(INSTALL)

ifneq (,$(findstring noopt,$(DEB_BUILD_OPTIONS)))
CFLAGS += -O0
else
CFLAGS += -O2
endif
ifeq (,$(findstring nostrip,$(DEB_BUILD_OPTIONS)))
INSTALL_BIN += -s
endif

.c.o:
	$(CC) $(CFLAGS) -c $<

all: radioclkd

radioclkd: radioclkd.o
	$(CC) -o $@ radioclkd.o

install: install-bin install-man

install-bin:
	$(INSTALL-BIN) -m 0755 radioclkd $(DESTDIR)/sbin

install-man:
	$(INSTALL) -m 0644 radioclkd.1 $(DESTDIR)/man/man1

clean:
	rm -f *.o *.bak core radioclkd

dist: clean
	(rm -f ChangeLog; \
	rcs2log -R > ChangeLog; \
	rm -rf /tmp/radioclk-$(VERSION); \
	mkdir /tmp/radioclk-$(VERSION); \
	cp * /tmp/radioclk-$(VERSION); \
	cp -a ./debian/ /tmp/radioclk-$(VERSION); \
	cd /tmp/radioclk-$(VERSION); \
	find -type d | xargs chmod 755; \
	find -type f | xargs chmod 644; \
	find -type d | xargs chown root:root; \
	find -type f | xargs chown root:root; \
	cd ..; \
	tar cvf radioclk-$(VERSION).tar radioclk-$(VERSION); \
	gzip -9f radioclk-$(VERSION).tar; \
	echo Done.)
