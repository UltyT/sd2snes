OBJS = header.ips reset.o65 tests.o65 spc7110.o65 data.o65 ppuconsole.o65 ppuconsole_font.o65 datarom.o65

all: clean spc7110_test.sfc map

map: spc7110_test.sfc
	../utils/mkmap.sh $(OBJS)

spc7110_test.sfc: $(OBJS)
	sneslink -fsmc -o $@ $^ 2>&1 | tee link.log

# Generic rule to create .o65 out from .a65
%.o65: %.a65
	snescom -J -Wall -o $@ $< 2>&1 | tee $@.log

# Generic rule to create .ips out from .a65
%.ips: %.a65
	snescom -I -J -Wall -o $@ $< 2>&1 | tee $@.log

clean:
	rm -f *.ips *.o65 *~ spc7110_test.sfc
