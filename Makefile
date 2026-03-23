DDPG = ddpg
IDIR =-Iinc -I$(DDPG)/src/mlpc -I$(DDPG)/src/ddpgc
CC=gcc -O3
CFLAGS=$(IDIR)

OUT_DIR = bin
ODIR = obj
SDIR = src

LIBS=-lncurses -lm -lpthread $(DDPG)/lib/mlpc.a $(DDPG)/lib/ddpgc.a

#
# Build for Windows
#
ifeq ($(WINDOWS),1)
CFLAGS += -DWINDOWS_BUILD
endif

_OBJ = main.o tg_render.o tg_obj.o tg_obj_ai.o tank_obj.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

OBJ += $(DDPG)/build/mlpc/*.o

$(ODIR)/%.o: $(SDIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(OUT_DIR)/tgai: $(OBJ)
	$(CC) -Wall -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
