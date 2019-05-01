     1 #
     2 #  =================================================================================
     3 #     @file     Makefile
     4 #     @brief
     5 #
     6 #   Makefile for BeagleBone Green Temperature and Light Application
     7 #
     8 #     @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
     9 #
    10 #     @internal
    11 #        Created:  04/01/2019
    12 #       Revision:  none
    13 #       Compiler:  gcc
    14 #   Organization:  University of Colorado: Boulder
    15 #
    16 #   This source code is released for free distribution under the terms of the
    17 #   GNU General Public License as published by the Free Software Foundation.
    18 #  =================================================================================
    19 #
    20 
    21 
    22 CFLAGS = -Wall -g3 -pthread -I./inc
    23 
    24 
    25 
    26 BIN     := ./bin
    27 SRC     := ./src
    28 RES := ./res
    29 SRCS  := $(wildcard $(SRC)/*.c)
    30 OBJS  := $(patsubst $(SRC)/%.c, $(RES)/%.o, $(SRCS))
    31 OBJS_CROSS  := $(patsubst $(SRC)/%.c, $(RES)/%_bbg.o, $(SRCS))
    32 
    33 PROG            := $(BIN)/project1.out
    34 PROG_CROSS  := $(BIN)/project1_bbg.out
    35 
    36 all: $(PROG)
    37 
    38 $(RES)/%.o: $(SRC)/%.c
-   39     mkdir -p $(RES)
|   40     $(CC) $(CFLAGS) -c $< -o $@
    41 
    42 $(PROG): $(OBJS)
-   43     mkdir -p $(BIN)
|   44     $(CC) $(CFLAGS) $^ -o $@ -lrt  -lm -lmraa
    45 clean:
-   46     rm -rf $(BIN)
|   47     rm -rf $(RES)
   ~                                                                                                                    
   ~                                                                                                                    
   ~                                                                                                                    
   ~                                                                                                                    
   ~                                                          
