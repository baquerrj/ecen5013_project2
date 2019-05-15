#
#  =================================================================================
#     @file     Makefile
#     @brief
#
#   Makefile for BeagleBone Green Temperature and Light Application
#
#     @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
#
#     @internal
#        Created:  04/01/2019
#       Revision:  none
#       Compiler:  gcc
#   Organization:  University of Colorado: Boulder
#
#   This source code is released for free distribution under the terms of the
#   GNU General Public License as published by the Free Software Foundation.
#  =================================================================================
#

CC	:= gcc

BIN 	:= bin
SRC 	:= source
INC 	:= include
BUILD	:= build
SRCS  	:= $(wildcard ./$(SRC)/*.c)
OBJS  	:= $(patsubst ./$(SRC)/%.c, ./$(BUILD)/%.o, $(SRCS))

PROG	:= $(BIN)/remote_node_comm

CFLAGS	:= -Wall -g3 -pthread -I./$(INC)
LFLAGS  := -lrt -lm -lmraa
all: $(PROG)

$(shell mkdir -p ./$(BUILD))
$(shell mkdir -p ./$(BIN))

$(BUILD)/%.o: $(SRC)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(PROG): $(OBJS)
	@echo $(OBJS)
#	$(CC) $(CFLAGS) $^ -o $@ -lrt  -lm -lmraa
	$(CC) $(CFLAGS) $^ -o $@ $(LFLAGS)

clean:
	rm -rf $(BIN)/*
	rm -rf $(BUILD)/*
