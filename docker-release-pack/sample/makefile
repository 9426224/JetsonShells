CC = gcc
CFLAGS = -Wall -g

LIBS = -lcqbit_target2mas  -lcjson

LIB_PATH = -L ../cJSON -L ./

SRCS = example.c


all: test_parse

test_parse:$(SRCS) 
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS) $(LIB_PATH)

.PHONY:clean

clean:
	-rm test_parse 