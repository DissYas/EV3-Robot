all:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment  -c tester.c -o tester.o
	gcc tester.o -Wall -lm -lev3dev-c -lbluetooth -o tester

run:
	./tester
