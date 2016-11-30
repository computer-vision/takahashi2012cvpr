demo: demo.cc sub_solveP3P.h tnm.h
	g++ -Wall demo.cc -o demo `pkg-config --libs --cflags opencv`

run: demo
	./demo

clean:
	@rm -vf demo *.bak *~
