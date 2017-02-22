all: teapot

OBJS = teapot.o

teapot: $(OBJS)
	$(CXX) -o $@ $^ -lGL -lGLU -lglut -lglfw

clean:
	rm -f $(OBJS) teapot

