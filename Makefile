CC = g++
FLAGS = -g -o

# Change the fourth entry in this line to point to the Eigen directory
INCLUDE = -I/usr/X11R6/include -I/usr/include/GL -I/usr/include -I/home/thepopeofrandomness/Documents/cs174

LIBDIR = -L/usr/X11R6/lib -L/usr/local/lib
SOURCES = util.cpp halfEdge.cpp game.cpp boundaryObject.cpp object.cpp physicalObject.cpp
LIBS = -lGLEW -lGL -lGLU -lglut -lm

EXENAME = game

all: game

game: $(SOURCES)
	$(CC) $(FLAGS) $(EXENAME) $(INCLUDE) $(LIBDIR) $(SOURCES) $(LIBS)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean

