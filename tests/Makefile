CC=gcc

# debugging on, source compiled into exe currently

#CFLAGS  = -Wall -g $(shell pkg-config --cflags freetype2) -m64
LFLAGS = -pg

# CFLAGS   = -m64 -pipe

#SDL_LDFLAGS = $(shell sdl2-config --libs)

#GLEW_LDFLAGS = $(shell pkg-config glew --libs)


#LIBS= $(SDL_LDFLAGS) $(GLEW_LDFLAGS) -lfreetype -lpng -lz -lm -L.-lthoth.a

CFLAGS =  $(shell pkg-config --cflags sdl3) $(shell pkg-config --cflags freetype2) -pg
FREETYPELIBS = $(shell pkg-config --libs freetype2)
GLEWLIBS = $(shell pkg-config  --libs glew)
SDLLIBS = $(shell pkg-config --libs sdl2)
LIBS = -L. -lthoth -lm -lpng  $(GLEWLIBS) $(SDLLIBS) $(FREETYPELIBS) -pg

SOURCES=main.c image_loader.c math.c shaders.c window.c mesh.c \
bounding_box.c octree.c world.c skybox.c utils.c object.c deflate.c log.c \
 ui.c freetype.c text.c particles.c pathfinding.c characters.c spells.c tower.c physics.c


OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) $(LFLAGS) $(LIBS) -o $@

.c.o:
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm *.o

# remember readelf -d main
# then just sudo find / -name 'whaever.so.0'