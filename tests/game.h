#ifndef GAME_DEF
#define GAME_DEF

#include <stdint.h>

#include "window.h"
#include "pathfinding.h"

typedef struct Object Object;
#define MAX_CHARACTERS 100
#define MAX_MINIONS 100

typedef struct {

	Object *player;
	Object *world;		

	Object *characters[MAX_CHARACTERS];
	int 	nCharacters;		
	Object *minions[MAX_MINIONS];
	int 	nMinions;
	Pathfinder pf;	
} Game;

#define CAMERA_FOV (60.0f * (PI / 180.0f))
#define CAMERA_FAR 70
#define CAMERA_NEAR 0.1
#define CAMERA_ASPECT (WINDOW_INIT_WIDTH/(float)WINDOW_INIT_HEIGHT)

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

float GetDeltaTime(void);

#endif