#ifndef GAME_DEF
#define GAME_DEF

#include "window.h"
#include "particles.h"
#include "pathfinding.h"
#include "mesh.h"
#include "image_loader.h"

typedef struct Object Object;
#define MAX_CHARACTERS 100
#define MAX_SPELLS 100

enum {
	MODEL_WORLD=1,
	MODEL_MINION,
	MODEL_PLAYER,
	NUM_MODELS,
};

enum {
	IMAGE_PARTICLES = 1,
	IMAGE_HEALTH,
	IMAGE_GOLD,
	NUM_IMAGES,
};

enum {
	ANIMATION_PLAYER = 1,
	ANIMATION_MINION,
	NUM_ANIMATIONS,
};

typedef struct Game Game;

struct Game {

	Object *player;
	Object *world;		

	Object *characters[MAX_CHARACTERS];
	int 			nCharacters;		
	Object 		*spells[MAX_SPELLS];
	int 			nSpells;
	Pathfinder 		pf;	
	Ray ray;
	Animation 		animations[NUM_ANIMATIONS];
	Model 			models[NUM_MODELS];
	Image			images[NUM_IMAGES];
	ParticleSystem 	particleSystem;
	int 			cs;
};

#define CAMERA_FOV (60.0f * (PI / 180.0f))
#define CAMERA_FAR 70
#define CAMERA_NEAR 0.1
#define CAMERA_ASPECT (WINDOW_INIT_WIDTH/(float)WINDOW_INIT_HEIGHT)


float GetDeltaTime(void);

#endif