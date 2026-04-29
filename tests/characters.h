#ifndef CHARACTERS_DEF
#define CHARACTERS_DEF

#include "game.h"
#include "particles.h"
#include "object.h"
#include "pathfinding.h"

enum {
	CHARACTER_TYPE_MINION=1,
};

typedef struct {
	Skeleton skeleton;
	Animation animation;
	float animDir;
	PlayingAnimation playingAnims[1];
} Minion;


typedef struct Character Character;

struct Character {
	float health;	
	float moveSpeed;
	int onPath;
	PathfinderPath path;
	Vec3 moveToPos;
	Vec3 force;
	int lastTime;	

	int type;	

	void *data;

};

Object *Minion_Create(Game *game, Model *model);

#endif