#ifndef MINION_DEF
#define MINION_DEF

#include "object.h"
#include "pathfinding.h"
typedef struct {
	float health;	
	float moveSpeed;
	int lastTime;	
	Skeleton skeleton;
	Animation animation;
	float animDir;
	PlayingAnimation playingAnims[1];
	int onPath;
	PathfinderPath path;
	Vec3 moveToPos;
} Minion;

Object *Minion_Create(Model *model);

#endif