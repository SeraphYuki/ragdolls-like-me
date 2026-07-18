#ifndef CHARACTERS_DEF
#define CHARACTERS_DEF

#include "game.h"
#include "particles.h"
#include "object.h"
#include "pathfinding.h"

enum {
	CHARACTER_NONE,
	CHARACTER_TYPE_MINION=1,
	CHARACTER_TYPE_PLAYER,
	CHARACTER_TYPE_TOWER,
};

typedef struct {
	Skeleton skeleton;
	Animation animation;
	float animDir;
	PlayingAnimation playingAnims[1];
} Minion;

typedef struct {
	int lastAttack;
} Tower;

typedef struct Character Character;

struct Character {
	Object *aggro;

	float health;	
	int team;
	int index;
	int death;
	int deathTime;
	int lastAttack;
	float aggroRadius;
	float moveSpeed;
	int showGold;
	int onPath;
	PathfinderPath path;
	Vec3 moveToPos;
	Vec3 force;
	int lastTime;	

	int type;	
	
	void (*Damage)(Game *game, Object *obj, Object *cause, float amount);
	void *data;

};

Object *Minion_Create(Game *game, Model *model, int team);
Object *Tower_Create(Game *game, int team);

#endif