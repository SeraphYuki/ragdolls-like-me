#ifndef SPELLS_DEF
#define SPELLS_DEF
#include "game.h"
#include "object.h"
#include "particles.h"

enum {
	SPELL_TYPE_AUTOATTACK=1,
	SPELL_TYPE_AOE,	
};

typedef struct {
	Particle particles[100];
	Image particleImage;
	Vec3 pos;
	float speed;
	float dieTime;
	int finished;
	Vec3 vel;
} SpellAutoAttack;

typedef struct {
	Particle particles[100];
	Image groundImage;
	Vec3 pos;
	float speed;
	float dieTime;
	int lastDamage;
	int finished;
	Vec3 vel;
} SpellAOE;

typedef struct Spell Spell;

struct Spell {
	int type;	
	void *data;
	Object *directedAt;
	Object *cameFrom;
};

Object *Spell_AutoAttack_Cast(Game *game, Object *cameFrom, Object *at);
Object *Spell_AOE_Cast(Game *game, Object *cameFrom, Vec3 at);


#endif