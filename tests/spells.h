#ifndef SPELLS_DEF
#define SPELLS_DEF
#include "game.h"
#include "object.h"
#include "particles.h"

enum {
	SPELL_TYPE_AUTOATTACK=1,
	
};

typedef struct {
	Particle particles[100];
	Image particleImage;
	Vec3 pos;
	float speed;
	Vec3 vel;
} SpellAutoAttack;


typedef struct Spell Spell;

struct Spell {

	int type;	
	void *data;

	Object *directedAt;
	Object *cameFrom;
};

Object *Spell_AutoAttack_Cast(Game *game, Object *cameFrom, Object *at);



#endif