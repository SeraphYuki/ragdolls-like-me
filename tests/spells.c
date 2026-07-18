#include "spells.h"
#include "shaders.h"
#include "world.h"
#include "math.h"
#include "characters.h"

void SpellAutoAttack_Update(Game *game, Object *obj){
	Spell  *spell = (Spell *)obj->data;
	SpellAutoAttack  *autoAttack = (SpellAutoAttack  *)spell->data;

	if(autoAttack->finished == 1){
		if(Window_GetTicks() - autoAttack->dieTime > 1000){
			World_RemoveOffScreenUpdatedObject(obj);
			World_RemoveObjectFromOctree(obj);
			return;
		}
	}
	Character *directedCharacter = (Character *)spell->directedAt->data;

	if(directedCharacter->death == 1){
		if(autoAttack->finished == 0){
			spell->directedAt->RemoveUser(spell->directedAt);
			spell->cameFrom->RemoveUser(spell->cameFrom);
		}
		autoAttack->dieTime = Window_GetTicks();
		autoAttack->finished = 1;
		World_RemoveOffScreenUpdatedObject(obj);
		World_RemoveObjectFromOctree(obj);

		return;
	}

	if(autoAttack->finished == 0){
		Vec3 path = Math_Vec3SubVec3( spell->directedAt->bb.pos, autoAttack->pos);
		if(Math_Vec3Magnitude(path) < 1){


			if(directedCharacter->Damage) directedCharacter->Damage(game, spell->directedAt, obj, 0.3);
			spell->directedAt->RemoveUser(spell->directedAt);
			spell->cameFrom->RemoveUser(spell->cameFrom);
			autoAttack->finished = 1;
			autoAttack->dieTime = Window_GetTicks();
		}

		path = Math_Vec3SubVec3( spell->directedAt->bb.pos,autoAttack->pos);
		path.y = 0;
		path = Math_Vec3Normalize(path);
		path = Math_Vec3MultFloat(path, autoAttack->speed * Window_GetDeltaTime());
		int j;
		for(j = 0; j < 100; j++){
			autoAttack->particles[j].pos = Math_Vec3AddVec3(autoAttack->particles[j].pos,
				path);
		}


		obj->bb.pos = Math_Vec3AddVec3( obj->bb.pos, path);	
		autoAttack->pos = obj->bb.pos;
		obj->ObjUpdate(obj);
		World_UpdateObjectInOctree(obj);
	}
}

void SpellAutoAttack_Draw(Game *game, Object *obj){
	Spell  *spell = (Spell *)obj->data;
	SpellAutoAttack  *autoAttack = (SpellAutoAttack  *)spell->data;

	Shaders_SetModelMatrix(obj->matrix);
	float invView[16];
	Shaders_GetInvViewMatrix(invView);
	Vec3 forward = (Vec3){invView[2], invView[6], invView[10]};
	Vec3 camPos = (Vec3){invView[3], invView[7], invView[11]};
	
	Particles_DrawParticles(autoAttack->particleImage, 
		&game->particleSystem, autoAttack->particles, 100, 50, forward, camPos, 0);

}


Object *Spell_AutoAttack_Cast(Game *game, Object *cameFrom, Object *at){
	
	at->AddUser(at);
	cameFrom->AddUser(cameFrom);

	Object *obj = Object_Create();
	
	obj->data = malloc(sizeof(Spell));
	Spell *spell = (Spell*)obj->data;	
	memset(spell, 0, sizeof(Spell));
	
	spell->data = malloc(sizeof(SpellAutoAttack));
	SpellAutoAttack *autoAttack = (SpellAutoAttack*)spell->data;
	memset(autoAttack, 0, sizeof(SpellAutoAttack));
	
	autoAttack->speed = 0.005;
	// todo add all resources to game struct
	autoAttack->particleImage = game->images[IMAGE_PARTICLES-1];
	autoAttack->particleImage.nFramesX = 5;
	autoAttack->particleImage.nFramesY = 5;
	Math_Identity(obj->matrix);
	spell->directedAt = at;
	spell->cameFrom = cameFrom;
	spell->type = SPELL_TYPE_AUTOATTACK;

	obj->Draw = SpellAutoAttack_Draw;
	obj->Update = SpellAutoAttack_Update;
	obj->bb.collisionFlag |= COLLISIONFLAG_NONE;

	autoAttack->pos = cameFrom->bb.pos;
	obj->bb.pos = cameFrom->bb.pos;
	obj->bb.scale = (Vec3){0.3,0.3,0.3};
	obj->bb.rot = (Vec3){0,0,0};
	obj->type = TYPE_SPELL;
	obj->bb.cube = (Cube){-2,-2,-2,4,4,4};
	obj->ObjUpdate(obj);
	World_UpdateObjectInOctree(obj);
	World_AddOffScreenUpdatedObject(obj);


	int j;
	for(j = 0; j < 100; j++){
		autoAttack->particles[j].createTime = Window_GetTicks();
		autoAttack->particles[j].lifeTime = 10000;
		autoAttack->particles[j].pos = Math_Vec3AddVec3(cameFrom->bb.pos,
		 (Vec3){ (-50 + rand()%100)/90.0f,(-50 + rand()%100)/90.0f,(-50 + rand()%100)/90.0f});
		autoAttack->particles[j].size = (Vec2){1,1};
		autoAttack->particles[j].color = (Vec4){0.1,0.1,0.1,0.1};
		autoAttack->particles[j].vel = (Vec3){(-5 + (rand()%10))/20000.0f,(-5 + (rand()%10))/20000.0f,
		(-5 + (rand()%10))/20000.0f};
	}

	return obj;
}

void Spell_AOE_OnCollision(Game *game, Object *obj,Object *obj1, Object *obj2, BoundingBox *bb1,
	BoundingBox *bb2, Vec3 axis, float overlap){
	
	Spell  *spell = (Spell *)obj->data;
	SpellAOE  *aoe = (SpellAOE  *)spell->data;

	// we caused it.
	if(obj == obj1 && obj2->type == TYPE_CHARACTER){
		Character *character = (Character*)obj2->data;
		character->Damage(game, obj2, obj, 0.4);
	}
}

void Spell_AOE_Update(Game *game, Object *obj){
	Spell *spell = (Spell*)obj->data;	
	SpellAOE *aoe = (SpellAOE*)spell->data;
	if(Window_GetTicks() - aoe->dieTime > 10000){
		World_RemoveOffScreenUpdatedObject(obj);
		World_RemoveObjectFromOctree(obj);
		spell->cameFrom->RemoveUser(spell->cameFrom);
		return;
	}

	if(Window_GetTicks() - aoe->lastDamage > 800){
		World_ResolveCollisions(game, obj, &obj->bb);
		aoe->lastDamage = Window_GetTicks();
	}
}

void Spell_AOE_Draw(Game *game, Object *obj){
	Spell *spell = (Spell*)obj->data;	
	SpellAOE *aoe = (SpellAOE*)spell->data;

	float identity[16];
	Math_Identity(identity);
	Shaders_UseProgram(TEXTURELESS_SHADER);
	Shaders_SetModelMatrix(identity);
	Shaders_UpdateModelMatrix();
	Cube cube = obj->bb.wsCube;
	cube.y += 1.5;
	cube.h = 0.1;
	World_DrawX(cube);
}

Object *Spell_AOE_Cast(Game *game, Object *cameFrom, Vec3 pos){

	Object *obj = Object_Create();
	
	obj->data = malloc(sizeof(Spell));
	Spell *spell = (Spell*)obj->data;	
	memset(spell, 0, sizeof(Spell));
	
	spell->data = malloc(sizeof(SpellAOE));
	SpellAOE *aoe = (SpellAOE*)spell->data;
	memset(aoe, 0, sizeof(SpellAOE));

	cameFrom->AddUser(cameFrom);
	spell->cameFrom = cameFrom;
	
	aoe->dieTime = Window_GetTicks();
	aoe->lastDamage = Window_GetTicks();
	obj->OnCollision = Spell_AOE_OnCollision;
	obj->Update = Spell_AOE_Update;
	obj->Draw = Spell_AOE_Draw;
	obj->bb.collisionFlag |= COLLISIONFLAG_SAT;
	obj->bb.collisionFlag |= COLLISIONFLAG_INVISIBLE;
	obj->bb.pos = pos;
	obj->bb.scale = (Vec3){1,1,1};
	obj->bb.rot = (Vec3){0,0,0};
	obj->type = TYPE_SPELL;
	obj->bb.cube = (Cube){-1.5,-1.5,-1.5,3,3,3};
	obj->ObjUpdate(obj);
	
	World_AddOffScreenUpdatedObject(obj);
	World_UpdateObjectInOctree(obj);	
	return obj;
}