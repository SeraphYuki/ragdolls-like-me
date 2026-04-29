#include "spells.h"
#include "shaders.h"
#include "world.h"
#include "math.h"

void SpellAutoAttack_Update(Game *game, Object *obj){
	Spell  *spell = (Spell *)obj->data;
	SpellAutoAttack  *autoAttack = (SpellAutoAttack  *)spell->data;


	Vec3 path = Math_Vec3SubVec3( spell->directedAt->bb.pos, autoAttack->pos);
	if(Math_Vec3Magnitude(path) < 1 && autoAttack->finished == 0){
		World_RemoveOffScreenUpdatedObject(spell->directedAt);
		World_RemoveObjectFromOctree(spell->directedAt);
		autoAttack->finished = 1;
		autoAttack->dieTime = Window_GetTicks();
		return;
	}
	
	if(autoAttack->finished == 1 && Window_GetTicks() - autoAttack->dieTime > 1000){
		World_RemoveOffScreenUpdatedObject(obj);
		World_RemoveObjectFromOctree(obj);
		return;
	}
	if(autoAttack->finished == 0){

		path = Math_Vec3SubVec3( spell->directedAt->bb.pos,spell->cameFrom->bb.pos);
		path = Math_Vec3MultFloat(Math_Vec3Normalize(path), autoAttack->speed);
		int j;
		for(j = 0; j < 100; j++){
			autoAttack->particles[j].pos = Math_Vec3AddVec3(autoAttack->particles[j].pos,
				path);
			//autoAttack->particles[j].size = (Vec2){1,1};
			//autoAttack->particles[j].color = (Vec4){0.1,0.1,0.1,0.1};
			//autoAttack->particles[j].vel = (Vec3){(-5 + (rand()%10))/10000.0f,(-5 + (rand()%10))/10000.0f,
			//(-5 + (rand()%10))/10000.0f};
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

	Object *obj = Object_Create();
	obj->data = malloc(sizeof(Spell));
	Spell *spell = (Spell*)obj->data;	
	memset(spell, 0, sizeof(Spell));
	spell->data = malloc(sizeof(SpellAutoAttack));
	SpellAutoAttack *autoAttack = (SpellAutoAttack*)spell->data;
	memset(autoAttack, 0, sizeof(SpellAutoAttack));
	autoAttack->speed = 0.005;
	// todo add all resources to game struct
	autoAttack->particleImage = ImageLoader_CreateImage("Resources/smoke.png",1);
	autoAttack->particleImage.nFramesX = 5;
	autoAttack->particleImage.nFramesY = 5;
	Math_Identity(obj->matrix);
	
	spell->directedAt = at;
	spell->cameFrom = cameFrom;
	spell->type = SPELL_TYPE_AUTOATTACK;

	obj->Draw = SpellAutoAttack_Draw;
	obj->Update = SpellAutoAttack_Update;
	obj->bb.collisionFlag |= COLLISIONFLAG_NONE;

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
		autoAttack->particles[j].createTime = SDL_GetTicks();
		autoAttack->particles[j].lifeTime = 10000;
		autoAttack->particles[j].pos = Math_Vec3AddVec3(cameFrom->bb.pos,
		 (Vec3){ (-50 + rand()%100)/90.0f,(-50 + rand()%100)/90.0f,(-50 + rand()%100)/90.0f});
		autoAttack->particles[j].size = (Vec2){1,1};
		autoAttack->particles[j].color = (Vec4){0.1,0.1,0.1,0.1};
		autoAttack->particles[j].vel = (Vec3){(-5 + (rand()%10))/10000.0f,(-5 + (rand()%10))/10000.0f,
		(-5 + (rand()%10))/10000.0f};
	}

	//World_RemoveOffScreenUpdatedObject(at);
	//World_RemoveObjectFromOctree(at);

	return obj;
}
