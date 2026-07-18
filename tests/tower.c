#include "characters.h"
#include "world.h"
#include "shaders.h"
#include "spells.h"
#define GLEW_STATIC
#include <GL/glew.h>

void Tower_OnCollision(Game *game, Object *obj,Object *obj1, Object *obj2, BoundingBox *bb1,
	BoundingBox *bb2, Vec3 axis, float overlap){

	if(obj == obj1 && bb1->collisionFlag & COLLISIONFLAG_RADIUS){
		if(obj2->type == TYPE_CHARACTER ){
			Character *character = (Character*)obj->data;
			Character *character2 = (Character*)obj2->data;
			if(character2->team != character->team && obj2 != character->aggro){
				if(character->aggro){ character->aggro->RemoveUser(character->aggro); } 
				character->aggro = obj2;
				character->aggro->AddUser(character->aggro);
			}
		}
	}
}

void Tower_Update(Game *game, Object *obj){
	Character *character = (Character*)obj->data;
	Tower *tower = (Tower *)character->data;

	if(character->death && Window_GetTicks() - character->deathTime > 500){
		World_RemoveOffScreenUpdatedObject(obj);
		if(character->aggro) character->aggro->RemoveUser(character->aggro);
		World_RemoveObjectFromOctree(obj);
		return;
	}
	
	if(character->aggro == NULL){
		BoundingBox bb;
		memset(&bb,0,sizeof(bb));
		bb.pos = obj->bb.pos;
		if(character->team == 0) obj->bb.pos.x -= 0.001 * GetDeltaTime();
		if(character->team == 1) obj->bb.pos.x += 0.001 * GetDeltaTime();
		
		bb.radius = character->aggroRadius;
		bb.collisionFlag |= COLLISIONFLAG_RADIUS;
		World_ResolveCollisions(game, obj, &bb);
	} else {
		if(Window_GetTicks() - character->lastAttack > 1000){
			Spell_TowerAttack_Cast(game, obj, character->aggro);
			character->aggro = NULL;
			character->lastAttack = Window_GetTicks();
		}	
	}
}

void Tower_Draw(Game *game, Object *obj){
	Character *character = (Character*)obj->data;
	Tower *tower = (Tower *)character->data;

	if(!character->death){
		//Particles_DrawBillboard( game->images[IMAGE_HEALTH-1], &game->particleSystem, 
		//Math_Vec3AddVec3(obj->bb.pos,(Vec3){-0.2,1.4,0}), (Vec2){1*MIN(character->health,1),0.1},(Vec4){1,1,1,1},
		//(Rect2D){0,0,game->images[IMAGE_HEALTH-1].w,game->images[IMAGE_HEALTH-1].h});


		Shaders_UseProgram(TEXTURED_SHADER);

		Shaders_SetModelMatrix(obj->bb.matrix);
		Shaders_UpdateModelMatrix();

		glActiveTexture(GL_TEXTURE0);

		glBindVertexArray(obj->model->vao);
		glCullFace(GL_BACK);
		int curr = 0;

		int k;
		for(k = 0; k < obj->model->nMaterials; k++){
			glBindTexture(GL_TEXTURE_2D, obj->model->materials[k].texture);
			//glUniform4fv(Shaders_GetDiffuseLocation(), 1, (float *)&obj->model->materials[k].diffuse);
			//glUniform4fv(Shaders_GetSpecularLocation(), 1, (float *)&obj->model->materials[k].specular);
			glDrawElements(GL_TRIANGLES, obj->model->nElements[k], GL_UNSIGNED_INT, (void *)(curr * sizeof(GLuint)));
			curr += obj->model->nElements[k];
		}

		glBindVertexArray(0);
	}
}
void Tower_Damage(Game *game, Object *this, Object *cause, float amount){

	//Spell *spell = (Spell*)cause->data;
	 //handle custom logic

	//Character *character = (Character*)this->data;
	//Tower *tower = (Tower *)character->data;

	//if aggro
	//if(character->health < 0.4 && cause){
		//game->cs += 100;
		//character->showGold = 1;
	//}

	//if(character->health <= 0 && character->death == 0){
		//character->death = 1;
		//character->deathTime = Window_GetTicks();
		//return;
	//}		
}

Object *Tower_Create(Game *game, int team){
	Object *obj = Object_Create();
	
	obj->data = malloc(sizeof(Character));
	obj->type = TYPE_CHARACTER;
	Character *character = (Character*)obj->data;
	memset(character, 0, sizeof(Character));
	character->team = team;
	character->aggroRadius = 4;
	character->Damage = Tower_Damage;
	character->data = malloc(sizeof(Tower));
	character->type = CHARACTER_TYPE_TOWER;
	character->health = 1;
	character->moveSpeed = 0.001;
	character->death = 0;
	character->deathTime = Window_GetTicks();	
	Tower *tower = (Tower *)character->data;
	memset(tower, 0, sizeof(Tower));
	Object_SetModel(obj, &game->models[MODEL_TOWER-1]);
	obj->Draw = Tower_Draw;
	obj->Update = Tower_Update;
	obj->OnCollision = Tower_OnCollision;
	obj->bb.collisionFlag |= COLLISIONFLAG_AABB | COLLISIONFLAG_RAY_OBJ;
	obj->bb.renderDebug = 0;
	if(team == 0){
	} else {
	}		
	
	
	obj->bb.scale = (Vec3){1,1,1};
	obj->bb.rot = (Vec3){0,0,0};
	obj->bb.pos = (Vec3){0,0,10};	
	obj->ObjUpdate(obj);
	World_UpdateObjectInOctree(obj);
	World_AddOffScreenUpdatedObject(obj);
	return obj;
}


void SpellTowerAttack_Update(Game *game, Object *obj){
	Spell  *spell = (Spell *)obj->data;
	SpellTowerAttack  *towerAtt = (SpellTowerAttack  *)spell->data;

	if(towerAtt->finished == 1){
		if(Window_GetTicks() - towerAtt->dieTime > 1000){
			World_RemoveOffScreenUpdatedObject(obj);
			World_RemoveObjectFromOctree(obj);
			return;
		}
	}
	Character *directedCharacter = (Character *)spell->directedAt->data;

	if(directedCharacter->death == 1){
		if(towerAtt->finished == 0){
			spell->directedAt->RemoveUser(spell->directedAt);
			spell->cameFrom->RemoveUser(spell->cameFrom);
		}
		towerAtt->dieTime = Window_GetTicks();
		towerAtt->finished = 1;
		World_RemoveOffScreenUpdatedObject(obj);
		World_RemoveObjectFromOctree(obj);

		return;
	}

	if(towerAtt->finished == 0){
		Vec3 path = Math_Vec3SubVec3( spell->directedAt->bb.pos, towerAtt->pos);
		if(Math_Vec3Magnitude(path) < 1){

			if(directedCharacter->Damage) directedCharacter->Damage(game, spell->directedAt, 
			obj, 0.8);
			spell->directedAt->RemoveUser(spell->directedAt);
			spell->cameFrom->RemoveUser(spell->cameFrom);
			towerAtt->finished = 1;
			towerAtt->dieTime = Window_GetTicks();
		}

		path = Math_Vec3SubVec3( spell->directedAt->bb.pos,towerAtt->pos);
		path.y = 0;
		path = Math_Vec3Normalize(path);
		path = Math_Vec3MultFloat(path, towerAtt->speed * Window_GetDeltaTime());
		int j;
		for(j = 0; j < 100; j++){
			towerAtt->particles[j].pos = Math_Vec3AddVec3(towerAtt->particles[j].pos,
				path);
		}


		obj->bb.pos = Math_Vec3AddVec3( obj->bb.pos, path);	
		towerAtt->pos = obj->bb.pos;
		obj->ObjUpdate(obj);
		World_UpdateObjectInOctree(obj);
	}
}

void SpellTowerAttack_Draw(Game *game, Object *obj){
	Spell  *spell = (Spell *)obj->data;
	SpellTowerAttack  *towerAtt = (SpellTowerAttack  *)spell->data;

	Shaders_SetModelMatrix(obj->matrix);
	float invView[16];
	Shaders_GetInvViewMatrix(invView);
	Vec3 forward = (Vec3){invView[2], invView[6], invView[10]};
	Vec3 camPos = (Vec3){invView[3], invView[7], invView[11]};
	
	Particles_DrawParticles(towerAtt->particleImage, 
		&game->particleSystem, towerAtt->particles, 100, 50, forward, camPos, 0);

}


Object *Spell_TowerAttack_Cast(Game *game, Object *cameFrom, Object *at){
	
	at->AddUser(at);
	cameFrom->AddUser(cameFrom);

	Object *obj = Object_Create();
	
	obj->data = malloc(sizeof(Spell));
	Spell *spell = (Spell*)obj->data;	
	memset(spell, 0, sizeof(Spell));
	
	spell->data = malloc(sizeof(SpellTowerAttack));
	SpellTowerAttack *towerAtt = (SpellTowerAttack*)spell->data;
	memset(towerAtt, 0, sizeof(SpellTowerAttack));
	
	towerAtt->speed = 0.005;
	// todo add all resources to game struct
	towerAtt->particleImage = game->images[IMAGE_PARTICLES-1];
	towerAtt->particleImage.nFramesX = 5;
	towerAtt->particleImage.nFramesY = 5;
	Math_Identity(obj->matrix);
	spell->directedAt = at;
	spell->cameFrom = cameFrom;
	spell->type = SPELL_TYPE_AUTOATTACK;

	obj->Draw = SpellTowerAttack_Draw;
	obj->Update = SpellTowerAttack_Update;
	obj->bb.collisionFlag |= COLLISIONFLAG_NONE;

	towerAtt->pos = cameFrom->bb.pos;
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
		towerAtt->particles[j].createTime = Window_GetTicks();
		towerAtt->particles[j].lifeTime = 10000;
		towerAtt->particles[j].pos = Math_Vec3AddVec3(cameFrom->bb.pos,
		 (Vec3){ (-50 + rand()%100)/90.0f,( -50 +  rand()%100)/90.0f,(-50 + rand()%100)/90.0f});
		
		towerAtt->particles[j].pos.y += 10;
		towerAtt->particles[j].size = (Vec2){1,1};
		towerAtt->particles[j].color = (Vec4){0.4,0.0,0.0,0.1};
		towerAtt->particles[j].vel = (Vec3){(-5 + (rand()%10))/20000.0f,(-5 + (rand()%10))/20000.0f,
		(-5 + (rand()%10))/20000.0f};
		towerAtt->particles[j].vel.y -= 0.01;
	}

	return obj;
}
