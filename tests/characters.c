#include "characters.h"
#include "world.h"
#include "shaders.h"
#include "spells.h"
#define GLEW_STATIC
#include <GL/glew.h>

void Minion_OnCollision(Game *game, Object *obj,Object *obj1, Object *obj2, BoundingBox *bb1,
	BoundingBox *bb2, Vec3 axis, float overlap){

	if(!(bb2->collisionFlag & COLLISIONFLAG_INVISIBLE) && obj2->type == TYPE_CHARACTER){
		//Vec3 resolve = Math_Vec3MultFloat(axis, -overlap);
		//obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,resolve);
		//obj->ObjUpdate(obj);
		//World_UpdateObjectInOctree(obj);
	}
	
	if(obj == obj1 && bb1->collisionFlag & COLLISIONFLAG_RADIUS){
		if(obj2->type == TYPE_CHARACTER ){
			Character *character = (Character*)obj->data;
			Character *character2 = (Character*)obj2->data;
			if(character2->type != CHARACTER_TYPE_MINION){
				character->aggro = obj2;
				character->aggro->AddUser(character->aggro);
			}
		}
	}
}

void Minion_Update(Game *game, Object *obj){
	Character *character = (Character*)obj->data;
	Minion *minion = (Minion *)character->data;

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
		bb.radius = character->aggroRadius;
		bb.collisionFlag |= COLLISIONFLAG_RADIUS;
		World_ResolveCollisions(game, obj, &bb);
	} else {
		if(Window_GetTicks() - character->lastAttack > 4000){
			Spell_AutoAttack_Cast(game, obj, character->aggro);
			character->lastAttack = Window_GetTicks();
		}	
	}

	if(Window_GetTicks() - character->lastTime > 1000 && character->aggro != NULL){
		
		Vec3 toPos = character->aggro->bb.pos;
		toPos = Math_Vec3SubVec3(toPos, 
		Math_Vec3MultFloat(
		Math_Vec3Normalize(Math_Vec3SubVec3(toPos,obj->bb.pos)), character->aggroRadius));
		srand(character->index);
		int randval = rand();
		toPos = Math_Vec3AddVec3(toPos,
			(Vec3){ (-50 + randval%100)/40.0f, 0, (-50 + randval%100)/40.0f});

		Pathfinding_FindPathGrid(&game->pf,obj->bb.pos, toPos, &character->path);
		
		character->lastTime = Window_GetTicks();
		// this doesnt work very well and is slow idk about it
		//Pathfinding_SetClosedBoundingBoxDynamic(&game->pf, &obj->bb);
	}
	character->health -= 0.0001 * Window_GetDeltaTime();
	character->Damage(game,obj,NULL);

	if(character->health <=  0) return;
	Vec3 pos = obj->bb.pos;
	Vec3 toPos = character->path.path[character->onPath];
	pos.y = toPos.y;

	if(character->path.nPath > 0 && character->onPath < character->path.nPath-1 && 
		Math_Vec3Magnitude(Math_Vec3SubVec3(toPos, pos)) < 0.1){
	
		character->onPath++; 
		if(character->onPath >= character->path.nPath) character->onPath = 0;
		if(character->onPath < character->path.nPath){
			character->moveToPos = character->path.path[character->onPath];
			character->moveToPos.y = obj->bb.pos.y;
		}
	}

	if(character->path.nPath > 0){
		toPos = character->path.path[character->onPath];
		character->force = Math_Vec3SubVec3(toPos,pos);
		character->moveToPos.y = obj->bb.pos.y;

	     character->force = Math_Vec3Normalize(character->force);
		
		Vec3 xz = character->force;
		xz.y = 0;
		if(Math_Vec3Magnitude(xz) > 0){
			xz = Math_Vec3Normalize(xz);
			Vec3 forward = Math_Rotate((Vec3){0,0,-1},obj->bb.rot);
			float dot = Math_Vec3Dot(forward,xz);
			obj->bb.rot.y -= dot * character->moveSpeed * Window_GetDeltaTime();
		}   
		  character->force = Math_Vec3MultFloat(character->force, Window_GetDeltaTime() *
		 character->moveSpeed);

		obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos, character->force);

		obj->ObjUpdate(obj);

		World_UpdateObjectInOctree(obj);
	}
	//World_ResolveCollisions(game, obj, &obj->bb);
}

void Minion_Draw(Game *game, Object *obj){
	Character *character = (Character*)obj->data;
	Minion *minion = (Minion *)character->data;

	

	if(character->showGold){
		Particles_DrawBillboard(game->images[IMAGE_GOLD-1], 
			&game->particleSystem, Math_Vec3AddVec3(obj->bb.pos,(Vec3){0,1,0}),
			 (Vec2){1.5,1.5}, (Vec4){1,1,1,1},
			(Rect2D){0,0,1,1});
	}

	if(!character->death){
		Particles_DrawBillboard( game->images[IMAGE_HEALTH-1], &game->particleSystem, 
		Math_Vec3AddVec3(obj->bb.pos,(Vec3){-0.2,1.4,0}), (Vec2){1*MIN(character->health,1),0.1},(Vec4){1,1,1,1},
		(Rect2D){0,0,game->images[IMAGE_HEALTH-1].w,game->images[IMAGE_HEALTH-1].h});

		minion->playingAnims[0].into += minion->animDir * Window_GetDeltaTime() / 40.1f;
				
		if(minion->playingAnims[0].into > minion->animation.length || minion->playingAnims[0].into < 0){
			minion->animDir = - minion->animDir;
		}

		Skeleton_Update(&minion->skeleton, minion->playingAnims, 1);
		Object_UpdateSkeleton(obj, &minion->skeleton);
		Skeleton_Apply(&minion->skeleton);

		Shaders_UseProgram(SKELETAL_ANIMATION_SHADER);

		Shaders_SetModelMatrix(obj->bb.matrix);
		Shaders_UpdateModelMatrix();

		glUniform4fv(Shaders_GetBonesLocation(), obj->skeleton->nBones * 3, &obj->skeleton->matrices[0].x);
		glActiveTexture(GL_TEXTURE0);
		glBindVertexArray(obj->model->vao);

		int curr = 0;


		int k;
		for(k = 0; k < obj->model->nMaterials; k++){

			glBindTexture(GL_TEXTURE_2D, obj->model->materials[k].texture);
			glUniform4fv(Shaders_GetDiffuseLocation(), 1, (float *)&obj->model->materials[k].diffuse);
			glUniform4fv(Shaders_GetSpecularLocation(), 1, (float *)&obj->model->materials[k].specular);
			glDrawElements(GL_TRIANGLES, obj->model->nElements[k], GL_UNSIGNED_INT, (void *)(curr * sizeof(GLuint)));
			curr += obj->model->nElements[k];
		}

		glBindVertexArray(0);
	}
}
void Minion_Damage(Game *game, Object *this, Object *cause){

	//Spell *spell = (Spell*)cause->data;
	// handle custom logic

	Character *character = (Character*)this->data;
	Minion *minion = (Minion *)character->data;

	//if aggro
	if(character->health < 0.4 && cause){
		game->cs += 100;
		character->showGold = 1;
	}

	if(character->health <= 0 && character->death == 0){
		character->death = 1;
		character->deathTime = Window_GetTicks();
		return;
	}		
}

Object *Minion_Create(Game *game, Model *model){
	Object *obj = Object_Create();
	
	obj->data = malloc(sizeof(Character));
	obj->type = TYPE_CHARACTER;
	Character *character = (Character*)obj->data;
	memset(character, 0, sizeof(Character));
	
	character->aggroRadius = 4;
	character->Damage = Minion_Damage;
	character->data = malloc(sizeof(Minion));
	character->type = CHARACTER_TYPE_MINION;
	character->health = 1;
	character->moveSpeed = 0.001;
	character->death = 0;
	character->deathTime = Window_GetTicks();	
	Minion *minion = (Minion *)character->data;
	memset(minion, 0, sizeof(Minion));
	
	minion->animDir = 1;
	Object_SetModel(obj, model);
	Skeleton_Copy(&minion->skeleton, &model->skeleton);
	Object_SetSkeleton(obj,&minion->skeleton);
	
	minion->playingAnims[0] = (PlayingAnimation){
		 .active = 1,
		 .weight = 1,
		 .into = 0,
		 .anim = &game->animations[ANIMATION_MINION-1],
	};

	obj->Draw = Minion_Draw;
	obj->Update = Minion_Update;
	obj->OnCollision = Minion_OnCollision;
	obj->bb.collisionFlag |= COLLISIONFLAG_AABB | COLLISIONFLAG_RAY_OBJ;
	obj->bb.renderDebug = 0;
	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,
	(Vec3){ (-50 + rand()%100)/20.0f, 1, (-50 + rand()%100)/10.0f});
	obj->bb.scale = (Vec3){0.3,0.3,0.3};
	obj->bb.rot = (Vec3){0,0,0};
	obj->bb.pos.z += 14;	
	obj->bb.pos.x -= 5;	
	obj->ObjUpdate(obj);
	World_UpdateObjectInOctree(obj);
	World_AddOffScreenUpdatedObject(obj);
	return obj;
}