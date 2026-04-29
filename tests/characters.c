#include "characters.h"
#include "world.h"
#include "shaders.h"
#include <GL/glew.h>

void Minion_OnCollision(Game *game, Object *obj,Object *obj1, Object *obj2, BoundingBox *bb1,
	BoundingBox *bb2, Vec3 axis, float overlap){
	Character *character = (Character*)obj->data;
	Minion *minion = (Minion *)character->data;

	if(obj == obj1 && obj2->type == TYPE_CHARACTER){
		Character *character2 = (Character*)obj2->data;

		if(character2->type == CHARACTER_TYPE_MINION){
			Vec3 resolve = Math_Vec3MultFloat(axis, -overlap);
			obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,resolve);
			obj->ObjUpdate(obj);
			World_UpdateObjectInOctree(obj);
		}
	}
}

void Minion_Update(Game *game, Object *obj){
	Character *character = (Character*)obj->data;
	Minion *minion = (Minion *)character->data;
	if(Window_GetTicks() - character->lastTime > 1000){
		Pathfinding_FindPathGrid(&game->pf,obj->bb.pos, game->player->bb.pos, &character->path);
		character->lastTime - Window_GetTicks();
	}

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
		obj->bb.rot.y -= dot * character->moveSpeed;
	}   
	  character->force = Math_Vec3MultFloat(character->force, Window_GetDeltaTime() * character->moveSpeed);

	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos, character->force);

	obj->ObjUpdate(obj);
	World_UpdateObjectInOctree(obj);
	World_ResolveCollisions(game, obj, &obj->bb);
}

void Minion_Draw(Game *game, Object *obj){
	Character *character = (Character*)obj->data;
	Minion *minion = (Minion *)character->data;
	
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

Object *Minion_Create(Game *game, Model *model){
	Object *obj = Object_Create();
	
	obj->data = malloc(sizeof(Character));
	obj->type = TYPE_CHARACTER;
	Character *character = (Character*)obj->data;
	memset(character, 0, sizeof(Character));
	character->data = malloc(sizeof(Minion));
	Minion *minion = (Minion *)character->data;
	character->type = CHARACTER_TYPE_MINION;
	memset(minion, 0, sizeof(Minion));
	minion->animDir = 1;
	character->moveSpeed = 0.001;
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
	obj->bb.collisionFlag |= COLLISIONFLAG_AABB;
	obj->bb.collisionFlag |= COLLISIONFLAG_SAT;
		obj->bb.renderDebug = 0;
	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,
	(Vec3){ (-50 + rand()%100)/20.0f, 1, (-50 + rand()%100)/20.0f});
	obj->bb.scale = (Vec3){0.3,0.3,0.3};
	obj->bb.rot = (Vec3){0,0,0};
	obj->bb.pos.z += 10;
	obj->ObjUpdate(obj);
	World_UpdateObjectInOctree(obj);
	World_AddOffScreenUpdatedObject(obj);
	return obj;
}