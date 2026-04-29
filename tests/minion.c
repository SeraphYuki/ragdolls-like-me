#include "minion.h"
#include "world.h"
#include "shaders.h"
#include <GL/glew.h>

void Minion_OnCollision(Object *obj,Object *obj1, Object *obj2, BoundingBox *bb1,
	BoundingBox *bb2, Vec3 axis, float overlap){
	
	Minion *minion = (Minion *)obj->data;
	
	if(obj2->type == TYPE_MINION){
		Vec3 resolve = Math_Vec3MultFloat(axis, -overlap);
		obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,resolve);
		obj->ObjUpdate(obj);
		World_UpdateObjectInOctree(obj);
	}
}

void Minion_Update(Game *game, Object *obj){
	Minion *minion = (Minion *)obj->data;
	if(Window_GetTicks() - minion->lastTime > 1000){
		Pathfinding_FindPathGrid(&game->pf,obj->bb.pos, game->player->bb.pos, &minion->path);
		minion->lastTime - Window_GetTicks();
	}

	Vec3 pos = obj->bb.pos;
	Vec3 toPos = minion->path.path[minion->onPath];
	pos.y = toPos.y;

	if(minion->path.nPath > 0 && minion->onPath < minion->path.nPath-1 && 
		Math_Vec3Magnitude(Math_Vec3SubVec3(toPos, pos)) < 0.1){
	
		minion->onPath++; 
		if(minion->onPath >= minion->path.nPath) minion->onPath = 0;
		if(minion->onPath < minion->path.nPath){
			minion->moveToPos = minion->path.path[minion->onPath];
			minion->moveToPos.y = obj->bb.pos.y;
		}
	}

	toPos = minion->path.path[minion->onPath];
	minion->force = Math_Vec3SubVec3(toPos,pos);
	minion->moveToPos.y = obj->bb.pos.y;

     minion->force = Math_Vec3Normalize(minion->force);
	
	Vec3 xz = minion->force;
	xz.y = 0;
	if(Math_Vec3Magnitude(xz) > 0){
		xz = Math_Vec3Normalize(xz);
		Vec3 forward = Math_Rotate((Vec3){0,0,-1},obj->bb.rot);
		float dot = Math_Vec3Dot(forward,xz);
		obj->bb.rot.y -= dot * minion->moveSpeed;
	}   
	  minion->force = Math_Vec3MultFloat(minion->force, Window_GetDeltaTime() * minion->moveSpeed);

	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos, minion->force);

	obj->ObjUpdate(obj);
	World_UpdateObjectInOctree(obj);
	World_ResolveCollisions(obj, &obj->bb);
}

void Minion_Draw(Object *obj){
	Minion *minion = (Minion *)obj->data;
	
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

Object *Minion_Create(Model *model){
	Object *obj = Object_Create();
	obj->data = malloc(sizeof(Minion));
	Minion *minion = (Minion *)obj->data;
	memset(minion, 0, sizeof(Minion));
	minion->animDir = 1;
	minion->moveSpeed = 0.001;
	Object_SetModel(obj, model);
	Skeleton_Copy(&minion->skeleton, &model->skeleton);
	Object_SetSkeleton(obj,&minion->skeleton);
	Animation_Load(&minion->animation, "Resources/minion_ArmatureAction.anm");
	minion->playingAnims[0] = (PlayingAnimation){
		 .active = 1,
		 .weight = 1,
		 .into = 0,
		 .anim = &minion->animation,
	};

	obj->Draw = Minion_Draw;
	obj->Update = Minion_Update;
	obj->OnCollision = Minion_OnCollision;
	obj->bb.collisionFlag |= COLLISIONFLAG_AABB;
	obj->bb.renderDebug = 0;
	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,
	(Vec3){ (-50 + rand()%100)/20.0f, 1, (-50 + rand()%100)/20.0f});
	obj->bb.scale = (Vec3){0.3,0.3,0.3};
	obj->bb.rot = (Vec3){0,0,0};
	obj->bb.pos.z += 10;
	obj->type = TYPE_MINION;
	obj->ObjUpdate(obj);
	World_UpdateObjectInOctree(obj);
	World_AddOffScreenUpdatedObject(obj);
	return obj;
}