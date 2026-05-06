#define GLEW_STATIC
#include <GL/glew.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_opengl.h>
#include <SDL2/SDL.h>

#include "characters.h"
#include "spells.h"
#include "pathfinding.h"
#include "game.h"
#include "particles.h"
#include "freetype.h"
//#include "thoth/thoth.h"
#include "sound.h"
#include "world.h"
#include "mesh.h"
#include "image_loader.h"
#include "shaders.h"
#include "ui.h"
#include "object.h"
#include "skybox.h"
#define WINDOW_WIDTH 960 
#define WINDOW_HEIGHT 544

#define NUM_MINIONS 20


static Game game;

static float invView[16]; 
static float invPersp[16];

static int onPath = 0;
static PathfinderPath pfpath;

//static Thoth_t *thoth;


static Particle particles[100];
static Image particleImg, billboardImg;

static Skeleton playerSkel;
static PlayingAnimation playingAnims[1];

static Skybox skybox;

static float mouseSensitivity = 0.0005;
static float moveSpeed = 0.005;
static char movingDirs[5];
static Vec3 moveToPos;

float persp[16], view[16], model[16];
static Vec3 rotation = {0,-1,0};
static Vec2 mousepos = {0,0};
static Vec3 position = {4,6,-4};
static Vec3 renderpos = {-2,5,4};
static Vec3 lookatPos = {0,2,0};

static UI ui;

static int castingAOE = 0;
static Vec3 castingPos;

static void BeginCast(){
	castingAOE = 1;
}


float GetDeltaTime(void){
	  return Window_GetDeltaTime();
}
static void onThrow(Object *obj, Object *obj2, BoundingBox *bb, BoundingBox *bb2, Vec3 axis, float overlap){
	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,Math_Vec3MultFloat(axis, -overlap));
	Vec3 satAxis;
	float satOverlap = 0;
	if(Object_SkeletonCollision(&obj->bb, &obj2->skelBb, &satAxis, &satOverlap)){
		obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,Math_Vec3MultFloat(satAxis, -satOverlap));
	}
	obj->ObjUpdate(obj);
}

static void onCube(Game *game, Object *obj, Object *obj1, 
Object *obj2, BoundingBox *bb, BoundingBox *bb2, Vec3 axis, float overlap){
	
	if(bb2->collisionFlag & COLLISIONFLAG_POLYSOUP){
		//bb->pos.y = 4;
		int k;
		for(k = 0; k < bb2->soup.nTris*3; k+=3){
			Vec3 p0 = bb2->soup.verts[k];
			Vec3 p1 = bb2->soup.verts[k+1];
			Vec3 p2 = bb2->soup.verts[k+2];

			Vec3 point;
			p0 = Math_Vec3MultVec3(p0, bb2->scale);
			p1 = Math_Vec3MultVec3(p1, bb2->scale);
			p2 = Math_Vec3MultVec3(p2, bb2->scale);
			p0 = Math_Vec3AddVec3(p0, bb2->pos);
			p1 = Math_Vec3AddVec3(p1, bb2->pos);
			p2 = Math_Vec3AddVec3(p2, bb2->pos);

			if(Math_IntersectLineTriangle(Math_Vec3AddVec3(bb->pos,(Vec3){0,4,0}),
				(Vec3){0,-1,0}, p0, p1, p2, &point)){
				bb->pos.y = point.y + (bb->wsCube.h/2) - 1;
				obj->ObjUpdate(obj);
			}
		}

	}

	//obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,Math_Vec3MultFloat(axis, -overlap));
}

static void Update(){

	World_Update(&game);
	int j;
	for(j = 0; j < NUM_MINIONS; j++){
		if(game.characters[j]->numUsers <= 1){
			game.characters[j]->RemoveUser(game.characters[j]);
			game.characters[j] = Minion_Create(&game,&game.models[MODEL_MINION-1]);
			game.characters[j]->AddUser(game.characters[j]);
			((Character *)game.characters[j]->data)->health += j * 0.5;
			((Character *)game.characters[j]->data)->index = j;
			((Character *)game.characters[j]->data)->lastTime = Window_GetTicks() + (j * 100);
		}
	}
	 game.player->bb.pos.y -= Window_GetDeltaTime() / 1000.0f;
	if(game.player->bb.pos.y < 0.2){
	    //game.player->bb.pos.y = 0.2;
	}

	static float animDir = 1;
	 playingAnims[0].into += animDir * Window_GetDeltaTime() / 40.1f;
			
	static float shapeKeyDir = 1;
	
	Model_SetShapeKey(&game.models[MODEL_PLAYER-1], 1,ui.sliderValue);
	game.player->model->materials[0].diffuse = (Vec4){ui.sliderValue2,1,1,1};;
		
	if(playingAnims[0].into > game.animations[ANIMATION_PLAYER-1].length || playingAnims[0].into < 0){
		animDir = - animDir;
	}

	Skeleton_Update(&playerSkel, playingAnims, 1);
	//Skeleton_Update(&hairSkel, hairAnims, 0);
	game.player->ObjUpdate(game.player);
	Object_UpdateSkeleton(game.player, &playerSkel);
	//Object_UpdateModel(game.world, &game.models[MODEL_WORLD-1]);
	
	Vec3 moveVec = {0,0,0};
	
	//if(movingDirs[0]) moveVec.z += 1;
	//if(movingDirs[1]) moveVec.z -= 1;
	//if(movingDirs[2]) moveVec.x += 1;
	//if(movingDirs[3]) moveVec.x -= 1;
	//if(movingDirs[4]) moveVec.y += 1;
	moveVec = Math_Vec3SubVec3(moveToPos,game.player->bb.pos);
	moveToPos.y = game.player->bb.pos.y;
	if(Math_Vec3Magnitude(moveVec)){

	     moveVec = Math_Vec3Normalize(moveVec);

		
		Vec3 xz = moveVec;
		xz.y = 0;
		if(Math_Vec3Magnitude(xz) > 0){
			xz = Math_Vec3Normalize(xz);
			Vec3 forward = Math_Rotate((Vec3){0,0,-1},game.player->bb.rot);
			float dot = Math_Vec3Dot(forward,xz);
			game.player->bb.rot.y -= dot * moveSpeed;
		}   
	     moveVec = Math_Vec3MultFloat(moveVec, Window_GetDeltaTime() * moveSpeed);

	    game.player->bb.pos.x += moveVec.x;
	    game.player->bb.pos.z += moveVec.z;
	    game.player->bb.pos.y += moveVec.y;
	}

	game.player->ObjUpdate(game.player);
	game.player->OnCollision = onCube;
	World_UpdateObjectInOctree(game.player);
	World_ResolveCollisions(&game,game.player, &game.player->bb);
	game.player->ObjUpdate(game.player);

	
	
	//float thrown = GetDeltaTime() / 500.0f;
	//Vec3 forward = Math_Rotate((Vec3){0,0,-1}, (Vec3){-rotation.y, -rotation.x, 0});
	//throwObj->bb.pos.x += thrown * forward.x;
	//throwObj->bb.pos.y += thrown * forward.y;
	//throwObj->bb.pos.z += thrown * forward.z;
	//if(Math_Vec3Magnitude(Math_Vec3SubVec3(throwObj->bb.pos,position)) > 5) 
	//throwObj->bb.pos = (Vec3){0,0,0};
	//throwObj->OnCollision = onThrow;
	//throwObj->ObjUpdate(throwObj);
	//World_ResolveCollisions(throwObj, &throwObj->bb);
	
	Pathfinding_ClearDynamic(&game.pf);
	
}

static void Event(SDL_Event ev){
	UI_Event(&ui, ev); 
	
	  //Thoth_Event(thoth, ev);
	if(ev.type == SDL_QUIT || ev.key.keysym.sym == SDLK_ESCAPE) exit(0);
		
	if(ev.type == SDL_MOUSEBUTTONDOWN){

		if(castingAOE){
			Spell_AOE_Cast(&game, game.player, castingPos);
			castingAOE = 0;
		}

		moveToPos = game.player->bb.pos;
		game.player->model->materials[0].diffuse = (Vec4){0.8,0.6,0.6,1};

		//throwObj->model->materials[0].diffuse = (Vec4){0.8,0.6,0.6,1};
		game.world->model->materials[0].diffuse = (Vec4){0.8,0.6,0.6,1};

		Vec4 rayWorld = (Vec4){
			(2.0 * (mousepos.x / WINDOW_WIDTH)) - 1.0, 
			1.0 - (2.0 * (mousepos.y / WINDOW_HEIGHT)), -1.0,1};

		rayWorld = Math_MatrixMult4(rayWorld, invPersp);
		rayWorld.z = -1;
		rayWorld.w = 0;
		rayWorld = Math_MatrixMult4(rayWorld, invView);
		Vec3 ray = (Vec3){rayWorld.x, rayWorld.y, rayWorld.z};
		
		ray = Math_Vec3Normalize(ray);
		float distance = HUGE_VAL;
		Object *collisionObj= NULL;
		BoundingBox *collision = NULL;
		game.ray = (Ray){renderpos,ray};
		game.ray.pos = (Vec3){invView[3],invView[7],invView[11]};;
		World_GetAllCollisionsRay(game.ray, &distance, &collision, &collisionObj, 
		COLLISIONFLAG_RAY_WORLD);
	
		if(collision && collisionObj){
			if(collisionObj == game.world){
				
				if(ev.button.button == SDL_BUTTON_LEFT){
				
					//Pathfinding_SetClosedGrid(&game.pf,
					//Math_Vec3AddVec3(renderpos,
					//Math_Vec3MultFloat(ray, distance)));

				} else if(ev.button.button == SDL_BUTTON_MIDDLE){
						Pathfinding_FindPathGrid(&game.pf,game.player->bb.pos,
					Math_Vec3AddVec3(renderpos,
					Math_Vec3MultFloat(ray, distance)), &pfpath);
					//Pathfinding_FindPath(&game.pf,game.player->bb.pos,
					//Math_Vec3AddVec3(renderpos,
					//Math_Vec3MultFloat(ray, distance)));
					onPath = 0;
				} else if(ev.button.button == SDL_BUTTON_RIGHT){ 
					
					//Pathfinding_SetOpenGrid(&game.pf,
					//Math_Vec3AddVec3(renderpos,
					//Math_Vec3MultFloat(ray, distance)));
				}
			}
		}
		World_GetAllCollisionsRay(game.ray, &distance, &collision, &collisionObj, 
		COLLISIONFLAG_RAY_OBJ);
		if(collision && collisionObj){
			if(collisionObj->type == TYPE_CHARACTER && collisionObj != game.player){
				Spell_AutoAttack_Cast(&game, game.player, collisionObj);

			}
		}
	} else if(ev.type == SDL_MOUSEMOTION){

		mousepos.x = ev.motion.x;
		mousepos.y = ev.motion.y;
		
		if(castingAOE){
			
			Vec4 rayWorld = (Vec4){
				(2.0 * (mousepos.x / WINDOW_WIDTH)) - 1.0, 
				1.0 - (2.0 * (mousepos.y / WINDOW_HEIGHT)), -1.0,1};

			rayWorld = Math_MatrixMult4(rayWorld, invPersp);
			rayWorld.z = -1;
			rayWorld.w = 0;
			rayWorld = Math_MatrixMult4(rayWorld, invView);
			Vec3 ray = (Vec3){rayWorld.x, rayWorld.y, rayWorld.z};
			
			ray = Math_Vec3Normalize(ray);
			float distance = HUGE_VAL;
			Object *collisionObj;
			BoundingBox *collision;
			game.ray = (Ray){renderpos,ray};
			game.ray.pos = (Vec3){invView[3],invView[7],invView[11]};;
			World_GetAllCollisionsRay(game.ray, &distance, &collision, &collisionObj,
			COLLISIONFLAG_RAY_WORLD);
			if(collision && collisionObj){
				
				castingPos = Math_Vec3AddVec3(
				Math_Vec3MultFloat(game.ray.line,distance), game.ray.pos);
			}
		}
		
	}
	if(ev.type == SDL_KEYDOWN){

	   if(ev.key.keysym.sym == SDLK_q)
			castingAOE = 1;;

	} else if(ev.type == SDL_KEYUP){

        if(ev.key.keysym.sym == SDLK_w)
	          movingDirs[0] = 0;
	      if(ev.key.keysym.sym == SDLK_q)
	          movingDirs[4] = 0;
	      else if(ev.key.keysym.sym == SDLK_s)
			movingDirs[1] = 0;
		else if(ev.key.keysym.sym == SDLK_a)
			movingDirs[2] = 0;
		else if(ev.key.keysym.sym == SDLK_d)
			movingDirs[3] = 0;

        int k;
	      for(k = 0; k < 4; k++) if(movingDirs[k] != 0) break;

	}
}

static void Focus(){

} 

static void DrawRigged(Game *game, Object *obj){

	Shaders_UseProgram(SKELETAL_ANIMATION_SHADER);

	Shaders_SetModelMatrix(obj->bb.matrix);
	Shaders_UpdateModelMatrix();

	if(obj == game->player){

		//glUniform4fv(Shaders_GetBonesLocation(), hairSkel.nBones * 3, &hairSkel.matrices[0].x);
		//glActiveTexture(GL_TEXTURE0);
		//glBindVertexArray(hairModel.vao);

		//int curr = 0;


		//int k;
		//for(k = 0; k < hairModel.nMaterials; k++){

			//glBindTexture(GL_TEXTURE_2D, hairModel.materials[k].texture);
			//glUniform4fv(Shaders_GetDiffuseLocation(), 1, (float *)&hairModel.materials[k].diffuse);
			//glUniform4fv(Shaders_GetSpecularLocation(), 1, (float *)&hairModel.materials[k].specular);
			//glDrawElements(GL_TRIANGLES, hairModel.nElements[k], GL_UNSIGNED_INT, (void *)(curr * sizeof(GLuint)));
			//curr += hairModel.nElements[k];
		//}

		//glBindVertexArray(0);
	}

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

static void DrawModel(Game *game, Object *obj){

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
		glUniform4fv(Shaders_GetDiffuseLocation(), 1, (float *)&obj->model->materials[k].diffuse);
		glUniform4fv(Shaders_GetSpecularLocation(), 1, (float *)&obj->model->materials[k].specular);
		glDrawElements(GL_TRIANGLES, obj->model->nElements[k], GL_UNSIGNED_INT, (void *)(curr * sizeof(GLuint)));
		curr += obj->model->nElements[k];
	}

	glBindVertexArray(0);
}
static char Draw(){
	float persp[16];
	glBindFramebuffer(GL_FRAMEBUFFER,0);
	//rotation.y += mouseSensitivity * Window_GetDeltaTime();
	
	renderpos = Math_Vec3AddVec3(game.player->bb.pos,Math_Rotate(position,rotation));
	Vec3 forward = Math_Rotate(Math_Vec3Normalize(Math_Vec3SubVec3(renderpos, game.player->bb.pos)),rotation);
		
	Math_LookAt(view, renderpos, game.player->bb.pos, (Vec3){0,1,0});
	Shaders_SetViewMatrix(view);
	memcpy(invView,view,sizeof(invView));
	Math_InverseMatrix(invView);
	 Math_Perspective(persp, 60.0f*(3.1415/180), (float)1920 / (float)1080, 0.1f, 50.0f);
	memcpy(invPersp,persp,sizeof(invPersp));	
	Math_InverseMatrix(invPersp);
	Shaders_SetProjectionMatrix(persp);
	Shaders_UseProgram(SKELETAL_ANIMATION_SHADER);
	Shaders_UpdateViewMatrix();
	Shaders_UpdateProjectionMatrix();
	Shaders_UseProgram(PARTICLE_SHADER);
	Shaders_UpdateViewMatrix();
	Shaders_UpdateProjectionMatrix();
	Shaders_UseProgram(TEXTURED_SHADER);
	Shaders_UpdateViewMatrix();
	Shaders_UpdateProjectionMatrix();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glCullFace(GL_BACK);
	float idenity[16];
	Shaders_UseProgram(TEXTURED_SHADER);

	Math_Identity(idenity);
	Shaders_SetModelMatrix(idenity);

	Skybox_Draw(&skybox);


	int k;
	for(k = 0; k < game.player->skelBb.numChildren; k++){
		//World_DrawSkeleton(&game.player->skelBb.children[k]);
	}


	//ConeConstraint_Create(&figure.constraints[0],&figure.skel->bones[14],
	//&figure.skel->bones[15],(Vec3){0,1,0},1);

	Shaders_UseProgram(TEXTURELESS_SHADER);
	Shaders_SetModelMatrix(game.player->bb.matrix);
	//Physics_ApplyForces(&figure);

	if(pfpath.nPath > 0 && onPath < pfpath.nPath-1 && Math_Vec3Magnitude(Math_Vec3SubVec3(moveToPos,game.player->bb.pos)) < 0.1){
		moveToPos = pfpath.path[onPath];
		onPath++; 
		if(onPath >= pfpath.nPath) onPath = 0;
		if(onPath < pfpath.nPath)
			moveToPos = pfpath.path[onPath];
	}
	


	Skeleton_Apply(&playerSkel);

	World_Render(&game,1);

	if(castingAOE){
		Math_Identity(idenity);
		Shaders_UseProgram(TEXTURELESS_SHADER);
		Shaders_SetModelMatrix(idenity);
		Shaders_UpdateModelMatrix();
		Cube cube = (Cube){castingPos.x - 1.5,castingPos.y,castingPos.z - 1.5,3,0.1,3};
		World_DrawX(cube);
	}
	
	UI_Clear(&ui);

	Image img;
	img.glTexture = game.player->model->materials[0].texture;
	img.w = game.player->model->materials[0].w;
	img.h = game.player->model->materials[0].h;
	img.nFramesX = img.nFramesY = 1;


	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);

	//Thoth_RenderIntoTexture(thoth, &img.glTexture, &img.w, &img.h);

	//UI_RenderRectTex(&ui, img, 0,0, img.w/2,img.h, 
	//0,0, img.w/2,img.h, 255,255,255,255);

	Pathfinding_RenderDebug(&game.pf,&pfpath);

	UI_Render(&ui, &game);

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	

	return 1;
}

static void OnResize(){
	//Thoth_Render(thoth); 

}

void PlayerDamage(Game *game, Object *this, Object *cause){
	
}

int main(int argc, char **argv){
	
	Window_Open("Editor", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,WINDOW_WIDTH, WINDOW_HEIGHT, 0);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	Memory_Init((0x01 << 20) * 64);
	
	
	glClearColor(0,0,0,1);
	Shaders_Init();

	Text_Init();
	UI_Init(&ui, WINDOW_WIDTH, WINDOW_HEIGHT);

	Particles_Init(&game.particleSystem);
	

	particleImg = ImageLoader_CreateImage("Resources/smoke.png",1);
	particleImg.nFramesX = 5;
	particleImg.nFramesY = 5;
	billboardImg = ImageLoader_CreateImage("Resources/tex.png",1);	
	game.images[IMAGE_HEALTH-1] = ImageLoader_CreateImage("Resources/health.png",1);	

	World_InitOctree((Vec3){-100, -100, -100}, 200, 25);

	glClearColor(0,0,0,1);

	skybox = Skybox_Create(30, (Vec3){0,0,0}, "Resources/skybox.png");

	Math_Perspective(persp, 60.0f*(3.1415/180), (float)1920 / (float)1080, 0.1f, 100.0f);
	memcpy(invPersp, persp, sizeof(persp));
	Math_InverseMatrix(invPersp);

	//Math_LookAt(view, (Vec3){0,0,-5}, (Vec3){0,0,0}, (Vec3){0,1,0});
	Math_Identity(model);
	Shaders_UseProgram(SKELETAL_ANIMATION_SHADER);
	Shaders_SetProjectionMatrix(persp);
	Shaders_UpdateProjectionMatrix();
	Shaders_UpdateModelMatrix();
	Shaders_SetViewMatrix(view);
	Shaders_UpdateViewMatrix();
	Shaders_UpdateProjectionMatrix();


	game.player = Object_Create();
	game.player->skeleton = &playerSkel;
	game.player->type = TYPE_CHARACTER;
	game.player->data = malloc(sizeof(Character));
	

	((Character*)game.player->data)->type = CHARACTER_TYPE_PLAYER;
	((Character*)game.player->data)->Damage = PlayerDamage;
	memcpy(game.player->matrix, Math_Identity, sizeof(Math_Identity));
	RiggedModel_Load(&game.models[MODEL_PLAYER-1], "Resources/figure.yuk");
	Skeleton_Copy(&playerSkel, &game.models[MODEL_PLAYER-1].skeleton);
	Animation_Load(&game.animations[ANIMATION_PLAYER-1], "Resources/figure_ArmatureAction.anm");
	Animation_Load(&game.animations[ANIMATION_MINION-1], "Resources/minion_ArmatureAction.001.anm");
	game.images[IMAGE_GOLD-1] = ImageLoader_CreateImage("Resources/gold.png",1);
	game.images[IMAGE_PARTICLES-1] = ImageLoader_CreateImage("Resources/smoke.png",1);;
	Object_SetModel(game.player, &game.models[MODEL_PLAYER-1]);
	game.player->bb.collisionFlag |= COLLISIONFLAG_SAT;
	
	game.player->Draw = DrawRigged;
	game.player->AddUser(game.player);
	game.player->bb.pos = moveToPos = pfpath.path[onPath];
	game.player->bb.pos.y =1;
	game.player->bb.scale = (Vec3){0.2,0.2,0.2};
	game.player->bb.rot = (Vec3){0,0,0};
	game.player->bb.renderDebug = 0;
	game.player->bb.cube = (Cube){-2.5,0.1,-2.5,5,10,5};
	World_UpdateObjectInOctree(game.player);
	playingAnims[0] = (PlayingAnimation){
		 .active = 1,
		 .weight = 1,
		 .into = 0,
		 .anim = &game.animations[ANIMATION_PLAYER-1],
	};

	game.world = Object_Create();
	Model_Load(&game.models[MODEL_WORLD-1], "Resources/room.yuk");
	Model_LoadCollisions(&game.models[MODEL_WORLD-1], "Resources/room.col");
	

	Object_SetModel(game.world, &game.models[MODEL_WORLD-1]);
	game.world->Draw = DrawModel;
	game.world->AddUser(game.world);
	
	game.world->bb.rot = (Vec3){0,0,0};
	moveToPos = game.player->bb.pos;
	game.world->ObjUpdate(game.world);
	
	World_UpdateObjectInOctree(game.world);
	
	Pathfinding_Init(&game.pf, 200,200);
	game.pf.cube.x = game.world->bb.wsCube.x;
	game.pf.cube.z = game.world->bb.wsCube.z;
	int k; // skip the ground
	for(k = 1; k < game.world->bb.numChildren; k++ ){
		game.world->bb.children[k].collisionFlag |= COLLISIONFLAG_RAY_WORLD;
		Pathfinding_SetClosedBoundingBoxStatic(&game.pf, &game.world->bb.children[k]);
	}

	game.world->bb.children[0].rot = (Vec3){0,1,0};;
	game.world->bb.children[0].collisionFlag = COLLISIONFLAG_AABB;
	game.world->bb.collisionFlag |= COLLISIONFLAG_AABB;
	game.world->bb.children[0].collisionFlag |= COLLISIONFLAG_RAY_WORLD;
	game.world->bb.children[0].collisionFlag |= COLLISIONFLAG_POLYSOUP;
	game.world->bb.collisionFlag |= COLLISIONFLAG_RAY_WORLD ;
	game.world->bb.collisionFlag |= COLLISIONFLAG_POLYSOUP ;
	
	BoundingBox_LoadSoup(&game.world->bb.children[0], "Resources/roomsoup.yuk");
	
	RiggedModel_Load(&game.models[MODEL_MINION-1], "Resources/minion.yuk");

	int j;
	for(j = 0; j < NUM_MINIONS; j++){
		game.characters[j] = Minion_Create(&game,&game.models[MODEL_MINION-1]);
		//((Character *)game.characters[j]->data)->health += j * 0.5;
		game.characters[j]->AddUser(game.characters[j]);

		((Character *)game.characters[j]->data)->index = j;
		((Character *)game.characters[j]->data)->lastTime = Window_GetTicks() + (j * 100);
	}

	Window_MainLoop(Update, Event, Draw, Focus, OnResize, 1, 1);
	 //Thoth_Destroy(thoth);

	World_Free();
	Shaders_Close();
	ImageLoader_Free();
	UI_Free(&ui);
	Text_Close();
	Skybox_Free(&skybox);
}