#include <GL/glew.h>
#include "window.h"
#include "pathfinding.h"
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

#define NUM_GARBAGE 10


//static Thoth_t *thoth;
static ParticleSystem ps;
static Particle particles[100];
static Image particleImg, billboardImg;
static Object *rotatingObj = NULL;
static Object *cubeObj, *groundObj, *throwObj, *cans[NUM_GARBAGE];
static Model cubeModel, groundModel, throwModel, canModel;
static Animation cubeAnim;
static Skeleton cubeSkel;
static PlayingAnimation cubeAnims[1];
static Skybox skybox;
static float mouseSensitivity = 0.0005;
static float moveSpeed = 0.005;
float invPersp[16];
float persp[16], view[16], model[16];
static Vec2 rotation = {0,0};
static Vec2 mousepos = {0,0};
static Vec3 position = {-2,7,6};
static Vec3 renderpos = {-2,4,4};
static UI ui;
static char movingDirs[5];
static Vec3 moveToPos;

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

static void onCube(Object *obj, Object *obj2, BoundingBox *bb, BoundingBox *bb2, Vec3 axis, float overlap){
	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,Math_Vec3MultFloat(axis, -overlap));
	
	obj->ObjUpdate(obj);
}

static void Update(){

	 cubeObj->bb.pos.y -= Window_GetDeltaTime() / 1000.0f;
	if(cubeObj->bb.pos.y < 0.2){
	    cubeObj->bb.pos.y = 0.2;
		moveToPos.y = cubeObj->bb.pos.y;
	}

	static float animDir = 1;
	 cubeAnims[0].into += animDir * Window_GetDeltaTime() / 100.1f;

    if(cubeAnims[0].into > cubeAnim.length || cubeAnims[0].into < 0){
		animDir = - animDir;
	}

	Skeleton_Update(&cubeSkel, cubeAnims, 1);
	cubeObj->ObjUpdate(cubeObj);
	Object_UpdateSkeleton(cubeObj, &cubeSkel);
	Object_UpdateModel(groundObj, &groundModel);
	
	Vec3 moveVec = {0,0,0};
	moveVec = Math_Vec3SubVec3(moveToPos,cubeObj->bb.pos);

		if(Math_Vec3Magnitude(moveVec)){

	     moveVec = Math_Vec3Normalize(moveVec);

	     moveVec = Math_Vec3MultFloat(moveVec, Window_GetDeltaTime() * moveSpeed);

	      //position.x += moveVec.x;
						//position.z += moveVec.z;
	    cubeObj->bb.pos.x += moveVec.x;
	    cubeObj->bb.pos.z += moveVec.z;
	    cubeObj->bb.pos.y += moveVec.y;
	}

	cubeObj->ObjUpdate(cubeObj);
	cubeObj->OnCollision = onCube;
	World_UpdateObjectInOctree(cubeObj);
	World_ResolveCollisions(cubeObj, &cubeObj->bb);
	cubeObj->ObjUpdate(cubeObj);

	

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
}

static void Event(SDL_Event ev){
	      //Thoth_Event(thoth, ev);
	      //if(ev.window.event == SDL_WINDOWEVENT_RESIZED || 
	          //ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED){
	          //int w = ev.window.data1;
	          //int h = ev.window.data2;

	          //Thoth_Resize(thoth, 0, 0, w, h);
	      //}
	if(ev.type == SDL_MOUSEBUTTONUP){

		if(ev.button.button == SDL_BUTTON_LEFT){
			rotatingObj = NULL;
			moveToPos = cubeObj->bb.pos;
			throwObj->model->materials[0].diffuse = (Vec4){0.8,0.6,0.6,1};
			cubeObj->model->materials[0].diffuse = (Vec4){0.8,0.6,0.6,1};
			groundObj->model->materials[0].diffuse = (Vec4){0.8,0.6,0.6,1};

			float invView[16];
			Shaders_GetInvViewMatrix(invView);
			
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
			Object *collisionObj = NULL;
			BoundingBox *collision = NULL;
	
			World_GetAllCollisionsRay((Ray){renderpos, ray}, &distance, &collision, &collisionObj);
			if(collisionObj){
				collisionObj->model->materials[0].diffuse = (Vec4){1,1,1,1};

				if(collisionObj->type == TYPE_CAN){
					
					ui.stress += 0.1;
					
					World_RemoveObjectFromOctree(collisionObj);
					
					int j;
					for(j = 0; j < 100; j++){
						particles[j].createTime = SDL_GetTicks();
						particles[j].lifeTime = 10000;
						particles[j].pos = Math_Vec3AddVec3(collisionObj->bb.pos,
						 (Vec3){ (-50 + rand()%100)/90.0f,(-50 + rand()%100)/90.0f,(-50 + rand()%100)/90.0f});
						particles[j].size = (Vec2){1,1};
						particles[j].color = (Vec4){0.1,0.1,0.1,0.1};
						particles[j].vel = (Vec3){(-5 + (rand()%10))/10000.0f,(-5 + (rand()%10))/10000.0f,
						(-5 + (rand()%10))/10000.0f};
					}
				} else {
					if(collisionObj == groundObj){
						moveToPos = Math_Vec3AddVec3(renderpos,
						Math_Vec3MultFloat(ray, distance));
					} else {
						
						rotatingObj = collisionObj;
					}
				}
			}
		}

	} else if(ev.type == SDL_MOUSEMOTION){

		mousepos.x = ev.motion.x;
		mousepos.y = ev.motion.y;

	} else if(ev.type == SDL_KEYDOWN){

        if(ev.key.keysym.sym == SDLK_w)
	          movingDirs[0] = 1;
	      if(ev.key.keysym.sym == SDLK_q)
	          movingDirs[4] = 1;
		else if(ev.key.keysym.sym == SDLK_s)
			movingDirs[1] = 1;
		else if(ev.key.keysym.sym == SDLK_a)
			movingDirs[2] = 1;
	      else if(ev.key.keysym.sym == SDLK_d)
	          movingDirs[3] = 1;
	      //else if(ev.key.keysym.sym == SDLK_jESCAPE)
	          //exit(0);

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

static void DrawRigged(Object *obj){

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
static void DrawModel(Object *obj){

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
	rotation.y += mouseSensitivity * Window_GetDeltaTime();
	
	renderpos = Math_Rotate(position, (Vec3){0,rotation.y,0});
	Vec3 forward = Math_Vec3Normalize(Math_Vec3SubVec3((Vec3){0,0,0}, renderpos));

	float view[16];
	Math_LookAt(view, renderpos, forward, (Vec3){0,1,0});
	Shaders_SetViewMatrix(view);
	 Math_Perspective(persp, 60.0f*(3.1415/180), (float)1920 / (float)1080, 0.1f, 50.0f);
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

	int k;
	for(k = 0; k < cubeObj->skelBb.numChildren; k++){
		World_DrawSkeleton(&cubeObj->skelBb.children[k]);
	}

	Skeleton_Update(&cubeSkel, cubeAnims, 1);
	Skeleton_Apply(&cubeSkel);

	Skybox_Draw(&skybox);
	int j;
	for(j = 0; j < NUM_GARBAGE; j++){
		cans[j]->bb.renderDebug = 1;
	}
	Shaders_UseProgram(TEXTURELESS_SHADER);
	Shaders_SetModelMatrix(cubeObj->bb.matrix);


	World_Render(1);
	Particles_DrawParticles(particleImg, &ps, particles, 100, 50, forward, renderpos, 0);
	
	UI_Clear(&ui);
	UI_Render(&ui);
	return 1;
}

static void OnResize(){
	//Thoth_Render(thoth); 

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

	Particles_Init(&ps);
	
	//thoth = Thoth_Create(WINDOW_WIDTH, WINDOW_HEIGHT );
	//Thoth_Resize(thoth, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	//Thoth_LoadFile(thoth, "main.c");


	particleImg = ImageLoader_CreateImage("Resources/smoke.png",1);
	particleImg.nFramesX = 5;
	particleImg.nFramesY = 5;
	billboardImg = ImageLoader_CreateImage("Resources/tex.png",1);	

	Pathfinder pf;
	Pathfinding_Init(&pf, 10,10);
	Pathfinding_SetClosed(&pf, 1,1);	
	Pathfinding_SetClosed(&pf, 1,2);	
	Pathfinding_SetClosed(&pf, 1,3);
	Pathfinding_FindPath(&pf, 0,0,3,3);

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
	(model);
	Shaders_UpdateModelMatrix();
	Shaders_SetViewMatrix(view);
	Shaders_UpdateViewMatrix();
	Shaders_UpdateProjectionMatrix();
	memset(&cubeSkel, 0, sizeof(Skeleton));

	cubeObj = Object_Create();
	cubeObj->skeleton = &cubeSkel;
	memcpy(cubeObj->matrix, Math_Identity, sizeof(Math_Identity));
	RiggedModel_Load(&cubeModel, &cubeSkel, "Resources/figure.yuk");
	memset(&cubeAnim, 0, sizeof(Animation));
	Animation_Load(&cubeAnim, "Resources/figure_ArmatureAction.anm");
	

	Object_SetModel(cubeObj, &cubeModel);
	cubeObj->Draw = DrawRigged;
	cubeObj->AddUser(cubeObj);
	cubeObj->bb.pos.y = 1;
	cubeObj->bb.scale = (Vec3){0.2,0.2,0.2};
	cubeObj->bb.rot = (Vec3){0,0,0};
	cubeObj->bb.cube = (Cube){-2.5,0.1,-2.5,5,10,5};
	World_UpdateObjectInOctree(cubeObj);

	groundObj = Object_Create();
	Model_Load(&groundModel, "Resources/room.yuk");
	Model_LoadCollisions(&groundModel, "Resources/room.col");
	Object_SetModel(groundObj, &groundModel);
	groundObj->Draw = DrawModel;
	groundObj->AddUser(groundObj);
	groundObj->bb.rot = (Vec3){0,1,0};
	moveToPos = cubeObj->bb.pos;
	groundObj->ObjUpdate(groundObj);
	World_UpdateObjectInOctree(groundObj);
	
	throwObj = Object_Create();
	Model_Load(&throwModel, "Resources/cube.yuk");
	Object_SetModel(throwObj, &throwModel);
	throwObj->Draw = DrawModel;
	throwObj->bb.pos = (Vec3){0,1,0};
	throwObj->bb.scale = (Vec3){0.5,0.5,0.5};
	throwObj->AddUser(throwObj);
	throwObj->ObjUpdate(throwObj);
	World_UpdateObjectInOctree(throwObj);
	Model_Load(&canModel, "Resources/can.yuk");

	int j;
	for(j = 0; j < NUM_GARBAGE; j++){
			cans[j] = Object_Create();
			Object_SetModel(cans[j], &canModel);
			cans[j]->Draw = DrawModel;
			cans[j]->bb.renderDebug = 1;
			cans[j]->bb.pos = Math_Vec3AddVec3(cans[j]->bb.pos,
			(Vec3){ (-50 + rand()%100)/20.0f,0.1, (-50 + rand()%100)/20.0f});
			cans[j]->bb.scale = (Vec3){0.2,0.2,0.2};
			cans[j]->type = TYPE_CAN;
			cans[j]->ObjUpdate(cans[j]);
			World_UpdateObjectInOctree(cans[j]);
	}

	cubeAnims[0] = (PlayingAnimation){
		 .active = 1,
		 .weight = 1,
		 .into = 0,
		 .anim = &cubeAnim,
	};

	// thoth = Thoth_Create(WINDOW_WIDTH, WINDOW_HEIGHT );
	// Thoth_LoadFile(thoth, "main.c");
	Window_MainLoop(Update, Event, Draw, Focus, OnResize, 1, 1);
	

    //Thoth_Destroy(thoth);

	World_Free();
	Shaders_Close();
	ImageLoader_Free();
	UI_Free(&ui);
	Text_Close();
	Skybox_Free(&skybox);
}