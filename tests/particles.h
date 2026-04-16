#ifndef PARTICLES_DEF
#define PARTICLES_DEF


#include "types.h"
#include "math.h"
#include "image_loader.h"

#define MAX_PARTICLES 1024

typedef struct {
	float 	createTime;
	float 	lifeTime;
	float  	distFromEye;
	Vec3 	pos;
	Vec2 	size;
	Vec4 	color;
	Vec3 	vel;
} Particle;

typedef struct {
	u32 		vao;
	u32 		centerVbo;
	u32 		positionVbo;
	u32 		uvVbo;
	u32 		colorVbo;
	u32 		ebo;
} ParticleSystem;

void Particles_DrawBillboard(Image img, ParticleSystem *ps, Vec3 pos, Vec2 size, Vec4 color, Rect2D imgRect);
void Particles_DrawParticles(Image img, ParticleSystem *ps, Particle *particles, int nParticles, float animSpeed, Vec3 camForward, Vec3 camPos, int depthTexture);
void Particles_Init(ParticleSystem *ps);
void Particles_Close(ParticleSystem *ps);

#endif