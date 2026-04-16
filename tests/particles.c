#include <GL/glew.h>
#include "particles.h"
#include "shaders.h"
#include "window.h"
#include "memory.h"
#include "shaders.h"


static int SortParticles(const void *p1, const void *p2){

	Particle *particle1 = (Particle *)p1;
	Particle *particle2 = (Particle *)p2;

	if(particle1->distFromEye < particle2->distFromEye) return 1;
	if(particle1->distFromEye > particle2->distFromEye) return -1;
	return 0;
} 

void Particles_DrawParticles(Image img, ParticleSystem *ps,Particle *particles, int nParticles, 
	float animSpeed, Vec3 camForward, Vec3 camPos, int depthTexture){

	nParticles = MIN(MAX_PARTICLES, nParticles);

	Shaders_UseProgram(PARTICLE_SHADER);

	glBindVertexArray(ps->vao);

	float currTime = Window_GetTicks();

	int num = 0;

	int k;
	for(k = 0; k < nParticles; k++){

		if(particles[k].createTime <= 0 || currTime - particles[k].createTime > particles[k].lifeTime){
			particles[k].distFromEye = -HUGE_VAL;
			continue;
		}

		// particles[k].distFromEye = Math_Vec3Magnitude(Math_Vec3SubVec3(particles[k].pos, camPos));
		particles[k].distFromEye = -Math_Vec3Dot(camForward, Math_Vec3SubVec3(particles[k].pos, camPos));
		++num;
	}

	qsort(particles, nParticles, sizeof(Particle), SortParticles);

	for(k = 0; k < nParticles; k++){

		if(particles[k].distFromEye < 0)
			continue;

		Vec3 pos = particles[k].pos;
		Vec4 color = particles[k].color;
		Vec2 size = particles[k].size;

		Vec3 centers[] = { pos, pos, pos, pos };
		Vec4 colors[] = { color, color, color, color };

		Vec2 positions[] = {
			{ -size.x/2, -size.y/2 },
			{ size.x/2, -size.y/2 },
			{ size.x/2, size.y/2 },
			{ -size.x/2, size.y/2 },
		};

		int onFrame = (currTime - particles[k].createTime) / animSpeed;

		onFrame %= img.nFramesX * img.nFramesY;

		float tsx = 1.0 / img.nFramesX;
		float tsy = 1.0 / img.nFramesX;

		float tx = (onFrame % img.nFramesX) * tsx;
		float ty = (onFrame / img.nFramesX) * tsy;

		Vec2 uvs[] = {
			{ tx, ty },
			{ tx + tsx, ty },
			{ tx + tsx, ty + tsy },
			{ tx, ty + tsy },
		};

		glBindBuffer(GL_ARRAY_BUFFER, ps->centerVbo);
		glBufferSubData(GL_ARRAY_BUFFER, k * sizeof(Vec3) * 4, sizeof(Vec3) * 4, centers);

		glBindBuffer(GL_ARRAY_BUFFER, ps->colorVbo);
		glBufferSubData(GL_ARRAY_BUFFER, k * sizeof(Vec4) * 4, sizeof(Vec4) * 4, colors);

		glBindBuffer(GL_ARRAY_BUFFER, ps->positionVbo);
		glBufferSubData(GL_ARRAY_BUFFER, k * sizeof(Vec2) * 4, sizeof(Vec2) * 4, positions);

		glBindBuffer(GL_ARRAY_BUFFER, ps->uvVbo);
		glBufferSubData(GL_ARRAY_BUFFER, k * sizeof(Vec2) * 4, sizeof(Vec2) * 4, uvs);
	}

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, img.glTexture);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	
	glDrawElements(GL_TRIANGLES, num * 6, GL_UNSIGNED_SHORT, NULL);

	glBindVertexArray(0);
}

void Particles_DrawBillboard( Image img,ParticleSystem *ps, Vec3 pos, Vec2 size, Vec4 color, Rect2D imgRect){

	Shaders_UseProgram(PARTICLE_SHADER);

	glBindVertexArray(ps->vao);

	Vec3 centers[] = { pos, pos, pos, pos };
	Vec4 colors[] = { color, color, color, color };

	Vec2 positions[] = {
		{ -size.x/2, -size.y/2 },
		{ size.x/2, -size.y/2 },
		{ size.x/2, size.y/2 },
		{ -size.x/2, size.y/2 },
	};

	Vec2 uvs[] = {
		{ imgRect.x, imgRect.y },
		{ imgRect.x + imgRect.w, imgRect.y },
		{ imgRect.x + imgRect.w, imgRect.y + imgRect.h },
		{ imgRect.x, imgRect.y + imgRect.h },
	};

	glBindBuffer(GL_ARRAY_BUFFER, ps->positionVbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * 4, positions);

	glBindBuffer(GL_ARRAY_BUFFER, ps->centerVbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec3) * 4, centers);

	glBindBuffer(GL_ARRAY_BUFFER, ps->colorVbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4) * 4, colors);

	glBindBuffer(GL_ARRAY_BUFFER, ps->uvVbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * 4, uvs);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, img.glTexture);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, NULL);

	glBindVertexArray(0);
}

void Particles_Init(ParticleSystem *ps){
	
	GLuint centerLoc = glGetAttribLocation(Shaders_GetProgram(PARTICLE_SHADER), SHADERS_CENTER_ATTRIB);
	GLuint colorLoc = glGetAttribLocation(Shaders_GetProgram(PARTICLE_SHADER), SHADERS_COLOR_ATTRIB);
	GLuint uvLoc = glGetAttribLocation(Shaders_GetProgram(PARTICLE_SHADER), SHADERS_COORD_ATTRIB);
	GLuint posLoc = glGetAttribLocation(Shaders_GetProgram(PARTICLE_SHADER), SHADERS_POSITION_ATTRIB);
					
	glGenVertexArrays(1, &ps->vao);
	glBindVertexArray(ps->vao);

	glGenBuffers(1, &ps->centerVbo);
	glBindBuffer(GL_ARRAY_BUFFER, ps->centerVbo);

    glEnableVertexAttribArray(centerLoc);
	 glVertexAttribPointer(centerLoc, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * MAX_PARTICLES * 4, NULL, GL_STATIC_DRAW);

	glGenBuffers(1, &ps->uvVbo);
	glBindBuffer(GL_ARRAY_BUFFER, ps->uvVbo);

    glEnableVertexAttribArray(uvLoc);
	 glVertexAttribPointer(uvLoc, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * MAX_PARTICLES * 4, NULL, GL_STATIC_DRAW);

	glGenBuffers(1, &ps->colorVbo);
	glBindBuffer(GL_ARRAY_BUFFER, ps->colorVbo);

    glEnableVertexAttribArray(colorLoc);
	 glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);

	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * MAX_PARTICLES * 4, NULL, GL_STATIC_DRAW);

	glGenBuffers(1, &ps->positionVbo);
	glBindBuffer(GL_ARRAY_BUFFER, ps->positionVbo);
	
	 glEnableVertexAttribArray(posLoc);
	 glVertexAttribPointer(posLoc, 2, GL_FLOAT, GL_FALSE, 0, 0);
	
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * MAX_PARTICLES * 4, NULL, GL_STATIC_DRAW);

	glGenBuffers(1, &ps->ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ps->ebo);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(u16) * MAX_PARTICLES * 6, NULL, GL_STATIC_DRAW);

	u16 elements[] = {0, 1, 2, 0, 2, 3};

	int k;
	for(k = 0; k < MAX_PARTICLES; k++){

		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, k * sizeof(u16) * 6, sizeof(u16) * 6, elements);

		int j;
		for(j = 0; j < 6; j++)
			elements[j] += 4;

	}

	glBindVertexArray(0);
}

void Particles_Close(ParticleSystem *ps){

	glDeleteBuffers(1, &ps->positionVbo);
	glDeleteBuffers(1, &ps->centerVbo);
	glDeleteBuffers(1, &ps->uvVbo);
	glDeleteBuffers(1, &ps->colorVbo);
	glDeleteBuffers(1, &ps->ebo);
	glDeleteVertexArrays(1, &ps->vao);
}