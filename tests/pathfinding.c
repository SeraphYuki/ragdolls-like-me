#include "pathfinding.h"
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include "shaders.h"
#include "memory.h"
#include "game.h"
#include <GL/glew.h>

void Pathfinding_Init(Pathfinder *pf, int w, int h){
	memset(pf, 0, sizeof(Pathfinder));
	pf->w = w;
	pf->h = h;
}

void Pathfinding_SetClosed(Pathfinder *pf, int x, int y){
	int index = x + (y * pf->w);
	AStarNode node;
	node.index = index;
	pf->closed[pf->nClosed] = node;
	pf->nClosed++;
}

void Pathfinding_LoadNavMesh(Pathfinder *pf, const char *path){


	int k;
	 u16 stride = sizeof(Vec3);

    glGenVertexArrays(1, &pf->vao);
	 glBindVertexArray(pf->vao);

    glGenBuffers(1, &pf->ebo);
	 glGenBuffers(1, &pf->vbo);
	 glBindBuffer(GL_ARRAY_BUFFER, pf->vbo);

    GLuint positionAttribute = glGetAttribLocation(Shaders_GetProgram(TEXTURED_SHADER), SHADERS_POSITION_ATTRIB);
    glEnableVertexAttribArray(positionAttribute);
    glVertexAttribPointer(positionAttribute, 3, GL_FLOAT, GL_FALSE, stride, 0);

    void *offset = (void *)sizeof(Vec3);

	 FILE *fp = fopen(path, "rb");


    int nVerts = 0;
	 fread(&nVerts, 1, sizeof(int), fp);

    int size = stride * nVerts;

    u8 *vboData = (u8 *)Memory_StackAlloc(TEMP_STACK, size);

    // Deflate_Read(fp, vboData, size);
	 
	 fread(vboData, 1, size, fp);

	pf->cube.x = pf->cube.y = pf->cube.z = HUGE_VAL;
	pf->cube.w = pf->cube.h = pf->cube.d = -HUGE_VAL;

    for(k = 0; k < nVerts; k++){
	     Vec3 *pos = (Vec3 *)&vboData[(stride * k)];
	     if(pos->x < pf->cube.x)
	         pf->cube.x = pos->x;
	     if(pos->x > pf->cube.w)
	         pf->cube.w = pos->x;
	     if(pos->y < pf->cube.y)
	         pf->cube.y = pos->y;
	     if(pos->y > pf->cube.h)
	         pf->cube.h = pos->y;
	     if(pos->z < pf->cube.z)
	         pf->cube.z = pos->z;
	     if(pos->z >  pf->cube.d)
	         pf->cube.d = pos->y;
	 }
	 pf->cube.w -= pf->cube.x;
	 pf->cube.h -= pf->cube.y;
	 pf->cube.d -= pf->cube.z;

    glBindBuffer(GL_ARRAY_BUFFER, pf->vbo);
	 glBufferData(GL_ARRAY_BUFFER, size, vboData, GL_STATIC_DRAW);

    Memory_StackPop(TEMP_STACK, 1);

    fread(&pf->nElements, 1, sizeof(int), fp);

    u32 *elements = (u32 *)Memory_StackAlloc(TEMP_STACK, sizeof(u32) * pf->nElements);

    fread(elements, pf->nElements, sizeof(u32), fp);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pf->ebo);
	 glBufferData(GL_ELEMENT_ARRAY_BUFFER, pf->nElements * sizeof(u32), elements, GL_STATIC_DRAW);


	 Memory_StackPop(TEMP_STACK, 1);
    fclose(fp);

    glBindVertexArray(0);
	
}

void Pathfinding_RenderDebug(Pathfinder *pf){

	Shaders_UseProgram(TEXTURED_SHADER);
	
	float matrix[16];
	Math_TranslateMatrix(matrix,pf->pos);
	Shaders_SetModelMatrix(matrix);
	Shaders_UpdateModelMatrix();

	glActiveTexture(GL_TEXTURE0);

	glBindVertexArray(pf->vao);
	glCullFace(GL_BACK);
	int curr = 0;




	glDrawElements(GL_LINE_STRIP, pf->nElements, GL_UNSIGNED_INT, 0);
	curr += pf->nElements;



	glBindVertexArray(0);

}

int Pathfinding_FindPath(Pathfinder *pf, int x, int y, int gx, int gy){

	int h(int){ return 1; } // magnitude 
	int reconstructpath(int*,int){
		printf("goal\n");
	
	}
			
	AStarNode *curr = &pf->open[0];
	curr->index = x + (y * pf->w); 
	curr->f = INT_MAX;
	curr->g = 0;
	pf->nOpen++;
	
	int goal = gx + (gy * pf->w);	
	while(pf->nOpen > 0){
		
		curr = &pf->open[0];
		int m;
		for(m = 0; m < pf->nOpen; m++){
			if( pf->open[m].f < curr->f){
				curr = &pf->open[m];
			}	
		}
		
		if(curr->index == goal){ printf("goal\n"); return 0; }

		pf->nOpen--;
		
		pf->closed[pf->nClosed++] = *curr;
		
		
		int neighbors[] = {
			curr->index > 0 			? curr->index - 1 : curr->index,
			curr->index < pf->w 		? curr->index + 1 : curr->index,
			curr->index + pf->w < pf->w*pf->h ? curr->index + pf->w : curr->index,
			curr->index - pf->w > 0 	? curr->index - pf->w : curr->index,
		};
		
		int f;
		for(f = 0; f < 4; f++){
			int m;
			for(m = 0; m < pf->nClosed; m++){
				if(pf->closed[m].index == neighbors[f]) break;
			}
			if(m != pf->nClosed) continue;
			
			
			int tentative_gscore = curr->g + 1;
			int nx = neighbors[f] % pf->w;
			int ny = neighbors[f] / pf->w;

			for(m = 0; m < pf->nOpen; m++){
				if(pf->open[m].index == neighbors[f]) break;			
			}

			if(m != pf->nOpen){
				if(pf->open[m].g < tentative_gscore){
					pf->open[m].f = tentative_gscore + ((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy));
					pf->open[m].g = tentative_gscore;
					pf->open[m].index = neighbors[f];
					continue;
				}
			}
			
			pf->open[pf->nOpen].f = tentative_gscore + ((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy));
			pf->open[pf->nOpen].g = tentative_gscore;
			pf->open[pf->nOpen].index = neighbors[f];
			pf->nOpen++;	
		}
	}
	printf("none\n");
	return 0;
}