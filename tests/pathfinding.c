#include "pathfinding.h"
#include "world.h"
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include "shaders.h"
#include "memory.h"
#include "game.h"
#include <GL/glew.h>

static int GetClosestNotClosed(Pathfinder *pf, int index);

float triangleArea(Vec2 a, Vec2 b, Vec2 c){
	return (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y);
}

Vec3 barycentric(Vec2 a, Vec2 b, Vec2 c, Vec2 p) { 
	 Vec3 u = Math_Vec3Cross((Vec3){c.x-a.x, b.x-a.x, a.x-p.x}, (Vec3){c.y-a.y, b.y-a.y, a.y-p.y});
	 if(fabs(u.z)<1) return (Vec3){-1,1,1};
	 return (Vec3){1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z}; 
} 

void Pathfinding_RenderDebug(Pathfinder *pf, PathfinderPath *path){

	Shaders_UseProgram(TEXTURELESS_SHADER);
	
	float matrix[16];
	Math_TranslateMatrix(matrix,(Vec3){0,0.4,0});
	Shaders_SetModelMatrix(matrix);
	Shaders_UpdateModelMatrix();
	Shaders_UpdateViewMatrix();
	Shaders_UpdateProjectionMatrix();

	glBindVertexArray(pf->vao);
	glCullFace(GL_BACK);


	//glDisable(GL_DEPTH_TEST);
	 //glBindBuffer(GL_ARRAY_BUFFER, pf->vbo);
	//glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	 //glBufferData(GL_ARRAY_BUFFER, 6 * 3 * sizeof(Vec3), (float *)&pf->verts[0].x, GL_STATIC_DRAW);
	//glDrawArrays(GL_TRIANGLES, 0, 6 * 3);
	//glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

	int k;
	//for(k = 0; k < pf->nChannel; k++){
		
		//Shaders_SetUniformColor((Vec4){1,1,1,1});		

		//Line line = pf->channel[k];
		
		 //glBindBuffer(GL_ARRAY_BUFFER, pf->vbo);
		 //glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(Vec3), (float *)&line.start.x, GL_STATIC_DRAW);
		//glDrawArrays(GL_LINES, 0, 2);
	//}



	Shaders_SetUniformColor((Vec4){1,0,1,1});
	glBindBuffer(GL_ARRAY_BUFFER, pf->vbo);
	glBufferData(GL_ARRAY_BUFFER, path->nPath  * sizeof(Vec3), (float *)&path->path[0].x, GL_STATIC_DRAW);
	glDrawArrays(GL_LINE_STRIP, 0, path->nPath);
	Shaders_SetUniformColor((Vec4){1,1,1,1});		

	Shaders_SetUniformColor((Vec4){1,1,1,1});
	glBindBuffer(GL_ARRAY_BUFFER, pf->vbo);
	glPointSize(10);
	AStarNode *closed = pf->closedFirst;
	while(closed){
		float x = (float)((int)closed->index % pf->w);
		float y = (float)((int)closed->index / pf->w);
		Vec3 pos = (Vec3){(x * PATHFINDING_NODE_GRID_SIZE),1, 
			(y * PATHFINDING_NODE_GRID_SIZE)};
		pos.x += pf->cube.x;
		pos.z += pf->cube.z;
		pos.x -= PATHFINDING_NODE_GRID_SIZE/2;
		pos.y -= PATHFINDING_NODE_GRID_SIZE/2;
		pos.z -= PATHFINDING_NODE_GRID_SIZE/2;
		Cube cube = (Cube){pos.x, 0.01,pos.z,PATHFINDING_NODE_GRID_SIZE,PATHFINDING_NODE_GRID_SIZE,
		PATHFINDING_NODE_GRID_SIZE};
		World_DrawX(cube);
		closed = closed->next;	
	}
	
	//Shaders_SetUniformColor((Vec4){0,1,1,1});		
	//for(; k < pf->nClosed; k++){
		//float x = (float)((int)closed->index % pf->w);
		//float y = (float)((int)closed->index / pf->w);
		//Vec3 pos = (Vec3){(x * PATHFINDING_NODE_GRID_SIZE),1, 
			//(y * PATHFINDING_NODE_GRID_SIZE)};
		//pos.x += pf->cube.x;
		//pos.z += pf->cube.z;
		//pos.x -= PATHFINDING_NODE_GRID_SIZE/2;
		//pos.y -= PATHFINDING_NODE_GRID_SIZE/2 - 0.3;
		//pos.z -= PATHFINDING_NODE_GRID_SIZE/2;
		//Cube cube = (Cube){pos.x, 0.01,pos.z,PATHFINDING_NODE_GRID_SIZE,PATHFINDING_NODE_GRID_SIZE,
		//PATHFINDING_NODE_GRID_SIZE};
		//World_DrawX(cube);
	//}
	//Shaders_SetUniformColor((Vec4){1,1,1,1});		

	//Shaders_SetUniformColor((Vec4){1,1,0,1});		
	//for(k=0; k < pf->nOpen; k++){
		//float x = (float)((int)pf->open[k].index % pf->w);
		//float y = (float)((int)pf->open[k].index / pf->w);
		//Vec3 pos = (Vec3){(x * PATHFINDING_NODE_GRID_SIZE),1, 
			//(y * PATHFINDING_NODE_GRID_SIZE)};
		//pos.x += pf->cube.x;
		//pos.z += pf->cube.z;
		//pos.x -= PATHFINDING_NODE_GRID_SIZE/2;
		//pos.y -= PATHFINDING_NODE_GRID_SIZE/2 - 0.3;
		//pos.z -= PATHFINDING_NODE_GRID_SIZE/2;
		//Cube cube = (Cube){pos.x, 0.01,pos.z,PATHFINDING_NODE_GRID_SIZE,PATHFINDING_NODE_GRID_SIZE,
		//PATHFINDING_NODE_GRID_SIZE};
		//World_DrawX(cube);
	//}
	glBindVertexArray(0);
	glEnable(GL_DEPTH_TEST);
}


void Pathfinding_Init(Pathfinder *pf, int w, int h){
	memset(pf, 0, sizeof(Pathfinder));
	pf->w = w; 
	pf->h = h;


	 glGenVertexArrays(1, &pf->vao);
	 glBindVertexArray(pf->vao);

    glGenBuffers(1, &pf->ebo);
	 glGenBuffers(1, &pf->vbo);
	 glBindBuffer(GL_ARRAY_BUFFER, pf->vbo);

    GLuint positionAttribute = glGetAttribLocation(Shaders_GetProgram(TEXTURED_SHADER), SHADERS_POSITION_ATTRIB);
	 glEnableVertexAttribArray(positionAttribute);
	 glVertexAttribPointer(positionAttribute, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), 0);


}

void Pathfinding_SetClosedBoundingBox(Pathfinder *pf, BoundingBox *bb){

	Vec3 tris[4][3];

	tris[0][0] = bb->points[0];
	tris[0][1] = bb->points[1];
	tris[0][2] = bb->points[2];
	
	tris[1][0] = bb->points[2];
	tris[1][1] = bb->points[3];
	tris[1][2] = bb->points[0];
	
	tris[2][0] = bb->points[1];
	tris[2][1] = bb->points[5];
	tris[2][2] = bb->points[6];
	
	tris[3][0] = bb->points[6];
	tris[3][1] = bb->points[2];
	tris[3][2] = bb->points[1];
	
	
	int k;
	int x = 0, y = 0;
	for(x = 0; x < pf->w; x++){
		for(y = 0; y < pf->h; y++){
			Vec2 p = (Vec2){x*PATHFINDING_NODE_GRID_SIZE,y*PATHFINDING_NODE_GRID_SIZE};
			for(k = 0; k < 4; k++){

				Vec3 p0 = tris[k][0];
				Vec3 p1 = tris[k][1];
				Vec3 p2 = tris[k][2];
				
				p0.y = 0;				
				p1.y = 0;				
				p2.y = 0;

				Vec2 v0 = {p0.x-pf->cube.x,p0.z-pf->cube.z};
				Vec2 v1 = {p1.x-pf->cube.x,p1.z-pf->cube.z};
				Vec2 v2 = {p2.x-pf->cube.x,p2.z-pf->cube.z};

				float w0 =  triangleArea(v1, v0, p);
				float w1 =  triangleArea(v2, v1, p);
				float w2 =  triangleArea(v0, v2, p);
				float hasNeg = w0 < 0 || w1 < 0 || w2 < 0;
				float hasPos = w0 > 0 || w1 > 0 || w2 > 0;
				if((hasNeg && hasPos)) continue;
				Pathfinding_SetClosedStatic(pf, x, y);
				break;
			}
		}
	}
}

static void SetClosed(Pathfinder *pf, int x, int y){
	int index = (x%pf->w) + (y * pf->w);
	Vec3 pos = (Vec3){(int)(x * PATHFINDING_NODE_GRID_SIZE) % pf->w, 1, 
		(y * PATHFINDING_NODE_GRID_SIZE)};
		pos.x += pf->cube.x;
		pos.z += pf->cube.z;

	if(!pf->closedFirst) {
		pf->closedFirst = malloc(sizeof(AStarNode)); 
		pf->closedFirst->next = NULL; 
		pf->closedFirst->prev = NULL; 
	}

	AStarNode *last = pf->closedFirst;
	while (last->next) { last = last->next; }
	
	last->next = malloc(sizeof(AStarNode));
	last->next->index = index;
	last->next->next = NULL;
	last->next->prev = last;
	
	pf->closedFirstObstacles = last->next;
}
void Pathfinding_SetClosedStatic(Pathfinder *pf, int x, int y){
	SetClosed(pf, x,y);
	pf->nClosedStatic++;	
}

void Pathfinding_SetClosedDynamic(Pathfinder *pf, int x, int y){
	SetClosed(pf, x,y);
}

static int GetClosestNotClosed(Pathfinder *pf, int index){

	AStarNode *curr = pf->closedFirst;
	
	while(curr){
		
		if(curr->index == index ) break;
		curr = curr->next;
	}
	if(curr){
		
			while(1){
				
				int neighbors[] = {
					index + 1,
					index + pf->w,
					index - 1,
					index - pf->w,
					index + 1 + pf->w,
					index + (pf->w -  1),
					index - 1 - pf->w,
					index - (pf->w - 1),
				};
				
				int f;
				
				for(f = 0; f < 8; f++){
					if(neighbors[f] < 0 || neighbors[f] > pf->w * pf->h) continue;
					AStarNode *tmp = pf->closedFirst;
					while(tmp){
						if(tmp->index == neighbors[f]){
							break;
						}
						tmp = tmp->next;
					}
					if(!tmp) continue;

					return neighbors[f];
				}
				index++;
			}
		}
	return index;
}

int Pathfinding_FindPathGrid(Pathfinder *pf, Vec3 pos, Vec3 goal,PathfinderPath*path){

	int x =  ( pos.x - pf->cube.x) / PATHFINDING_NODE_GRID_SIZE;
	int y =  ( pos.z - pf->cube.z) / PATHFINDING_NODE_GRID_SIZE;	
	int gx = (  goal.x - pf->cube.x) / PATHFINDING_NODE_GRID_SIZE;
	int gy = (  goal.z - pf->cube.z) / PATHFINDING_NODE_GRID_SIZE;	


	pf->openFirst = malloc(sizeof(AStarNode));	
	pf->openFirst->next = pf->openFirst->prev = NULL;
	pf->openFirst->prev = pf->openFirst;
	
	AStarNode *curr = pf->openFirst;
	
	curr->index = GetClosestNotClosed(pf, x + (y * pf->w)); 
	curr->f = 0;
	curr->g = 0;
	curr->parent = NULL;
	curr->prev = NULL;
	path->nPath = 0;

	int goalIndex = GetClosestNotClosed(pf, gx + (gy * pf->w));
	int attempts = 0;
	
	AStarNode current;
	while(pf->openFirst && attempts < 100){
		attempts++;
		curr = pf->openFirst;

		int m = 0;

		AStarNode *first = pf->openFirst;
		while(first){
			if(first->f <= curr->f ){
				curr = first;
			}
			first = first->next;
		}

		current = *curr;
		
		if(current.index == goalIndex){ 
			path->nPath = 0;
			Vec3 pathReversed[MAX_PATHFINDING_PATH];
			while(curr){
				float nx = (float)((int)curr->index % pf->w);
				float ny = (float)((int)curr->index / pf->w);
		
				Vec3 pos = (Vec3){(nx * PATHFINDING_NODE_GRID_SIZE), 1, 
				(ny * PATHFINDING_NODE_GRID_SIZE)};
				pos.x += pf->cube.x;
				pos.z += pf->cube.z;
				pathReversed[path->nPath++] = pos;
				curr = curr->parent;
			}
			
			int k;
			for(k = 0; k < path->nPath; k++){
				path->path[k] = pathReversed[path->nPath-1-k];
			}


			curr = pf->openFirst;
			while(curr){
				AStarNode *tmp = curr;
				curr = curr->next;
				free(tmp);
			}
			pf->openFirst = NULL;

			if(pf->closedFirstObstacles->next){
				curr = pf->closedFirstObstacles->next;
				while(curr){
					AStarNode *tmp = curr;
					curr = curr->next;
					free(tmp);
				}
				pf->closedFirstObstacles->next = NULL;
			}
			return 0;
		}
		
	
		if(curr->next)curr->next->prev = curr->prev;
		if(curr->prev)curr->prev->next = curr->next;

		if(curr->prev == NULL){
			pf->openFirst = NULL;
		}


		if(pf->closedFirst){
			AStarNode *last = pf->closedFirst;
			while(last->next) { last = last->next; }
			last->next = malloc(sizeof(AStarNode));
			*last->next = current; 
			last->next->next = NULL;
			last->next->prev = last;
		} else { 
			pf->closedFirst = malloc(sizeof(AStarNode));
			*pf->closedFirst = current; 
			pf->closedFirst->next = NULL;
			pf->closedFirst->prev = NULL;
		}

		int neighbors[] = {
			current.index + 1,
			current.index + pf->w,
			current.index - 1,
			current.index - pf->w,
			current.index + 1 + pf->w,
			current.index + (pf->w -  1),
			current.index - 1 - pf->w,
			current.index - (pf->w - 1),
		};
		
		int f;
		for(f = 0; f < 8; f++){
			if(neighbors[f] < 0 || neighbors[f] > pf->w * pf->h) continue;
			AStarNode *closed = pf->closedFirst;
			while(closed) {
				if(closed->index == neighbors[f]) break;
				closed = closed->next;
			}
			if(closed) continue; 

			float tentative_gscore = current.g + 1;
			float nx = (float)((int)neighbors[f] % pf->w);
			float ny = (float)((int)neighbors[f] / pf->w);

			AStarNode *open = pf->openFirst;
			while(open) { 
				if(open->index == neighbors[f]) break;			
				open = open->next;
			}

			if(open){
				if(open->g >= tentative_gscore){
					open->f = tentative_gscore + ((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy));
					open->g = tentative_gscore;
					open->index = neighbors[f];
					open->parent = curr;
					continue;
				}
			}

			// last on list			

			AStarNode *tmp;
			if(!pf->openFirst){
				pf->openFirst = malloc(sizeof(AStarNode));
				pf->openFirst->next = pf->openFirst->prev = NULL;
				tmp = NULL;
				open = pf->openFirst;
			} else {
				open = pf->openFirst;
				while(open->next) { open = open->next; }
				open->next = malloc(sizeof(AStarNode));
				tmp = open;
				open = open->next;
			
			}
			
			open->prev = tmp;
			open->next = NULL;
			if(open->prev) open->prev->next = open;
			open->f = tentative_gscore + ((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy));
			open->g = tentative_gscore;
			open->index = neighbors[f];
			open->parent = curr;
		}
	}
	
	curr = pf->openFirst;
	while(curr){
		AStarNode *tmp = curr;
		curr = curr->next;
		free(tmp);
	}
	pf->openFirst = NULL;
		
	if(pf->closedFirstObstacles->next){
		curr = pf->closedFirstObstacles->next;
		while(curr){
			AStarNode *tmp = curr;
			curr = curr->next;
			free(tmp);
		}
		pf->closedFirstObstacles->next = NULL;
	}

	return 0;
}