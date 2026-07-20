/*
Heuristic is squared distance: ((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy)). Squaring makes it wildly overestimate the true cost, so it's inadmissible — you lose the optimality guarantee and it degenerates toward greedy best-first. Use plain Euclidean, or octile distance for 8-connected grids.
Uniform step cost of 1 for diagonals too, plus that odd diagnal fudge that adds +1 only when re-examining an already-open node. Diagonals should cost ~1.414 consistently, not conditionally.
Open list is a linked list with a linear min scan, and the closed list is a linear scan too. That's O(n) per operation where classic A* uses a binary heap plus a hash set / flag array.

Actual bugs

Removing the head destroys the open list: if(curr->prev == NULL) pf->openFirst = NULL; should be pf->openFirst = curr->next. When you pop the first node — which is common — every other open node is orphaned and leaked. This alone will make searches fail or wander.
The popped node is never freed, only copied into current and into the closed list. Meanwhile open->parent = curr points at that unlinked open-list node, not at the closed-list copy. So parent chains point into memory that's leaked now and, in the cleanup loops at the end, may already have been freed — reconstruction reads dangling pointers.
Duplicate open entries: if a node is already open and tentative_gscore >= open->g, the code falls through and appends a second node for the same index instead of skipping it.
No row-wrap check on neighbors: index ± 1 at column 0 or w-1 wraps to the adjacent row. You need to reject neighbors where abs(nx - curr_x) > 1. Also neighbors[f] > pf->w * pf->h should be >=.
Negative-coordinate truncation: (pos.x - pf->cube.x) / PATHFINDING_NODE_GRID_SIZE with ints truncates toward zero, so cells left/below the origin are off by one. Use floorf before the cast.
*/

#define GLEW_STATIC
#include <GL/glew.h>
#include <SDL2/SDL_opengl.h>
#include "utils.h"
#include "pathfinding.h"
#include "world.h"
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include "shaders.h"
#include "memory.h"
#include "game.h"
#include "math.h"
static int GetClosestNotClosed(Pathfinder *pf, int index);

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
	
	glBindVertexArray(0);
	glEnable(GL_DEPTH_TEST);
}


void Pathfinding_Init(Pathfinder *pf, int w, int h){
	memset(pf, 0, sizeof(Pathfinder));
	pf->w = w; 
	pf->h = h;
	pf->cube.x = -w/2;
	pf->cube.y = -h/2;
	

	 glGenVertexArrays(1, &pf->vao);
	 glBindVertexArray(pf->vao);

    glGenBuffers(1, &pf->ebo);
	 glGenBuffers(1, &pf->vbo);
	 glBindBuffer(GL_ARRAY_BUFFER, pf->vbo);

    GLuint positionAttribute = glGetAttribLocation(Shaders_GetProgram(TEXTURED_SHADER), SHADERS_POSITION_ATTRIB);
	 glEnableVertexAttribArray(positionAttribute);
	 glVertexAttribPointer(positionAttribute, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), 0);


}

static void SetClosedBoundingBox(Pathfinder *pf, BoundingBox *bb,
	void (*close)(Pathfinder *pf, int x, int y)){
	
	Vec3 oldscale = bb->scale;
	bb->scale = Math_Vec3MultFloat(bb->scale, 1.3);
	BoundingBox_UpdatePoints(bb); 

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
	
	bb->scale = oldscale;
	BoundingBox_UpdatePoints(bb); 		
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

				float w0 =  Math_TriangleAreaVec2(v1, v0, p);
				float w1 =  Math_TriangleAreaVec2(v2, v1, p);
				float w2 =  Math_TriangleAreaVec2(v0, v2, p);
				float hasNeg = w0 < 0 || w1 < 0 || w2 < 0;
				float hasPos = w0 > 0 || w1 > 0 || w2 > 0;
				if((hasNeg && hasPos)) continue;
				close(pf, x, y);
				break;
			}
		}
	}

}

void Pathfinding_SetClosedBoundingBoxDynamic(Pathfinder *pf, BoundingBox *bb){
	SetClosedBoundingBox(pf,bb,Pathfinding_SetClosedDynamic);
}
void Pathfinding_SetClosedBoundingBoxStatic(Pathfinder *pf, BoundingBox *bb){
	SetClosedBoundingBox(pf,bb,Pathfinding_SetClosedStatic);
}

void Pathfinding_ClearDynamic(Pathfinder *pf){
	if(pf->closedStaticLast){

		AStarNode *curr = pf->closedStaticLast;
		curr = curr->next;
		
		while(curr){
			AStarNode *tmp = curr;
			curr = curr->next;
			free(tmp);
		}
		pf->closedStaticLast->next = NULL;
		pf->closedObstaclesLast = pf->closedStaticLast;
	}

}

static void SetClosed(Pathfinder *pf, int x, int y){
	int index = (x%pf->w) + (y * pf->w);
	Vec3 pos = (Vec3){(int)(x * PATHFINDING_NODE_GRID_SIZE) % pf->w, 1, 
		(y * PATHFINDING_NODE_GRID_SIZE)};
		pos.x += pf->cube.x;
		pos.z += pf->cube.z;

	if(!pf->closedFirst){
		pf->closedFirst = malloc(sizeof(AStarNode));
		pf->closedFirst->index = index;
		pf->closedFirst->next = NULL;
		pf->closedFirst->prev = NULL;
		pf->closedObstaclesLast = pf->closedFirst;
		return;
	}

	AStarNode *last = pf->closedFirst;
	while(last->next) { last = last->next; }
	
	last->next = malloc(sizeof(AStarNode));
	last->next->index = index;
	last->next->next = NULL;
	last->next->prev = last;
	
	pf->closedObstaclesLast = last->next;
}
void Pathfinding_SetClosedStatic(Pathfinder *pf, int x, int y){
	SetClosed(pf, x,y);
	pf->closedStaticLast = pf->closedObstaclesLast;	
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
					if(tmp) continue;

					return neighbors[f];
				}
				if(index > pf->w * pf->h) return 0;
				index++;
			}
		}
	return index;
}

int Pathfinding_FindPathGrid(Pathfinder *pf, Vec3 pos, Vec3 goal,PathfinderPath*path){

	pf->closed = HashTable_Create();
	pf->open = HashTable_Create();

	AStarNode *closed = pf->closedFirst;	
	while(closed){
		HashTable_Insert(pf->closed, (const char*)&closed->index, (char*)closed);
		closed = closed->next;
	}
	
	
	int x =  abs( pos.x - pf->cube.x) / PATHFINDING_NODE_GRID_SIZE;
	int y =  abs( pos.z - pf->cube.z) / PATHFINDING_NODE_GRID_SIZE;	
	int gx = abs(  goal.x - pf->cube.x) / PATHFINDING_NODE_GRID_SIZE;
	int gy = abs(  goal.z - pf->cube.z) / PATHFINDING_NODE_GRID_SIZE;	
	
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

	HashTable_Insert(pf->open, (char * ) &curr->index, (char *)curr);

	int goalIndex = GetClosestNotClosed(pf, gx + (gy * pf->w));
	int attempts = 0;

	AStarNode current;
	while(pf->openFirst && attempts < 1000){
		attempts++;
		curr = pf->openFirst;

		int m = 0;

		AStarNode *first = pf->openFirst;
		while(first){
			if(first->f < curr->f ){
				curr = first;
			}
			first = first->next;
		}

		current = *curr;
		
		if(current.index == goalIndex){ 
			path->nPath = 0;
			Vec3 pathReversed[MAX_PATHFINDING_PATH];
			while(curr && path->nPath < MAX_PATHFINDING_PATH){
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

			if(pf->closedObstaclesLast && pf->closedObstaclesLast->next){
				curr = pf->closedObstaclesLast->next;
				while(curr){
					AStarNode *tmp = curr;
					curr = curr->next;
					free(tmp);
				}
				pf->closedObstaclesLast->next = NULL;
			}

			HashTable_Free(pf->open);
			HashTable_Free(pf->closed);
			return 0;
		}

		HashTable_Delete(pf->open, (char * ) &curr->index);
		HashTable_Insert(pf->closed, (char * ) &curr->index, (char *)curr);
		
		if(curr->next)curr->next->prev = curr->prev;
		if(curr->prev)curr->prev->next = curr->next;

		if(curr->prev == NULL){
			pf->openFirst = curr->next;
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
	
			AStarNode *closed = (AStarNode *)HashTable_Search(pf->closed, (char *)&neighbors[f]);
			if(closed) continue; 

			int tentative_gscore = current.g + f/4 ? 1.414 : 1;

			int nx = neighbors[f] % pf->w;
			int ny = neighbors[f] / pf->w;

			AStarNode *open = (AStarNode *)HashTable_Search(pf->open, (char *)&neighbors[f]);

			if(open){

				if(tentative_gscore < open->g){ // this path is better
					open->f = tentative_gscore + sqrt((float)((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy)));
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
			open->f = tentative_gscore + sqrt((float)((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy)));
			open->g = tentative_gscore;
			open->index = neighbors[f];
			open->parent = curr;

			HashTable_Insert(pf->open, (char * ) &neighbors[f], (char *)open);
		}
	}
	curr = pf->openFirst;
	while(curr){
		AStarNode *tmp = curr;
		curr = curr->next;
		free(tmp);
	}
	pf->openFirst = NULL;
		
	if(pf->closedObstaclesLast && pf->closedObstaclesLast->next){
		curr = pf->closedObstaclesLast->next;
		while(curr){
			AStarNode *tmp = curr;
			curr = curr->next;
			free(tmp);
		}
		pf->closedObstaclesLast->next = NULL;
	}

	HashTable_Free(pf->open);
	HashTable_Free(pf->closed);
	
	return 0;
}