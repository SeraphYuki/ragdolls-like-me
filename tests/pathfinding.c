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
float triArea(PathfindingTri tri){
	Vec3 v1 = Math_Vec3SubVec3(tri.points[1], tri.points[0]);
	Vec3 v2 = Math_Vec3SubVec3(tri.points[2], tri.points[0]);
	return Math_Vec3Magnitude(Math_Vec3Cross(v1,v2))/2;
}

 int equalMagnitude(Vec3 v, Vec3 v1){
	return (Math_Vec3Magnitude(Math_Vec3SubVec3(v, v1)) < 0.00001);
}

 float triarea2(Vec3 a, Vec3 b, Vec3 c){

	Vec3 v1 = Math_Vec3SubVec3(b, a);
	Vec3 v2 = Math_Vec3SubVec3(c, a);
	return Math_Vec3Magnitude(Math_Vec3Cross(v1,v2));

	 //const float ax = b.x - a.x;
	 //const float ay = b.y - a.y;
	 //const float bx = c.x - a.x;
	 //const float by = c.y - a.y;
	 //return bx*ay - ax*by;
}


float vdistsqr(float *v1, float *v2){
	return Math_Vec2Magnitude(Math_Vec2SubVec2(*(Vec2*)v1,*(Vec2*)v2));
}

int AlmostEqual(float a, float b){
 static const float eq = 0.001f*0.001f;
 return fabs(a - b) < eq;
}
int AlmostEqualVec3(Vec3 a, Vec3 b){
 static const float eq = 0.001f*0.001f;
 return fabs(a.x - b.x) < eq && fabs(a.y - b.y) < eq && fabs(a.z - b.z) < eq;
}

int AreCollinear(Line line1, Line line2){
	float area1 = triarea2(line1.start, line1.end, line2.start);
	float area2 = triarea2(line1.start, line1.end, line2.start);
	return (AlmostEqual(area1,0) && AlmostEqual(area2,0));
}

 // Check two collinear line segments to see if they overlap by sorting the points.
 // Algorithm source: http://stackoverflow.com/a/17152247
float GetOverlap( Line line1, Line line2, Line *ret){

   struct point {
		int index;
		Vec3 point;
	};
	
	struct point points[4]  = {
	   {  0, line1.start },
	   {  0, line1.end },
	   {  1, line2.start },
	   {  1, line2.end }
	 };

	int sort(const void *point1, const void *point2){

		struct point a = *(struct point *)point1;
		struct point b = *(struct point *)point2;

		if (a.point.x < b.point.x) return -1;
		else if (a.point.x > b.point.x) return 1;
		if (a.point.y < b.point.y) return -1;
		else if (a.point.y > b.point.y) return 1;
		//if (a.point.z < b.point.z) return -1;
		//else if (a.point.z > b.point.z) return 1;
		return 0;
	} 
	

	qsort(points, 4, sizeof(struct point), sort);

    // If the first two points in the array come from the same line, no overlap
	 int noOverlap = points[0].index == points[1].index;

	 // If the two middle points in the array are the same coordinates, then there is a
	 // single point of overlap.

	 int singlePointOverlap = equalMagnitude(points[1].point, points[2].point);

	 if (noOverlap || singlePointOverlap) return 0;

	*ret = (Line){points[1].point, points[2].point};
	 return 1;
}
float  angleDifference(float x, float y) {
	float a = x - y;
	float i = a + PI;
	float j = PI * 2;
	a = i - floor(i / j) * j; // (a+180) % 360; this ensures the correct sign
	a -= PI;
	return a;
}

void vcpy(float *v1, float *v2){
	memcpy(v1,v2,sizeof(float)*2);
}

void stringPull(Pathfinder *pf, Line *portals, int nPortals, PathfinderPath *path){

	 Vec3 pts[MAX_PATHFINDING_NODES];
	int nPts = 0;
	 // Init scan state
	 int apexIndex = 0;
	   int leftIndex = 0;
	   int rightIndex = 0;

    Vec3 portalApex = portals[0].start;
	 Vec3 portalLeft = portals[0].start;
	 Vec3 portalRight = portals[0].end;

    // Add start point.
	 pts[nPts++] = portalApex;

	int i;
	
	 for ( i = 1; i < nPortals; i++) {
	    //Find the next portal vertices
	   Vec3 left = portals[i].start;
	   Vec3 right = portals[i].end;

       //Update right vertex.
	   if (triarea2(portalApex, portalRight, right) <= 0.0) {
	     if (AlmostEqualVec3(portalApex,portalRight) || triarea2(portalApex, portalLeft, right) > 0.0) {
	        //Tighten the funnel.
	       portalRight = right;
	       rightIndex = i;
	     } else {
	        //Right vertex just crossed over the left vertex, so the left vertex should
	        //now be part of the path.
			pts[nPts++] = portalLeft;
			
	        //Restart scan from portal left point.

           //Make current left the new apex.
	       portalApex = portalLeft;
	       apexIndex = leftIndex;
	        //Reset portal
	       portalLeft = portalApex;
	       portalRight = portalApex;
	       leftIndex = apexIndex;
	       rightIndex = apexIndex;
	        //Restart scan
	       i = apexIndex;
	       continue;
	     }
	   }

       //Update left vertex.
	   if (triarea2(portalApex, portalLeft, left) >= 0.0) {
	     if(AlmostEqualVec3(portalApex,portalLeft) || triarea2(portalApex, portalRight, left) < 0.0) {
	        //Tighten the funnel.
	       portalLeft = left;
	       leftIndex = i;
	     } else {
	        //Left vertex just crossed over the right vertex, so the right vertex should
	        //now be part of the path
	       pts[nPts++] = (portalRight);

           //Restart scan from portal right point.

           //Make current right the new apex.
	       portalApex = portalRight;
	       apexIndex = rightIndex;
	        //Reset portal
	       portalLeft = portalApex;
	       portalRight = portalApex;
	       leftIndex = apexIndex;
	       rightIndex = apexIndex;
	        //Restart scan
	       i = apexIndex;
	       continue;
	     }
	   }
	 }

    if (nPts == 0 || !AlmostEqualVec3(pts[nPts - 1],(portals[nPortals - 1].start))) {
	   // Append last point to path.
	   pts[nPts++] = (portals[nPortals - 1].start);
	 }
	
	Vec3 lastPoint = (Vec3){0,0};
	int k;
	for(k = 0; k < nPts; k++){
		if(AlmostEqualVec3(pts[k],lastPoint)) continue;
		path->path[k] = pts[k];
		lastPoint = pts[k];
	}
	path->nPath = nPts;
 }

static void LoadNavFile(Pathfinder *pf, const char *path){
	int k;
	 u16 stride = sizeof(Vec3);
	
	 void *offset = (void *)sizeof(Vec3);

	 FILE *fp = fopen(path, "rb");

	 fread(&pf->nFaces, 1, sizeof(int), fp);
	pf->nVerts = pf->nFaces*3;
	 int size = stride * pf->nVerts;

    u8 *vboData = (u8 *)Memory_StackAlloc(TEMP_STACK, size);

    // Deflate_Read(fp, vboData, size);
	 
	 fread(vboData, 1, size, fp);


	pf->cube.x = pf->cube.y = pf->cube.z = HUGE_VAL;
	pf->cube.w = pf->cube.h = pf->cube.d = -HUGE_VAL;
	 for(k = 0; k < pf->nVerts; k++){
	     Vec3 *pos = (Vec3 *)&vboData[(stride * k)];

		pf->verts[k] = *pos;


		AStarNode *node = &pf->nodes[k];
		node->index = k;
		node->f = HUGE_VAL;
		
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
	 fclose(fp);
	
	 for(k = 0; k < pf->nFaces; k++){
	     pf->tris[k].points[0]  = pf->verts[k*3];
	     pf->tris[k].points[1]  = pf->verts[(k*3) + 1];
	     pf->tris[k].points[2]  = pf->verts[(k*3) + 2];
		pf->tris[k].index = k;
	     pf->tris[k].centroid = Math_Vec3MultFloat(Math_Vec3AddVec3(
						Math_Vec3AddVec3(pf->tris[k].points[0],pf->tris[k].points[1]),
									pf->tris[k].points[0]), 1.0f/3);

		float mag1 = Math_Vec3Magnitude(Math_Vec3SubVec3(pf->tris[k].points[0],pf->tris[k].points[1]));
		float mag2 = Math_Vec3Magnitude(Math_Vec3SubVec3(pf->tris[k].points[0],pf->tris[k].points[2]));
		pf->tris[k].radius = mag1 > mag2 ? mag1 : mag2;
	
	}
	
	for(k = 0; k < pf->nFaces; k++){
		pf->tris[k].index = k;
	}

}

float triangleArea(Vec2 a, Vec2 b, Vec2 c){
	return (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y);
}

Vec3 barycentric(Vec2 a, Vec2 b, Vec2 c, Vec2 p) { 
	 Vec3 u = Math_Vec3Cross((Vec3){c.x-a.x, b.x-a.x, a.x-p.x}, (Vec3){c.y-a.y, b.y-a.y, a.y-p.y});
	 if(fabs(u.z)<1) return (Vec3){-1,1,1};
	 return (Vec3){1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z}; 
} 
void Pathfinding_LoadNavGrid(Pathfinder *pf, const char *path){
	
	LoadNavFile(pf, path);

	pf->w = ceil(pf->cube.w) * PATHFINDING_NODE_GRID_SIZE;
	pf->h = ceil(pf->cube.d) * PATHFINDING_NODE_GRID_SIZE;
	
	int k;
	int x = 0, y = 0;
	for(x = 0; x < pf->w; x++){
		for(y = 0; y < pf->h; y++){

			Vec2 p = (Vec2){x,y};
			int inTriangle = 0;
			for(k = 0; k < pf->nFaces; k++){

				Vec3 p0 = pf->tris[k].points[0];
				Vec3 p1 = pf->tris[k].points[1];
				Vec3 p2 = pf->tris[k].points[2];
				Vec2 v0 = {p0.x-pf->cube.x,p0.z-pf->cube.z};
				Vec2 v1 = {p1.x-pf->cube.x,p1.z-pf->cube.z};
				Vec2 v2 = {p2.x-pf->cube.x,p2.z-pf->cube.z};

				float w0 =  triangleArea(v1, v0, p);
				float w1 =  triangleArea(v2, v1, p);
				float w2 =  triangleArea(v0, v2, p);
				float hasNeg = w0 < 0 || w1 < 0 || w2 < 0;
				float hasPos = w0 > 0 || w1 > 0 || w2 > 0;
				if((hasNeg && hasPos)) continue;
				inTriangle = 1;				
				break;
			}
			if(!inTriangle)
				Pathfinding_SetClosedStatic(pf, x, y);
		}
	}
}

void Pathfinding_LoadNavMesh(Pathfinder *pf, const char *path){

	LoadNavFile(pf, path);
	int k;
	 for(k = 0; k < pf->nFaces; k++){
		int j;
		PathfindingTri *tri = &pf->tris[k];
		  for(j = k+1; j < pf->nFaces; j++){
			PathfindingTri *tri2 = &pf->tris[j];
			
			float distance = Math_Vec3Magnitude(Math_Vec3SubVec3(tri->centroid, tri2->centroid));
			// > radius
			if(distance > tri->radius/2 + tri2->radius/2) continue;
	
			
			int edge, otherEdge;
			
			for(edge = 0; edge < 3; edge++){
			
				Line edge1 =(Line){tri->points[edge],tri->points[(edge+1) % 3]};
			
				for(otherEdge = 0; otherEdge < 3; otherEdge++){
					Line edge2 =(Line){tri2->points[otherEdge],tri2->points[(otherEdge+1) % 3]};
					
					 //colinear
					if(!AreCollinear(edge1, edge2)) continue;
					
					Line overlapLine;
					int overlap = GetOverlap(edge1, edge2, &overlapLine);
					
					if(!overlap) continue;
					
					//!overlap
					tri->neighbors[tri->nNeighbors++] = tri2;
					tri2->neighbors[tri2->nNeighbors++] = tri;


		            // Calculate the portal between the two polygons - this needs to be in
		            // counter-clockwise order, relative to each polygon
		            Vec3 p1 = overlapLine.start;
		            Vec3 p2 = overlapLine.end;
		            float edgeStartAngle = Math_Vec3Dot(tri->centroid, edge1.start);
		            float a1 = Math_Vec3Dot(tri->centroid, overlapLine.start);
		            float a2 = Math_Vec3Dot(tri->centroid, overlapLine.end);
		            float d1 = angleDifference(edgeStartAngle, a1);
		            float d2 = angleDifference(edgeStartAngle, a2);
		            if (d1 < d2) {
		              tri->portals[tri->nPortals++] = (Line){p1,p2};
		            } else {
		              tri->portals[tri->nPortals++] = (Line){p2,p1};
		            }

		            edgeStartAngle = Math_Vec3Dot(tri2->centroid, edge2.start);
		            a1 = Math_Vec3Dot(tri2->centroid, overlapLine.start);
		            a2 = Math_Vec3Dot(tri2->centroid, overlapLine.end);
		            d1 = angleDifference(edgeStartAngle, a1);
		            d2 = angleDifference(edgeStartAngle, a2);
		            if (d1 < d2) {
		              tri2->portals[tri2->nPortals++] = (Line){p1,p2};
		            } else {
		              tri2->portals[tri2->nPortals++] = (Line){p2,p1};
		            }
				}
			}
		}
	}



    glBindVertexArray(0);

		
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

	for(k = 0; k < pf->nClosedObstacles; k++){
		float x = (float)((int)pf->closed[k].index % pf->w);
		float y = (float)((int)pf->closed[k].index / pf->w);
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
		
	}
	
	//Shaders_SetUniformColor((Vec4){0,1,1,1});		
	//for(; k < pf->nClosed; k++){
		//float x = (float)((int)pf->closed[k].index % pf->w);
		//float y = (float)((int)pf->closed[k].index / pf->w);
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

static int GetClosestNode(Pathfinder *pf, Vec3 point){
	
	float min = HUGE_VAL;
	int minIndex = 0;
	
	int k;
	for(k = 0; k < pf->nFaces; k++){
		
		float distance = Math_Vec3Magnitude(Math_Vec3SubVec3(point,pf->tris[k].centroid));
		
		if(distance < min){
			min = distance;
			minIndex = k;
		}
	}
	return minIndex;
}

int Pathfinding_FindPath(Pathfinder *pf, Vec3 pos, Vec3 goal, PathfinderPath *path){
	
	path->nPath = 0;
	pf->nOpen = 0;
	pf->nClosed = 0;

	AStarNode *curr = &pf->open[0];
	

	curr->index = GetClosestNode(pf, pos);
	curr->f = HUGE_VAL;
	curr->g = 0;
	curr->parent = NULL;
	pf->nOpen++;

	int goalIndex = GetClosestNode(pf, goal);
	
	goal = pf->tris[goalIndex].centroid;

	while(pf->nOpen > 0 && pf->nClosed < MAX_PATHFINDING_NODES){
		
		curr = &pf->open[0];
		int m, currIndex = curr->index;
		for(m = 0; m < pf->nOpen; m++){
			if( pf->open[m].f < curr->f){
				curr = &pf->open[m];
				currIndex = m;
			}	
		}
		
		pf->closed[pf->nClosed++] = *curr;

		curr = &pf->closed[pf->nClosed-1];
		
		for(m = currIndex; m < pf->nOpen-1; m++){
			pf->open[m] = pf->open[m+1];
		}
		pf->nOpen--;
		

		if(curr->index == goalIndex){
			
			Vec3 pathReversed[MAX_PATHFINDING_NODES];
			int pathIndicies[MAX_PATHFINDING_NODES];
			while(curr != NULL){
				pathIndicies[path->nPath] = curr->index;
				pathReversed[path->nPath++] = pf->tris[curr->index].centroid;
				curr = curr->parent;
			}
			
			int k;
			for(k = 0; k < path->nPath; k++){
				path->path[k] = pathReversed[path->nPath-1-k];
				path->pathIndicies[k] = pathIndicies[path->nPath-1-k];
			}
			pf->nChannel = 0;
			
			for(k = 0; k < path->nPath-1; k++){
				PathfindingTri tri = pf->tris[path->pathIndicies[k]];
				PathfindingTri tri2 = pf->tris[path->pathIndicies[k + 1]];
							
				Line portal;
				int f;
				for(f = 0; f < tri.nNeighbors; f++){
					if(tri.neighbors[f]->index == tri2.index){
						portal = tri.portals[f];
					}
				}

				pf->channel[pf->nChannel++] = portal;
 			}
			pf->channel[pf->nChannel++] = (Line){goal, goal};	

			stringPull(pf, pf->channel, pf->nChannel, path);
			
			return 0;			
		}
		
		
		int f;
		for(f = 0; f < pf->tris[curr->index].nNeighbors; f++){
			PathfindingTri *neighbor = pf->tris[curr->index].neighbors[f];
			
			int m;
			for(m = 0; m < pf->nClosed; m++){
				if(pf->closed[m].index == neighbor->index) break;
			}
			if(m != pf->nClosed && pf->nClosed > 0) continue;
			
			int tentative_gscore = curr->g + Math_Vec3Magnitude(
				Math_Vec3SubVec3(neighbor->centroid, pf->tris[curr->index].centroid));

			for(m = 0; m < pf->nOpen; m++){
				if(pf->open[m].index == neighbor->index) break;			
			}

			if(m != pf->nOpen && pf->nOpen > 0){
				if(pf->open[m].g < tentative_gscore){
					pf->open[m].f = tentative_gscore + 
						Math_Vec3Magnitude(Math_Vec3SubVec3(neighbor->centroid, goal));
					pf->open[m].g = tentative_gscore;
					pf->open[m].index = neighbor->index;
					pf->open[m].parent = curr;
				}
				continue;
			}

			pf->open[pf->nOpen].f = tentative_gscore + 
					Math_Vec3Magnitude(Math_Vec3SubVec3(neighbor->centroid, goal));
			pf->open[pf->nOpen].g = tentative_gscore;
			pf->open[pf->nOpen].index = neighbor->index;
			pf->open[pf->nOpen].parent = curr;
			pf->nOpen++;
		}
	}
	return 0;

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
	AStarNode node;
	Vec3 pos = (Vec3){(int)(x * PATHFINDING_NODE_GRID_SIZE) % pf->w, 1, 
		(y * PATHFINDING_NODE_GRID_SIZE)};
		pos.x += pf->cube.x;
		pos.z += pf->cube.z;

	pf->closedVerts[pf->nClosed] = pos;
	node.index = index;
	pf->closed[pf->nClosed] = node;
	pf->nClosed++;
	pf->nClosedObstacles++;

}
void Pathfinding_SetClosedStatic(Pathfinder *pf, int x, int y){
	SetClosed(pf, x,y);
	pf->nClosedStatic++;	
}

void Pathfinding_SetClosedDynamic(Pathfinder *pf, int x, int y){
	SetClosed(pf, x,y);
}

void Pathfinding_SetClosedGrid(Pathfinder *pf, Vec3 pos){
	AStarNode node;

	pos.x -= pf->cube.x;
	pos.z -= pf->cube.z;
	int x = (pos.x / PATHFINDING_NODE_GRID_SIZE);
	int y = (pos.z / PATHFINDING_NODE_GRID_SIZE);
	int index = x + (y * pf->w);

	pos.y += 1;
	pf->closedVerts[pf->nClosed] = pos;
	node.index = index;
	pf->closed[pf->nClosed] = node;
	pf->nClosed++;
	pf->nClosedObstacles++;
}
void Pathfinding_SetOpenGrid(Pathfinder *pf, Vec3 pos){
	AStarNode node;

	pos.x -= pf->cube.x;
	pos.z -= pf->cube.z;
	int x = (pos.x / PATHFINDING_NODE_GRID_SIZE);
	int y = (pos.z / PATHFINDING_NODE_GRID_SIZE);
	int index = x + (y * pf->w);

	int m;
	for(m = 0; m < pf->nClosedObstacles; m++){
		if(pf->closed[m].index == index) break;
	}
	
	if(m >= 0 && pf->nClosedObstacles > 0){
		pf->closed[m] = pf->closed[pf->nClosedObstacles-1];
		pf->nClosed--;
		pf->nClosedObstacles--;
	}
}

static int GetClosestNotClosed(Pathfinder *pf, int index){
	int m;
	for(m = 0; m < pf->nClosed; m++){
		if(pf->closed[m].index == index ) break;
	}
	if(m != pf->nClosed &&  pf->nClosed > 0) {
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
				for(m = 0; m < pf->nClosed; m++){
					if(pf->closed[m].index == neighbors[f]) break;
				}
				if(m != pf->nClosed &&  pf->nClosed > 0){
					continue;
				}
				break;
			}
			if(f == 8) { index++; continue; }
			return neighbors[f];	
		}
	}
	return index;
}

int Pathfinding_FindPathGrid(Pathfinder *pf, Vec3 pos, Vec3 goal,PathfinderPath*path){

	int x =  ( pos.x - pf->cube.x) / PATHFINDING_NODE_GRID_SIZE;
	int y =  ( pos.z - pf->cube.z) / PATHFINDING_NODE_GRID_SIZE;	
	int gx = (  goal.x - pf->cube.x) / PATHFINDING_NODE_GRID_SIZE;
	int gy = (  goal.z - pf->cube.z) / PATHFINDING_NODE_GRID_SIZE;	
	
	AStarNode *curr = &pf->open[0];
	curr->index = GetClosestNotClosed(pf, x + (y * pf->w)); 

	curr->f = 0;
	curr->g = 0;
	path->nPath = 0;
	curr->parent = NULL;
	pf->nClosed = pf->nClosedStatic;	
	pf->nOpen = 1;
	
	int goalIndex = GetClosestNotClosed(pf, gx + (gy * pf->w));
		
	


	while(pf->nOpen > 0 && pf->nClosed < MAX_PATHFINDING_NODES){
		
		curr = &pf->open[0];

		int m, currIndex = curr->index;
		for(m = 0; m < pf->nOpen; m++){
			if( pf->open[m].f <= curr->f){
				curr = &pf->open[m];
				currIndex = m;
			}	
		}

		pf->closed[pf->nClosed++] = *curr;
				
		curr = &pf->closed[pf->nClosed-1];

		for(m = currIndex; m < pf->nOpen-1; m++){
			pf->open[m] = pf->open[m+1];
		}
		
		pf->nOpen--;


		if(curr->index == goalIndex){ 
			path->nPath = 0;
			Vec3 pathReversed[MAX_PATHFINDING_NODES];
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
			pf->nClosed = pf->nClosedStatic;
			return 0;
		}

		int neighbors[] = {
			curr->index + 1,
			curr->index + pf->w,
			curr->index - 1,
			curr->index - pf->w,
			curr->index + 1 + pf->w,
			curr->index + (pf->w -  1),
			curr->index - 1 - pf->w,
			curr->index - (pf->w - 1),

		};
		
		int f;
		for(f = 0; f < 8; f++){

			if(neighbors[f] < 0 || neighbors[f] > pf->w * pf->h) continue;
			
			int m;
			for(m = 0; m < pf->nClosed; m++){
				if(pf->closed[m].index == neighbors[f]) break;
			}
			if(m != pf->nClosed &&  pf->nClosed > 0) {
				continue;
			}
			
			float tentative_gscore = curr->g + 1;
			float nx = (float)((int)neighbors[f] % pf->w);
			float ny = (float)((int)neighbors[f] / pf->w);

			for(m = 0; m < pf->nOpen; m++){
				if(pf->open[m].index == neighbors[f]) break;			
			}

			if(m != pf->nOpen && pf->nOpen > 0){
				if(pf->open[m].g >= tentative_gscore){
					pf->open[m].f = tentative_gscore + ((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy));
					pf->open[m].g = tentative_gscore;
					pf->open[m].index = neighbors[f];
					pf->open[m].parent = curr;
					continue;
				}
			}
			
			pf->open[pf->nOpen].f = tentative_gscore + ((nx-gx)*(nx-gx)) + ((ny-gy)*(ny-gy));
			pf->open[pf->nOpen].g = tentative_gscore;
			pf->open[pf->nOpen].index = neighbors[f];
			pf->open[pf->nOpen].parent = curr;
			
			pf->nOpen++;
		}
	}
	pf->nClosed = pf->nClosedStatic;
	return 0;
}