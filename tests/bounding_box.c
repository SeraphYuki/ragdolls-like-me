#include "bounding_box.h"
#include "window.h"
#include "object.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

static float GetOverlap(float minA, float maxA, float minB, float maxB){

    if(minA > maxB || minB > maxA)
	     return 0; 

    double smallest = HUGE_VAL;
	 double max0min1 = maxA - minB;
	 double max1min0 = maxB - minA;
	 double min0max1 = minA - maxB;

    if(minA < minB){
	     if(maxA < maxB)
	         smallest = maxA - minB;
	     else
	         if(maxA - minB < maxB - minA) 
	         	smallest = max0min1; 
	         else 
	            smallest = -max1min0;
	 } else {
	     if(maxA > maxB)
	         smallest = min0max1;            
	     else
	         if(max0min1 < max1min0) 
	         	smallest = max0min1; 
	         else 
	            smallest = -max1min0;
	 }

    return smallest;
 }

static int CheckCollision(Vec3 *axes, int nAxes, Vec3 *pointsA, int nPointsA, Vec3 *pointsB, int nPointsB, 
	float *minOverlap, Vec3 *minAxis){

	*minOverlap = HUGE_VAL;
	int k;
	for(k = 0; k < nAxes; k++){

		float minA = HUGE_VAL, maxA = -HUGE_VAL, minB = HUGE_VAL, maxB = -HUGE_VAL;

		if(axes[k].x == 0 && axes[k].y == 0 && axes[k].z == 0 ) return 0;
		
		axes[k] = Math_Vec3Normalize(axes[k]);

		int j;
		for(j = 0; j < nPointsB; j++){
			float dot = Math_Vec3Dot(axes[k], pointsB[j]);
			if(dot < minB)
				minB = dot;
			if(dot > maxB)
				maxB = dot;
		}
		for(j = 0; j < nPointsA; j++){
			float dot = Math_Vec3Dot(axes[k], pointsA[j]);
			if(dot < minA)
				minA = dot;
			if(dot > maxA)
				maxA = dot;
		}

		float overlap = GetOverlap(minA, maxA,minB,maxB);
		if(fabs(overlap) < *minOverlap){
			*minOverlap = fabs(overlap);
			if(overlap > 0)
				*minAxis = axes[k];
			else
				*minAxis = (Vec3){-axes[k].x,-axes[k].y,-axes[k].z};
		}

		if(overlap == 0) return 0;
	}

	return 1;
}

void PolySoupLeaf_Init(PolySoupLeaf *o, int index, int divisions){

	o->numTris = 0;
	o->level = 0;

	if(o->parent != NULL){
		o->width = o->parent->width / 2;
		o->height = o->parent->height / 2;
		o->depth = o->parent->depth / 2;
		o->pos.x = o->parent->pos.x + ((index % 2) * o->width);
		o->pos.y = o->parent->pos.y + (round((index % 4) / 2) * o->height);
		o->pos.z = o->parent->pos.z + (round(index / 4) * o->depth);
		o->level = o->parent->level+1;
	}
	
	o->cube = (Cube){o->pos.x, o->pos.y, o->pos.z, o->width, o->height, o->depth};

	int k;
	if(o->level <= divisions){
		for(k = 0; k < 8; k++){
			o->children[k] = (PolySoupLeaf *)Memory_StackAlloc(STACK_BOTTOM,sizeof(PolySoupLeaf));
			memset(o->children[k], 0, sizeof(PolySoupLeaf));
			o->children[k]->parent = o;
			PolySoupLeaf_Init(o->children[k], k, divisions);
		}
	}
}

int PolySoupLeaf_Insert(PolySoupLeaf *o, Vec3 v1, Vec3 v2, Vec3 v3){

	printf("%f %f %f\n", o->cube.x, o->cube.y, o->cube.z);
	if(!(((v1.x < o->cube.x + o->cube.w && v1.x > o->cube.x) &&
		 (v1.y < o->cube.y + o->cube.h && v1.y > o->cube.y) &&
		 (v1.z < o->cube.z + o->cube.d && v1.z > o->cube.z))
	&&  ((v2.x < o->cube.x + o->cube.w && v2.x > o->cube.x) &&
		 (v2.y < o->cube.y + o->cube.h && v2.y > o->cube.y) &&
		 (v2.z < o->cube.z + o->cube.d && v2.z > o->cube.z))
	&&  ((v3.x < o->cube.x + o->cube.w && v3.x > o->cube.x) &&
		 (v3.y < o->cube.y + o->cube.h && v3.y > o->cube.y) &&
		 (v3.z < o->cube.z + o->cube.d && v3.z > o->cube.z)))){
			return 0;
	}

	if(o->children[0]){

		int k;
		for(k = 0; k < 8; k++)
			if(PolySoupLeaf_Insert(o->children[k], v1, v2, v3))
				return 1;

	}

	o->verts = (Vec3 *)realloc(o->verts, sizeof(Vec3) * ++o->numTris *  3);

	o->verts[(o->numTris-1) * 3] = v1;
	o->verts[((o->numTris-1) * 3) + 1] = v2;
	o->verts[((o->numTris-1) * 3) + 2] = v3;
	
	return 1;
}

void PolySoupLeaf_ResolveCollisions(Game *game,
	PolySoupLeaf *o, Object *obj, BoundingBox *box, Cube minCube){

	if(!Math_CheckCollisionCube(o->cube, minCube))
		return;

	int k;
	for(k = 0; k < o->numTris*3; k+=3){
		Vec3 p0 = o->verts[k];
		Vec3 p1 = o->verts[k+1];
		Vec3 p2 = o->verts[k+2];
		Vec3 point;

		if(Math_IntersectLineTriangle(box->pos,
			(Vec3){0,-1,0}, p0, p1, p2, &point)){
				
			box->pos.y = point.y;
			obj->ObjUpdate(obj);
		}
	}

	if(o->children[0])
		for(k = 0; k < 8; k++)
			PolySoupLeaf_ResolveCollisions(game, o->children[k], obj, box, minCube);
}

int BoundingBox_ResolveSoupY(Game *game ,Object *obj, BoundingBox *bb, BoundingBox *bb2){

	PolySoupLeaf_ResolveCollisions(game,
	&bb2->soup.root, obj, bb, bb->wsCube);

	return 0;	
}

void BoundingBox_LoadSoup(BoundingBox *bb, const char *path, int octantWidth){
	
	// spatial partition this.
	
	PolySoup *soup = &bb->soup;

	int k;
	u16 stride = sizeof(Vec3);
	

	FILE *fp = fopen(path, "rb");

	fread(&soup->nTris, 1, sizeof(int), fp);

	int size = stride * soup->nTris * 3;

	Vec3 *verts = (Vec3 *)Memory_StackAlloc(TEMP_STACK, size);
	
	fread(verts, 1, size, fp);

	bb->cube.x = bb->cube.y = bb->cube.z = HUGE_VAL;
	bb->cube.w = bb->cube.h = bb->cube.d = -HUGE_VAL;
	for(k = 0; k < soup->nTris*3; k++){
		Vec3 *pos = &verts[k];

	     if(pos->x < bb->cube.x)
	         bb->cube.x = pos->x;
	     if(pos->x > bb->cube.w)
	         bb->cube.w = pos->x;
	     if(pos->y < bb->cube.y)
	         bb->cube.y = pos->y;
	     if(pos->y > bb->cube.h)
	         bb->cube.h = pos->y;
	     if(pos->z < bb->cube.z)
	         bb->cube.z = pos->z;
	     if(pos->z >  bb->cube.d)
	         bb->cube.d = pos->z;
	 }

	 bb->cube.w -= bb->cube.x;
	 bb->cube.h -= bb->cube.y;
	 bb->cube.d -= bb->cube.z;

	bb->scale = (Vec3){1,1,1};
	bb->rot = (Vec3){0,0,0};
	bb->pos = (Vec3){0,0,0};
	bb->wsCube = bb->cube;
	 fclose(fp);
	
	
	BoundingBox_UpdatePoints(bb);

	int divisions = roundf(log2(MAX(MAX(bb->cube.w,bb->cube.h),bb->cube.d) / octantWidth));

	memset(&soup->root, 0, sizeof(PolySoupLeaf));

	soup->root.pos = bb->pos;
	soup->root.pos.x -= size/2;
	soup->root.pos.y -= size/2;
	soup->root.pos.z -= size/2;
	soup->root.width = size;
	soup->root.height = size;
	soup->root.depth = size;
	soup->root.parent = NULL;

	PolySoupLeaf_Init(&soup->root, 0, divisions);

	for(k = 0; k < soup->nTris; k++){
		PolySoupLeaf_Insert(&soup->root, verts[(k*3)], verts[(k*3)+1], verts[(k*3)+2]);
	}

	
	
	Memory_StackPop(1,TEMP_STACK);

}

float SAT_Collision(Vec3 *pointsA, Vec3 *pointsB, Vec3 *axesA, Vec3 *axesB, float *overlap, Vec3 *axis){

		Vec3 axes[] = {
			axesA[0],
			axesA[1],
			axesA[2],
			axesB[0],
			axesB[1],
			axesB[2],
			Math_Vec3Cross(axesA[0], axesB[0]),
			Math_Vec3Cross(axesA[0], axesB[1]),
			Math_Vec3Cross(axesA[0], axesB[2]),

			Math_Vec3Cross(axesA[1], axesB[0]),
			Math_Vec3Cross(axesA[1], axesB[1]),
			Math_Vec3Cross(axesA[1], axesB[2]),

			Math_Vec3Cross(axesA[2], axesB[0]),
			Math_Vec3Cross(axesA[2], axesB[1]),
			Math_Vec3Cross(axesA[2], axesB[2])
		};

		if(CheckCollision(axes, 15, pointsA,8,pointsB, 8, overlap, axis))
			return 1;


		return 0;
}

int BoundingBox_SATCollision(BoundingBox *bb1, BoundingBox *bb2, float *overlap, Vec3 *axis){

	return SAT_Collision(bb1->points, bb2->points, bb1->axes, bb2->axes, overlap, axis);
}

void BoundingBox_Copy(BoundingBox *into, BoundingBox *bb){

	memcpy(into, bb, sizeof(BoundingBox));

	into->children = NULL;

	into->children = (BoundingBox *)malloc(bb->numChildren*sizeof(BoundingBox));
	into->types = (char **)malloc(bb->nTypes*sizeof(char *));


	int k;
	for(k = 0; k < bb->nTypes; k++){
		
		into->types[k] = malloc(strlen(bb->types[k]) + 1);
		strcpy(into->types[k], bb->types[k]);
		into->types[k][strlen(bb->types[k])] = 0;
	}

	for(k = 0; k < bb->numChildren; k++){
		BoundingBox_Copy(&into->children[k], &bb->children[k]);
		into->children[k].parent = into;
	}
}

void BoundingBox_AddType(BoundingBox *bb, char *name){
	
	if(!strlen(name)) return;

	bb->types = (char **)realloc(bb->types, sizeof(char *) * ++bb->nTypes);
	bb->types[bb->nTypes-1] = malloc(strlen(name) + 1);
	memset(bb->types[bb->nTypes-1], 0, strlen(name) + 1);
	strcpy(bb->types[bb->nTypes-1], name);
}

void BoundingBox_FreeData(BoundingBox *bb){
	int k;

	if(bb->types){

		for(k = 0; k < bb->nTypes; k++)
			free(bb->types[k]);
		
		free(bb->types);
	}

	if(bb->children){
		
		for(k = 0; k < bb->numChildren; k++)
			BoundingBox_FreeData(&bb->children[k]);
		
		free(bb->children);
	}

	memset(bb, 0, sizeof(BoundingBox));

	if(bb->parent){
		
		for(k = 0; k < bb->parent->numChildren; k++){
			if(&bb->parent->children[k] == bb){
				
				int j;
				for(j = k; j < bb->parent->numChildren-1; j++)
					bb->parent->children[j] = bb->parent->children[j+1];
			
				bb->parent->children = (BoundingBox *)realloc(bb->parent->children, 
					sizeof(BoundingBox) * --bb->parent->numChildren);
			
				break;
			}

		}
	}
}

Vec3 BoundingBox_GetPosition(BoundingBox *bb){

	return Math_CubeXYZ(bb->wsCube);
}

BoundingBox *BoundingBox_GetTop(BoundingBox *bb){

	if(bb->parent)
		return BoundingBox_GetTop(bb->parent);

	return bb;
}

BoundingBox BoundingBox_Create(Cube cube, Vec3 pos){

	BoundingBox bb;
	memset(&bb, 0, sizeof(BoundingBox));
	bb.radius = HUGE_VAL;
	bb.cube = cube;
	bb.pos = pos;
	BoundingBox_UpdatePoints(&bb);
	return bb;
}

void BoundingBox_Rotate(BoundingBox *bb, Vec3 rot){
	bb->rot = rot;
	BoundingBox_UpdatePoints(bb);
}

void BoundingBox_Scale(BoundingBox *bb, Vec3 scale){
	bb->scale = scale;
	BoundingBox_UpdatePoints(bb);
}

void BoundingBox_SetPos(BoundingBox *bb, Vec3 pos){
	bb->pos = pos;
	BoundingBox_UpdatePoints(bb);
}

int BoundingBox_AddChild(BoundingBox *bb, BoundingBox *child) {

	int index = bb->numChildren;

	if(bb->children == NULL)
		bb->children = (BoundingBox *)malloc((++bb->numChildren)*sizeof(BoundingBox));
	else
		bb->children = (BoundingBox *)realloc(bb->children, (++bb->numChildren)*sizeof(BoundingBox));

	BoundingBox_Copy(&bb->children[index], child);
	bb->children[index].parent = bb;

	return index;
}

int BoundingBox_CheckCollision(BoundingBox *bb, BoundingBox *bb2){


	int ret = 0;
	if(!Math_CheckCollisionCube(bb->wsCube, bb2->wsCube)) return ret;

	int k;
	 for(k = 0; k < bb->numChildren; k++){
	 	ret += BoundingBox_CheckCollision(&bb->children[k], bb2);
	}
	 for(k = 0; k < bb2->numChildren; k++){
	 	ret += BoundingBox_CheckCollision(bb, &bb2->children[k]);
	}

	return ret+1;
}

int BoundingBox_ResolveCollision(Game *game, Object *obj1, BoundingBox *bb, Object *obj2, BoundingBox *bb2){

	// children not iterative. need checking for each pair

	if((bb->collisionFlag & COLLISIONFLAG_RADIUS) || (bb2->collisionFlag & COLLISIONFLAG_RADIUS)){
		
		float mag = Math_Vec3Magnitude(Math_Vec3SubVec3(bb2->pos, bb->pos));
		if(mag > bb2->radius+bb->radius) return 0; 

	} else {

		if(!Math_CheckCollisionCube(bb->wsCube, bb2->wsCube))
			return 0;
	}
	int ret = 0;
	
	int k;
	 for(k = 0; k < bb->numChildren; k++){
	 	ret += BoundingBox_ResolveCollision(game, obj1, &bb->children[k], obj2, bb2);
	}
	 for(k = 0; k < bb2->numChildren; k++){
	 	ret += BoundingBox_ResolveCollision(game, obj1, bb, obj2, &bb2->children[k]);
	}

	if((bb->collisionFlag & COLLISIONFLAG_NONE)|| (bb2->collisionFlag & COLLISIONFLAG_NONE)) return ret;

	Vec3 axis;
	float overlap = 0;

	if(!(bb->soup.verts) &&  
		!(bb2->soup.verts)){
		
		if(!(bb->collisionFlag & COLLISIONFLAG_RADIUS) && !(bb2->collisionFlag & COLLISIONFLAG_RADIUS)){
					
			if((BoundingBox_IsSAT(bb) || BoundingBox_IsSAT(bb2))){

				if(!BoundingBox_SATCollision(bb, bb2, &overlap, &axis)){
					return ret;	
				}
			} else {

				Vec3 axes[] = {
					(Vec3){1,0,0},
					(Vec3){0,1,0},
					(Vec3){0,0,1},
				};

				if(!CheckCollision(axes, 3, bb->points,8,bb2->points, 8, &overlap,&axis))
					return 0;
			}
		}
	}


	if(obj1){
		
		if(obj1->OnCollision) obj1->OnCollision(game, obj1, obj1, obj2, bb, bb2, axis, overlap);
		
		if(obj1->storeLastCollisions){
			obj1->lastCollisions = (Collision *)realloc(obj1->lastCollisions, sizeof(Collision) * ++obj1->nLastCollisions);
			obj1->lastCollisions[obj1->nLastCollisions-1] = (Collision){obj1, obj2, bb, bb2,axis,overlap};
		}
	}
	
	axis = Math_Vec3MultFloat(axis, -1);

	if(obj2){
		if(obj2->OnCollision) obj2->OnCollision(game,obj2, obj1, obj2, bb, bb2, axis, overlap);
		if(obj2->storeLastCollisions){
			obj2->lastCollisions = (Collision *)realloc(obj2->lastCollisions, sizeof(Collision) * ++obj2->nLastCollisions);
			obj2->lastCollisions[obj2->nLastCollisions-1] = (Collision){obj2, obj1, bb2, bb,axis,overlap};
		}
	}

	return ret+1;
}
void BoundingBox_UpdateWorldSpaceCube(BoundingBox *bb){
	bb->wsCube.x = bb->wsCube.y = bb->wsCube.z = HUGE_VAL;
	bb->wsCube.w = bb->wsCube.h = bb->wsCube.d = -HUGE_VAL;

	int k;
	for(k = 0; k < 8; k++){
	Vec3 pos = bb->points[k];
	    if(pos.x < bb->wsCube.x)
	        bb->wsCube.x = pos.x;
	    if(pos.x > bb->wsCube.w)
	        bb->wsCube.w = pos.x;
	    if(pos.y < bb->wsCube.y)
	        bb->wsCube.y = pos.y;
	    if(pos.y > bb->wsCube.h)
	        bb->wsCube.h = pos.y;
	    if(pos.z < bb->wsCube.z)
	        bb->wsCube.z = pos.z;
	    if(pos.z >  bb->wsCube.d)
	        bb->wsCube.d = pos.z;
	}

	bb->wsCube.w -= bb->wsCube.x;
	bb->wsCube.h -= bb->wsCube.y;
	bb->wsCube.d -= bb->wsCube.z;
}

void BoundingBox_UpdatePoints(BoundingBox *bb){

	bb->points[1] = (Vec3){bb->cube.x, bb->cube.y, bb->cube.z+bb->cube.d};
	bb->points[5] = (Vec3){bb->cube.x+bb->cube.w, bb->cube.y, bb->cube.z+bb->cube.d};
	bb->points[3] = (Vec3){bb->cube.x, bb->cube.y, bb->cube.z};
	bb->points[7] = (Vec3){bb->cube.x+bb->cube.w, bb->cube.y, bb->cube.z};
	bb->points[0] = (Vec3){bb->cube.x, bb->cube.y+bb->cube.h, bb->cube.z+bb->cube.d};
	bb->points[2] = (Vec3){bb->cube.x, bb->cube.y+bb->cube.h, bb->cube.z};
	bb->points[4] = (Vec3){bb->cube.x+bb->cube.w, bb->cube.y+bb->cube.h, bb->cube.z+bb->cube.d};
	bb->points[6] = (Vec3){bb->cube.x+bb->cube.w, bb->cube.y+bb->cube.h, bb->cube.z};

	Vec3 rot = bb->rot;

	if(bb->parent){
		rot = Math_Vec3AddVec3(bb->parent->rot,bb->rot);
	}

	Math_RotateMatrix(bb->rmatrix,rot);
	bb->axes[0] = (Vec3){1,0,0};
	bb->axes[1] = (Vec3){0,1,0};
	bb->axes[2] = (Vec3){0,0,1};
	bb->axes[0] = Math_Vec3Normalize(Math_MatrixMult(bb->axes[0], bb->rmatrix));
	bb->axes[1] = Math_Vec3Normalize(Math_MatrixMult(bb->axes[1], bb->rmatrix));
	bb->axes[2] = Math_Vec3Normalize(Math_MatrixMult(bb->axes[2], bb->rmatrix));


	Math_RotateMatrix(bb->rmatrix,bb->rot);

	Math_ScalingMatrixXYZ(bb->matrix, bb->scale);
	Math_MatrixMatrixMult(bb->matrix, bb->rmatrix, bb->matrix);	
	//if(bb->parent){
		//Math_MatrixMatrixMult(bb->rmatrix, bb->parent->rmatrix, bb->rmatrix); 
	//}

	float matrix[16];
	Math_TranslateMatrix(matrix, bb->pos);
	Math_MatrixMatrixMult(bb->matrix, matrix, bb->matrix);

	if(bb->parent){
		Math_MatrixMatrixMult(bb->rmatrix, bb->parent->rmatrix, bb->rmatrix); 
		Math_MatrixMatrixMult(bb->matrix, bb->parent->matrix, bb->matrix); 
	}

	bb->points[0] = Math_MatrixMult(bb->points[0], bb->matrix);
	bb->points[1] = Math_MatrixMult(bb->points[1], bb->matrix);
	bb->points[2] = Math_MatrixMult(bb->points[2], bb->matrix);
	bb->points[3] = Math_MatrixMult(bb->points[3], bb->matrix);
	bb->points[4] = Math_MatrixMult(bb->points[4], bb->matrix);
	bb->points[5] = Math_MatrixMult(bb->points[5], bb->matrix);
	bb->points[6] = Math_MatrixMult(bb->points[6], bb->matrix);
	bb->points[7] = Math_MatrixMult(bb->points[7], bb->matrix);

	BoundingBox_UpdateWorldSpaceCube(bb);

	int k;	
	if(bb->children){
		for(k = 0; k < bb->numChildren; k++)
			BoundingBox_UpdatePoints(&bb->children[k]);

	}
}

int BoundingBox_IsSAT(BoundingBox *bb){
	if(!(bb->collisionFlag & COLLISIONFLAG_SAT) ||
	(fabs(bb->axes[0].x) == 1 || fabs(bb->axes[1].y) == 1 || fabs(bb->axes[2].z) == 1)){
		return 0;
	}
	return 1;
}


static int SameSide(Vec3 p, Vec3 a, Vec3 b, Vec3 c){
	Vec3 cp0 = Math_Vec3Cross(Math_Vec3SubVec3(b,a),Math_Vec3SubVec3(p,a));
	Vec3 cp1 = Math_Vec3Cross(Math_Vec3SubVec3(b,a),Math_Vec3SubVec3(c,a));
	if(Math_Vec3Dot(cp0,cp1) >= 0) return 1;
	return 0;
}

 int BoundingBox_CheckCollisionRay(BoundingBox *bb, Ray r, BoundingBox **b, float *dist){
	
	float d = Math_CubeCheckCollisionRay(bb->wsCube, r);
	
	if(d < *dist ){
		if((bb->collisionFlag & COLLISIONFLAG_AABB) && !(bb->collisionFlag & COLLISIONFLAG_NONE)){
			*b = bb;
			*dist = d;
		}

		int k;
		for(k = 0; k < bb->numChildren; k++){
			BoundingBox_CheckCollisionRay(&bb->children[k], r, b, dist);
		}
	}
	return 1;
}