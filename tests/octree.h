#ifndef OCTREE
#define OCTREE
#include "object.h"
#include "math.h"



typedef struct _OctreeLeaf {
	struct _OctreeLeaf *parent;
	struct _OctreeLeaf *children[8];
	Object **objects;
	int numObjects;
	Cube cube;
	float height;
	float width;
	float depth;
	Vec3 pos;
	int level;
} OctreeLeaf;

void OctreeLeaf_Init(OctreeLeaf *o, int index, int divisions);
int OctreeLeaf_Insert(OctreeLeaf *o, Object *obj);
void OctreeLeaf_Remove(OctreeLeaf *o, Object *obj);
void OctreeLeaf_MoveInto(OctreeLeaf *into, OctreeLeaf *oct);
void OctreeLeaf_ResolveCollisions(OctreeLeaf *o, Object *obj, BoundingBox *box, Cube minCube);
void OctreeLeaf_Clear(OctreeLeaf *o);
void OctreeLeaf_Free(OctreeLeaf *o);
void OctreeLeaf_ResolveCollisionsRay(OctreeLeaf *o, Ray ray, float *dist,  BoundingBox **closest);
void OctreeLeaf_GetVisibleObjects(OctreeLeaf *octant, Object ***ret, int *nObjects);

#endif