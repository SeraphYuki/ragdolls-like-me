#ifndef BOUNDING_BOX_DEF
#define BOUNDING_BOX_DEF

#include "math.h"

typedef struct Game Game;

#define COLLISIONFLAG_NONE 0x01
#define COLLISIONFLAG_AABB 0x02
#define COLLISIONFLAG_SAT 0x04
#
typedef struct Object Object;

typedef struct _BoundingBox {
	struct _BoundingBox *parent;
	struct _BoundingBox *children;
	short numChildren;
	Cube cube;
	Cube wsCube;
	float matrix[16];
	float rmatrix[16];
	Vec3 points[8];
	Vec3 axes[3];
	Vec3 initialAxes[3];
	Vec3 pos;
	Vec3 rot;
	Vec3 scale;
	Vec3 absPos;
	Vec3 absRot;
	Vec3 absScale;
	int collisionFlag;
	char **types;
	int nTypes;
	int renderDebug;
} BoundingBox;


int BoundingBox_IsSAT(BoundingBox *bb);

int BoundingBox_CheckCollision(BoundingBox *bb, BoundingBox *bb2);

void BoundingBox_AddType(BoundingBox *bb, char *name);

void BoundingBox_Rotate(BoundingBox *bb, Vec3 rot);
void BoundingBox_Scale(BoundingBox *bb, Vec3 scale);
void BoundingBox_SetPos(BoundingBox *bb, Vec3 pos);
void BoundingBox_UpdatePoints(BoundingBox *bb);
void BoundingBox_UpdateWorldSpaceCube(BoundingBox *bb);

BoundingBox BoundingBox_Create(Cube cube, Vec3 pos);

int BoundingBox_AddChild(BoundingBox *bb, BoundingBox *child);

float SAT_Collision(Vec3 *pointsA, Vec3 *pointsB, Vec3 *axesA, Vec3 *axesB, float *overlap, Vec3 *axis);
void BoundingBox_FreeData(BoundingBox *bb);

void BoundingBox_Copy(BoundingBox *into, BoundingBox *bb);

int BoundingBox_SATCollision(BoundingBox *bb1, BoundingBox *bb2, float *overlap, Vec3 *axis);
int BoundingBox_ResolveCollision(Game *game, Object *obj1, BoundingBox *bb, Object *obj2, BoundingBox *bb2);

Vec3 BoundingBox_GetPosition(BoundingBox *bb);

 int BoundingBox_CheckCollisionRay(BoundingBox *bb, Ray r, BoundingBox **b, float *dist);

BoundingBox *BoundingBox_GetTop(BoundingBox *bb);

#endif
