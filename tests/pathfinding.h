
#ifndef PATHFINDING_DEF
#define PATHFINDING_DEF
#define MAX_PATHFINDING_PATH 40
#define PATHFINDING_NODE_GRID_SIZE 0.6
#include "math.h"
#include "bounding_box.h"

typedef struct AStarNode AStarNode;
struct AStarNode {
		int g;
		int f;
		int index;
		AStarNode *parent;
		AStarNode *next;
		AStarNode *prev;
};

typedef struct {
	Vec3 path[MAX_PATHFINDING_PATH];
	int pathIndicies[MAX_PATHFINDING_PATH];
	int nPath;	
} PathfinderPath;

typedef struct {
	int w;
	int h;
	AStarNode *openFirst;
	AStarNode *closedFirst;
	AStarNode *closedObstaclesLast;
	AStarNode *closedStaticLast;
	Vec3 *closedVerts;
	int ebo;
	int vao;
	int vbo;
	int nVerts;
	Cube cube;
	Vec3 pos;
} Pathfinder;

void Pathfinding_LoadNavMesh(Pathfinder *pf, const char *path);
void Pathfinding_LoadNavGrid(Pathfinder *pf, const char *path);
int Pathfinding_FindPathGrid(Pathfinder *pf, Vec3 pos, Vec3 goal,PathfinderPath *path);
void Pathfinding_SetClosedGrid(Pathfinder *pf, Vec3 pos);
int Pathfinding_FindPath(Pathfinder *pf, Vec3 pos, Vec3 goal, PathfinderPath *path);
void Pathfinding_Init(Pathfinder *pf, int w, int h);
void Pathfinding_SetOpenGrid(Pathfinder *pf, Vec3 pos);
void Pathfinding_SetClosedBoundingBoxStatic(Pathfinder *pf, BoundingBox *bb);
void Pathfinding_SetClosedBoundingBoxDynamic(Pathfinder *pf, BoundingBox *bb);
void Pathfinding_SetClosedStatic(Pathfinder *pf, int x, int y);
void Pathfinding_SetClosedDynamic(Pathfinder *pf, int x, int y);
void Pathfinding_RenderDebug(Pathfinder *pf, PathfinderPath *path);
void Pathfinding_ClearDynamic(Pathfinder *pf);
#endif