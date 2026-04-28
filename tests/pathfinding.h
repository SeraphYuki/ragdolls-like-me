
#ifndef PATHFINDING_DEF
#define PATHFINDING_DEF

#define MAX_PATHFINDING_NODES 4096
#define PATHFINDING_NODE_GRID_SIZE 0.6

#include "math.h"
#include "bounding_box.h"
typedef struct AStarNode AStarNode;
struct AStarNode {
		float g;
		float f;
		int index;
		AStarNode *parent;
} ;

typedef struct {
	
} PathfindingPortal;

typedef struct PathfindingTri PathfindingTri;

struct PathfindingTri{
	Vec3 points[3];
	Vec3 centroid;
	float radius;
	PathfindingTri *neighbors[MAX_PATHFINDING_NODES];
	int nNeighbors;
	Line portals[MAX_PATHFINDING_NODES];
	int nPortals;
	int index;
};

typedef struct {
	int w;
	int h;
	AStarNode open[MAX_PATHFINDING_NODES];
	AStarNode closed[MAX_PATHFINDING_NODES];
	AStarNode nodes[MAX_PATHFINDING_NODES];
	Vec3 path[MAX_PATHFINDING_NODES];
	Vec3 closedVerts[MAX_PATHFINDING_NODES];
	int pathIndicies[MAX_PATHFINDING_NODES];
	Line channel[MAX_PATHFINDING_NODES];
	int nChannel;						
	int nPath;
	int nOpen;
	int nClosed;
	int nClosedObstacles;
	int ebo;
	int vao;
	int vbo;
	Vec3 verts[MAX_PATHFINDING_NODES];
	int nVerts;
	PathfindingTri tris[MAX_PATHFINDING_NODES];
	int nFaces;
	Cube cube;
	Vec3 pos;
} Pathfinder;

void Pathfinding_RenderDebug(Pathfinder *pf);
void Pathfinding_LoadNavMesh(Pathfinder *pf, const char *path);
void Pathfinding_LoadNavGrid(Pathfinder *pf, const char *path);
int Pathfinding_FindPathGrid(Pathfinder *pf, Vec3 pos, Vec3 goal);
void Pathfinding_SetClosedGrid(Pathfinder *pf, Vec3 pos);
void Pathfinding_SetClosed(Pathfinder *pf, int x, int y);
int Pathfinding_FindPath(Pathfinder *pf, Vec3 pos, Vec3 goal);
void Pathfinding_Init(Pathfinder *pf, int w, int h);
void Pathfinding_SetOpenGrid(Pathfinder *pf, Vec3 pos);
void Pathfinding_SetClosedBoundingBox(Pathfinder *pf, BoundingBox *bb);

#endif