#ifndef PATHFINDING_DEF
#define PATHFINDING_DEF

#define MAX_PATHFINDING_NODES 1024

typedef struct {
		int g;
		int f;
		int index;
} AStarNode;

typedef struct {
	AStarNode open[MAX_PATHFINDING_NODES];
	int nOpen;
	AStarNode closed[MAX_PATHFINDING_NODES];
	int nClosed;
	int w;
	int h;
} Pathfinder;

int Pathfinding_FindPath(Pathfinder *pf, int x, int y, int gx, int gy);
void Pathfinding_SetClosed(Pathfinder *pf, int x, int y);
void Pathfinding_Init(Pathfinder *pf, int w, int h);
void Pathfinding_SetClosed(Pathfinder *pf, int x, int y);
int Pathfinding_FindPath(Pathfinder *pf, int x, int y, int tx, int ty);

#endif