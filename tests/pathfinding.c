#include "pathfinding.h"
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

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