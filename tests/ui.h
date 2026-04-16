#ifndef UI_DEF
#define UI_DEF

#include "types.h"
#include "image_loader.h"
#include "text.h"

typedef struct {
	struct {
		int x;
		int y;
		int w;
		int h;
	} viewport;
	unsigned int fbTexture;
	unsigned int uvVbo;
	unsigned int posVbo;
	unsigned int vao;
	unsigned int quadVao;
	unsigned int quadVbo;
	unsigned int fb;
	Image test;
	FontRenderer fr;
} UI;


void UI_Clear(UI *ui);
void UI_RenderRect(UI *ui, float x, float y, u16 w, u16 h, u8 r, u8 g, u8 b, u8 a);
void UI_RenderRectTex(UI *ui, float x, float y, u16 w, u16 h, u16 ix, u16 iy, u16 iw, u16 ih, u8 r, u8 g, u8 b, u8 a);
void UI_Init(UI *ui, int w, int h);
void UI_Resize(UI *ui, int w, int h);
void UI_Render(UI *ui);
void UI_Free(UI *ui);


#endif