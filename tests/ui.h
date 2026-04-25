#ifndef UI_DEF
#define UI_DEF

#define MAX_UI_ELEMENTS 1024

#include "types.h"
#include "window.h"
#include "image_loader.h"
#include "math.h"
#include "text.h"
typedef struct UI UI;

enum {
	UI_TYPE_SLIDER = 1,
};

typedef struct {
	int dragging;
	Rect2D sliderrect;	
	Rect2D dragrect;
	float value;
} UI_Slider;

typedef struct UI_Element UI_Element;
struct UI_Element {
	int type;
	Rect2D rect;
	void (*Event)(UI *ui, UI_Element *this, SDL_Event ev);
	void (*Update)(UI *ui, UI_Element *this);
	void (*Render)(UI *ui, UI_Element *this);
	void (*Free)(UI *ui, UI_Element *this);
	void *data;
};


struct UI {
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
	Image uiImg;
	FontRenderer fr;
	float stress;
	UI_Element elements[MAX_UI_ELEMENTS];
	int nElements;
	int leftMouseDown;
	int rightMouseDown;
	Rect2D mouserect;
	float sliderValue;
	float sliderValue2;
};


void UI_Clear(UI *ui);
void UI_RenderRect(UI *ui, float x, float y, u16 w, u16 h, u8 r, u8 g, u8 b, u8 a);
void UI_RenderRectTex(UI *ui, Image img, float x, float y, u16 w, u16 h, u16 ix, u16 iy,
 u16 iw, u16 ih, u8 r, u8 g, u8 b, u8 a);
void UI_Init(UI *ui, int w, int h);
void UI_Resize(UI *ui, int w, int h);
void UI_Render(UI *ui);
void UI_Free(UI *ui);
void UI_Event(UI *ui, SDL_Event ev);

#endif