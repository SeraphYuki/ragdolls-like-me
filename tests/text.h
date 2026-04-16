#ifndef GRAPHICS_DEF
#define GRAPHICS_DEF

#include "types.h"
#include "freetype.h"


#define THOTH_MAX_TEXT_CHARS 10300

typedef struct {
	u8           fontSize;
// ncurses vao/vbos
	u32          stringOffset;
	u32          vao;
	u32          posVbo;
	u32          uvVbo;
	FontFace    fontTTF;
} FontRenderer;


u32 FontRenderer_FontWidth(FontRenderer *fr);
u32 FontRenderer_FontHeight(FontRenderer *fr);
void FontRenderer_Init(FontRenderer *fr,  int w, int h);
void FontRenderer_Clear(FontRenderer *fr);
void FontRenderer_Close(FontRenderer *fr);
void FontRenderer_Resize(FontRenderer *fr, int w, int h);
void FontRenderer_Render(FontRenderer *fr, int viewportW, int viewportH);
void FontRenderer_SetSize(FontRenderer *fr, float w, float h);
void FontRenderer_RenderString(FontRenderer *fr, float x, float y, char *str,
u8 r, u8 g, u8 b, u8 a);

#endif
