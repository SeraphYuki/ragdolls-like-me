#ifndef TEXT_DEF
#define TEXT_DEF


typedef struct {
    fontFace;
    unsigned int fontTexture;


} FontFace;

int FontFace_LoadFont(FontFace *font, const char *path);
int FontFace_SetSize(FontFace *font, int size);

#endif