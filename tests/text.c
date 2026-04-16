#ifdef WINDOWS_COMPILE
#define GLEW_STATIC
#endif
#include <GL/glew.h>
#include <stdio.h>
#include "types.h"
#include "shaders.h"
#include "text.h"
#include "log.h"
#include "window.h"
#include "freetype.h"

#define MINFONTSIZE 6
#define MAXFONTSIZE 70
#define FONT_SIZE_BITS 4
#define FONT_SIZE_MASK ((1 << FONT_SIZE_BITS)-1)
#define FONTSIZE 32
#define MAXTEXTHEIGHT 130
#define MAXTEXTWIDTH 130
#define RENDER_VRAM_SIZE THOTH_MAX_TEXT_CHARS*6// idk most text characters on screen possible
#define TAB_SPACING 4
#define SS_CHAR_SIZE 0.0625f

const static u8 RectTriangleVerts[] = {0,0,1,0,1,1,1,1,0,1,0,0};

void FontRenderer_Init(FontRenderer *fr, int w, int h){

	memset(fr, 0, sizeof(FontRenderer));
	fr->fontSize = FONTSIZE;


	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_POLYGON_SMOOTH);
		
	
	Text_Init();
	
	 //textures

	 //Utils_LoadImage(&fr->font_g, FONT_PATH);
	FontFace_LoadFont(&fr->fontTTF, "Resources/font.ttf");
	FontFace_SetSize(&fr->fontTTF, fr->fontSize);
	 //int k;
	 //int x = 0;
	 //for(k = 0; k < 128; k++){
	     //fr->fontTTF.fontCharacters[k] = {
	         //.ax = 0,
	         //.ay = 0,
	         //.bw = fr->fontSize,
	         //.bh = fr->fontSize,
	         //.bl = 0,
	         //.bt = 0,
	         //.tx = (float)x%16 * fr->fontTTF.atlasWidth;
	         //.ty = (float)x/16 * fr->fontTTF.atlasHeight;
	     //}
	     //x += fr->bitmap.width;
	 //}

	glGenVertexArrays(1, &fr->vao);
	glBindVertexArray(fr->vao);

	glGenBuffers(1, &fr->posVbo);
	glBindBuffer(GL_ARRAY_BUFFER, fr->posVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(u16)*2*RENDER_VRAM_SIZE, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(POS_LOC);
	glVertexAttribPointer(POS_LOC, 2, GL_SHORT, GL_FALSE, 0, 0);

	glGenBuffers(1, &fr->uvVbo);
	glBindBuffer(GL_ARRAY_BUFFER, fr->uvVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*RENDER_VRAM_SIZE, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(UV_LOC);
	glVertexAttribPointer(UV_LOC, 2, GL_FLOAT, GL_FALSE, 0, 0);
}

void FontRenderer_Close(FontRenderer *fr){

	FontFace_Delete(&fr->fontTTF);
	Text_Close();


	glDeleteVertexArrays(1, &fr->vao);
	glDeleteBuffers(1, &fr->uvVbo);
	glDeleteBuffers(1, &fr->posVbo);

}

void FontRenderer_Zoom(FontRenderer *fr, int by){
	if(by < 0 && fr->fontSize > MINFONTSIZE) fr->fontSize += by;
	if(by > 0 && fr->fontSize < MAXFONTSIZE) fr->fontSize += by;
	FontFace_SetSize(&fr->fontTTF, fr->fontSize);
}


void FontRenderer_SetFontSize(FontRenderer *fr, u8 fs){
	fr->fontSize = fs;
}

void FontRenderer_RenderString(FontRenderer *fr, float x, float y, char *str,
	u8 r, u8 g, u8 b, u8 a){

	Shaders_UseProgram(TEXT_2D_SHADER);
	glUniform4f(Shaders_GetUniColorLocation(), r/255.0f,g/255.0f,b/255.0f,a/255.0f); 

	int strLen = strlen(str);

	float fontSizeX = fr->fontSize;//FontRenderer_FontWidth(fr);
	float fontSizeY = fr->fontSize;//FontRenderer_FontHeight(fr);
	u32 k;

	y += fontSizeY;

	char p;

	int j;
	for(j = 0; j < strLen && fr->stringOffset < THOTH_MAX_TEXT_CHARS*6; j++){


		p = str[j];

		FontCharacter fc = fr->fontTTF.fontCharacters[(int)p];

		float x2 = x + fc.bl;
		float y2 = y - fc.bt;
		float w = fc.bw;
		float h = fc.bh;


		x += fontSizeX;

		if(!w || !h) continue;

		float tX = fc.tx;
		float tY = 0;

		float ah = fr->fontTTF.atlasHeight;
		float aw = fr->fontTTF.atlasWidth;
	
		float coord[2];
		short pos[2];

		for(k = 0; k < 12; k+=2){

				coord[0] = ((RectTriangleVerts[k] * (fc.bw / aw)) + tX);
				coord[1] = (((1-RectTriangleVerts[k+1]) * ((float)fc.bh / ah)) + tY);

				pos[0] = (RectTriangleVerts[k] * w) + x2;
				pos[1] = ((1-RectTriangleVerts[k+1]) * h) + y2;
				glBindBuffer(GL_ARRAY_BUFFER, fr->posVbo);
				glBufferSubData(GL_ARRAY_BUFFER, fr->stringOffset*sizeof(pos), sizeof(pos), pos);
				glBindBuffer(GL_ARRAY_BUFFER, fr->uvVbo);
				glBufferSubData(GL_ARRAY_BUFFER, fr->stringOffset*sizeof(coord), sizeof(coord), coord);

				++fr->stringOffset;
		}
	}

}

void FontRenderer_Render(FontRenderer *fr, int viewportW, int viewportH){

	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);

	Shaders_UseProgram(TEXT_2D_SHADER);
	glUniform2f(Shaders_GetInvViewportLocation(), 1.0f/viewportW, 1.0f/viewportH); 
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, fr->fontTTF.fontTexture);

	glBindVertexArray(fr->vao);

	glCullFace(GL_FRONT);
	glDrawArrays(GL_TRIANGLES, 0, fr->stringOffset);

	fr->stringOffset = 0;

}
