#include "ui.h"
#include "shaders.h"
#include "freetype.h"
#include "text.h"
#include "types.h"
#ifdef WINDOWS_COMPILE
#define GLEW_STATIC
#endif
#include <GL/glew.h>

#define RENDER_VRAM_SIZE 1024*6
const static u8 RectTriangleVerts[] = {0,0,1,0,1,1,1,1,0,1,0,0};

static void CreateFrameBuffer(UI *ui){

	glGenFramebuffers(1,&ui->fb);
	glBindFramebuffer(GL_FRAMEBUFFER, ui->fb);

	glGenTextures(1, &ui->fbTexture);
	glBindTexture(GL_TEXTURE_2D, ui->fbTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ui->viewport.w, ui->viewport.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ui->fbTexture, 0);

	glDrawBuffer(GL_COLOR_ATTACHMENT0);

	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE){
		glDeleteFramebuffers(1, &ui->fb);
		glDeleteTextures(1, &ui->fbTexture);
	}

	glViewport(0, 0, ui->viewport.w, ui->viewport.h);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void UI_Init(UI *ui, int w, int h){

	ui->stress = 0;
	ui->viewport.w = w;
	ui->viewport.h = h;
	ui->uiImg = ImageLoader_CreateImage("Resources/ui.png", 1);

	FontRenderer_Init(&ui->fr, w, h);	

	glGenVertexArrays(1, &ui->quadVao);
	glBindVertexArray(ui->quadVao);

	glGenBuffers(1, &ui->quadVbo);
	glBindBuffer(GL_ARRAY_BUFFER, ui->quadVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(RectTriangleVerts), RectTriangleVerts, GL_STATIC_DRAW);
	glEnableVertexAttribArray(POS_LOC);
	glVertexAttribPointer(POS_LOC, 2, GL_UNSIGNED_BYTE, GL_FALSE, 0, 0);

	// textured/textureless vao/vbos

	glGenVertexArrays(1, &ui->vao);
	glBindVertexArray(ui->vao);

	glGenBuffers(1, &ui->posVbo);
	glBindBuffer(GL_ARRAY_BUFFER, ui->posVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(u16)*2*RENDER_VRAM_SIZE, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(POS_LOC);
	glVertexAttribPointer(POS_LOC, 2, GL_SHORT, GL_FALSE, 0, 0);

	glGenBuffers(1, &ui->uvVbo);
	glBindBuffer(GL_ARRAY_BUFFER, ui->uvVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*RENDER_VRAM_SIZE, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(UV_LOC);
	glVertexAttribPointer(UV_LOC, 2, GL_FLOAT, GL_FALSE, 0, 0);

	// init framebuffer
	
	CreateFrameBuffer(ui);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void UI_Free(UI *ui){

	glDeleteVertexArrays(1, &ui->quadVao);
	glDeleteBuffers(1, &ui->quadVbo);

	glDeleteVertexArrays(1, &ui->vao);
	glDeleteBuffers(1, &ui->uvVbo);
	glDeleteBuffers(1, &ui->posVbo);
}

void UI_ViewportXY(UI *ui, int x, int y){
	ui->viewport.x = x;
	ui->viewport.y = y;
}

void UI_Resize(UI *ui, int w, int h){
	glBindTexture(GL_TEXTURE_2D, ui->fbTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	ui->viewport.w = w;
	ui->viewport.h = h;
}

void UI_Render(UI *ui){

	int x = 0, y = 0;	
	glBlendFunc(GL_ONE,GL_ONE);
	UI_RenderRectTex(ui, ui->uiImg, x,y,(u16)(438.0f * ui->stress),20,0,0,(u16)(430.0f * ui->stress),19,255,255,255,180);
	glBlendFunc(GL_ONE,GL_ONE_MINUS_SRC_ALPHA);

	FontRenderer_SetSize(&ui->fr,16);
	FontRenderer_RenderString(&ui->fr, x+36, y, "stress",0,0,0,255);
	
	glBindFramebuffer(GL_FRAMEBUFFER, ui->fb);
	FontRenderer_Render(&ui->fr, ui->viewport.w, ui->viewport.h);
	glBlendFunc(GL_ONE,GL_ONE);
	UI_RenderRectTex(ui, ui->uiImg, x,y,(u16)(438.0f * ui->stress),20,0,20,(u16)(430.0f * ui->stress),20,0,0,0,40);
	glBlendFunc(GL_ONE,GL_ONE_MINUS_SRC_ALPHA);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glBindVertexArray(ui->quadVao);

	glViewport(ui->viewport.x, ui->viewport.y, ui->viewport.w, ui->viewport.h);

	Shaders_UseProgram(QUAD_SHADER);

	glUniform2f(Shaders_GetInvViewportLocation(), 1.0f, 1.0f);

	glCullFace(GL_FRONT);
	glBindTexture(GL_TEXTURE_2D, ui->fbTexture);
	glDrawArrays(GL_TRIANGLES, 0, 6);
}

void UI_Clear(UI *ui){
	glBindFramebuffer(GL_FRAMEBUFFER, ui->fb);
	glViewport(ui->viewport.x, ui->viewport.y, ui->viewport.w, ui->viewport.h);

	glClearColor(0,0,0,0);

	glClear(GL_COLOR_BUFFER_BIT);
	glCullFace(GL_BACK);
}

void UI_RenderRect(UI *ui, float x, float y, u16 w, u16 h, u8 r, u8 g, u8 b, u8 a){
	
	glBindFramebuffer(GL_FRAMEBUFFER, ui->fb);
	glBindVertexArray(ui->vao);

	u32 offset = 0;
	u16 pos[2];

	u32 k;
	for(k = 0; k < 12; k+=2){
		pos[0] = x + ((s16)RectTriangleVerts[k] * w);
		pos[1] = y + ((s16)RectTriangleVerts[k+1] * h);

		glBindBuffer(GL_ARRAY_BUFFER, ui->posVbo);
		glBufferSubData(GL_ARRAY_BUFFER, offset*sizeof(pos), sizeof(pos), pos);
		 glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(pos), pos);
	
		++offset;
	}

	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);

	Shaders_UseProgram(TEXTURELESS_2D_SHADER);
	glUniform4f(Shaders_GetUniColorLocation(), r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f); 
	glUniform2f(Shaders_GetInvViewportLocation(), 1.0f/ui->viewport.w, 1.0f/ui->viewport.h); 

	glBindVertexArray(ui->vao);
	glBindBuffer(GL_ARRAY_BUFFER, ui->posVbo);
	
	glDisableVertexAttribArray(UV_LOC);
	
	glDrawArrays(GL_TRIANGLES, 0, 6);
	
	glEnableVertexAttribArray(UV_LOC);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
}

void UI_RenderRectTex(UI *ui, Image img, float x, float y, u16 w, u16 h, 
u16 ix, u16 iy, u16 iw, u16 ih, u8 r, u8 g, u8 b, u8 a){

	glBindFramebuffer(GL_FRAMEBUFFER, ui->fb);
	glBindVertexArray(ui->vao);

	u32 offset = 0;
	u16 pos[2];
	float coord[2];
	u32 k;
	for(k = 0; k < 12; k+=2){
		pos[0] = x + ((s16)RectTriangleVerts[k] * w);
		pos[1] = y + ((s16)RectTriangleVerts[k+1] * h);
		coord[0] = (ix + ((s16)RectTriangleVerts[k] * iw)) * 1.0f/img.w;
		coord[1] = 1-((iy + ((s16)RectTriangleVerts[k+1] *ih)) * 1.0f/img.h);
		glBindBuffer(GL_ARRAY_BUFFER, ui->posVbo);
		glBufferSubData(GL_ARRAY_BUFFER, offset*sizeof(pos), sizeof(pos), pos);
		glBindBuffer(GL_ARRAY_BUFFER, ui->uvVbo);
		glBufferSubData(GL_ARRAY_BUFFER, offset*sizeof(coord), sizeof(coord), coord);
	
		++offset;
	}

	glDisable(GL_DEPTH_TEST);

	Shaders_UseProgram(TEXTURED_2D_SHADER);
	glUniform4f(Shaders_GetUniColorLocation(), r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f); 
	glUniform2f(Shaders_GetInvViewportLocation(), 1.0f/ui->viewport.w, 1.0f/ui->viewport.h); 

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, img.glTexture);

	glDrawArrays(GL_TRIANGLES, 0, 6);
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glEnable(GL_DEPTH_TEST);
	
}