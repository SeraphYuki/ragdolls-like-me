#ifndef IMAGE_LOADER
#define IMAGE_LOADER

typedef struct {
	char *pixels;
	int w;
	int h;
	unsigned int glTexture;
} Image;

void ImageLoader_SetImageParameters(Image img, int minFilter, int magFilter, int glTextureWrapS, int glTextureWrapT);
void ImageLoader_DeleteImage(Image *img);
Image ImageLoader_CreateImage(const char *path, char loadMipMaps);
void ImageLoader_BindImage(Image img);

void ImageLoader_Free();
#endif