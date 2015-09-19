#ifndef ___TEXTURE_H__
#define ___TEXTURE_H__

#include <OpenGLES/ES2/glext.h>
#include "_basept.h"

class PTTexture
{
public:
    PTTexture(){};
    ~PTTexture(){};
    PTS32 createTexture(const void* Bufferpixels, int texWidth, int texHeight, GLenum RGB_Format = GL_RGB, GLenum DataType = GL_UNSIGNED_BYTE);
    PTS32 releaseTexture();

	void SetStateBind();

private:
	GLuint			m_uiTexture;
    GLuint          m_fboID;
	int				width;
	int				height;
	bool			fileFlag;//是否是通过load纹理来创建的对象
};
#endif

