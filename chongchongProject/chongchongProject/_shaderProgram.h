#ifndef ___SHADER_PROGRAM_H__
#define ___SHADER_PROGRAM_H__

#include "_basept.h"

typedef struct 
{
	GLuint uiId;
	GLuint auiLoc[13];
}ShaderProgram;


#define ROTATE_90   1.57f
#define ROTATE_180  3.14159f
#define ROTATE_270  4.71f

//----------------------------------------Uniform------------------------------------------
enum E_2D_SHADER_UNIFORM {TDMVPMatrix = 0, TDColor, TDSampler, TDUniformCnt};

//----------------------------------------Attrib------------------------------------------
enum E_2D_SHADER_ATTRIB {TD_POSITION_ARRAY = 0, TD_TEXCOORD_ARRAY ,TDAttricCnt};


class PTShaderProgram
{
public :
    PTShaderProgram(){m_TexWidth = 0; m_TexHeight = 0;};
	~PTShaderProgram(){};
	PTBOOL  loadShaders();
	PTBOOL  createPrograms(float angle = 0.0f);
	void    releaseShaders();
	void    releasePrograms();
    void    resetTexSize(int height, int width);
	
    ShaderProgram   m_TwoDimeProgram;
    
private:
    
    PTS32 shaderLoadSourceFromMemory(const char* pszShaderCode,
                                    const GLenum Type,
                                    GLuint* const pObject,
                                    const char* const* aszDefineArray = 0, GLuint uiDefArraySize = 0);
    
    PTS32 createProgram(GLuint* const pProgramObject,
                        const GLuint VertexShader,
                        const GLuint FragmentShader,
                        const char** const pszAttribs,
                        const int i32NumAttribs);
    
    int     m_TexWidth;
    int     m_TexHeight;
    
	GLuint	m_TwoDimeVertShader;
	GLuint	m_TwoDimeFragShader;
};




#endif