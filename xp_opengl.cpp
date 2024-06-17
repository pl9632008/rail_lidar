#include "xp_opengl.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "xp_lidar.h"
#include "xp_pcl.h"
#include <stdio.h>
#include <wchar.h>

using namespace std;

wchar_t OpenGLStatus[100]={0};//右上角提示信息
wchar_t OpenGLHUD[100]={0};//HUD提示信息
int OpenGLStart=1;//刷新使能，linux需要关闭使能才能打开file窗口

//快捷键使能
int Key_V=1;//显示点云
int Key_L=0;//录制线路点云
int Key_T=1;//显示目标

int Key_G=0;//实时/全局线路使能,G=1仅位移超过1m才存储1次数据

int Key_B=0;//显示背景
int Key_N=1;//显示隧道线阶
int Key_C=0;//录制/回放时自动调整camera
int Key_R=1;//视角重置
int Key_Z=0;//旋转移动模型，+左键旋转，+右键移动
int Key_X=0;//z+x旋转模型另一轴
int Key_P=0;//显示fps
int Key_Space=1;//回放时播放和暂停
int Key_Shift=0;
int Key_Alt=0;
int Key_Ctrl=0;

glm::vec3 dotCamera;//摄像机
glm::vec3 Euler;//欧拉角

//着色器
#if defined __APPLE__
static const char *vertexShader =
        "#version 330 core\n"
        "layout(location = 0) in vec3 vPosition;\n"
        "layout(location = 1) in vec4 vColor;\n"
        "uniform mat4 model;\n"
        "uniform mat4 view;\n"
        "uniform mat4 projection;\n"
        "out vec4 Color;\n"
        "void main()\n"
        "{\n"
        "gl_Position = projection * view * model * vec4(vPosition, 1.0);\n"
        "Color = vColor;\n"
        "}\n";
static const char *fragmentShader =
        "#version 330 core\n"
        "in highp vec4 Color;\n"
        "out highp vec4 FragColor;\n"
        "void main() {\n"
        "   FragColor = Color;\n"
        "}\n";

static const char *vertexShader1 =
        "#version 330 core\n"
        "layout(location = 0) in vec3 vPosition;\n"
        "layout(location = 1) in vec4 vColor;\n"
        "layout (location = 2) in vec2 aTexCoord;\n"
        "out vec2 TexCoord;\n"
        "uniform mat4 model;\n"
        "uniform mat4 view;\n"
        "uniform mat4 projection;\n"
        "out vec4 Color;\n"
        "void main()\n"
        "{\n"
        "gl_Position =projection*view*model*vec4(vPosition, 1.0);\n"
        "Color = vColor;\n"
        "TexCoord = aTexCoord;\n"
        "}\n";
static const char *fragmentShader1 =
        "#version 330 core\n"
        "in highp vec4 Color;\n"
        "in highp vec2 TexCoord;\n"
        "uniform sampler2D texture1;\n"
        "uniform sampler2D texture2;\n"
        "out highp vec4 FragColor;\n"
        "void main() {\n"
        "FragColor = mix(texture(texture1, TexCoord),texture(texture2, TexCoord),0.5);\n"
        "}\n";
#else
static const char *vertexShader =
        "#version 300 es\n"
        "layout(location = 0) in vec3 vPosition;\n"
        "layout(location = 1) in vec4 vColor;\n"
        "uniform mat4 model;\n"
        "uniform mat4 view;\n"
        "uniform mat4 projection;\n"
        "out vec4 Color;\n"
        "void main()\n"
        "{\n"
        "gl_PointSize = 2.0;\n"
        "gl_Position = projection * view * model * vec4(vPosition, 1.0);\n"
        "Color = vColor;\n"
        "}\n";//point_size
static const char *fragmentShader =
        "#version 300 es\n"
        "in highp vec4 Color;\n"
        "out highp vec4 FragColor;\n"
        "void main() {\n"
        "   FragColor = Color;\n"
        "}\n";

static const char *vertexShader1 =
        "#version 300 es\n"
        "layout(location = 0) in vec3 vPosition;\n"
        "layout(location = 1) in vec4 vColor;\n"
        "layout (location = 2) in vec2 aTexCoord;\n"
        "out vec2 TexCoord;\n"
        "uniform mat4 model;\n"
        "uniform mat4 view;\n"
        "uniform mat4 projection;\n"
        "out vec4 Color;\n"
        "void main()\n"
        "{\n"
        "gl_Position =projection*view*model*vec4(vPosition, 1.0);\n"
        "Color = vColor;\n"
        "TexCoord = aTexCoord;\n"
        "}\n";
static const char *fragmentShader1 =
        "#version 300 es\n"
        "in highp vec4 Color;\n"
        "in highp vec2 TexCoord;\n"
        "uniform sampler2D texture1;\n"
        "uniform sampler2D texture2;\n"
        "out highp vec4 FragColor;\n"
        "void main() {\n"
        "FragColor = mix(texture(texture1, TexCoord),texture(texture2, TexCoord),0.5);\n"
        "}\n";
#endif
xp_opengl::xp_opengl(QWidget *parent):QOpenGLWidget(parent)
{

#ifdef XP_PCL
    xp_Pcl=new xp_pcl;
#endif
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}

xp_opengl::~xp_opengl()
{
    for(int i=0;i<VAO.size();i++)
        ReleaseVABEO(i);
    for(int i=0;i<PROGRAM.size();i++)
        PROGRAM[i]=NULL;
    FT_Done_Face(face);
    FT_Done_FreeType(ft);
}

void xp_opengl::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);//只绘制第一层
    glEnable(GL_BLEND);//混合
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//混合模式
    glDepthFunc(GL_LESS);
    //    glEnable(GL_MULTISAMPLE);//抗锯齿
    printContextInformation();
    Genrate_Staff(20, 10, 0.0f);//生成标尺
    TextTextureInit();//文字 PROGRAM-VAO-VBO-VEO:0,0,0,0
    BuildProgram(vertexShader1,fragmentShader1);
    PrebuildVABEO([=]{//纹理 1,1,1,1
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
    });
    BuildProgram(vertexShader,fragmentShader);//点云shader
    PrebuildVABEO([=]{//标尺部分 2,2,2,NULL
        glVertexAttribPointer(0, 3,GL_FLOAT, GL_FALSE, sizeof(pointattrs),(void *)0);
        glVertexAttribPointer(1, 4,GL_FLOAT, GL_FALSE,sizeof(pointattrs),(void *)(4*sizeof(float)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
    });
    PrebuildVABEO([=]{//目标部分 2,3,3,NULL
        glVertexAttribPointer(0, 3,GL_FLOAT, GL_FALSE, sizeof(pointattrs),(void *)0);
        glVertexAttribPointer(1, 4,GL_FLOAT, GL_FALSE,sizeof(pointattrs),(void *)(4*sizeof(float)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
    });
    PrebuildVABEO([=]{//线阶部分 2,4,4,NULL
        glVertexAttribPointer(0, 3,GL_FLOAT, GL_FALSE, sizeof(pointattrs),(void *)0);
        glVertexAttribPointer(1, 4,GL_FLOAT, GL_FALSE,sizeof(pointattrs),(void *)(4*sizeof(float)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
    });
    PrebuildVABEO([=]{//点云部分 2,5,5,NULL
        glVertexAttribPointer(0, 3,GL_FLOAT, GL_FALSE, sizeof(pointattrs),(void *)0);
        glVertexAttribPointer(1, 4,GL_FLOAT, GL_FALSE,sizeof(pointattrs),(void *)(4*sizeof(float)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
    });
}

void xp_opengl::resizeGL(int w,int h)
{
    glprojection = glm::perspective(glm::radians(45.0f),(float)w/(h?h:1), 0.1f, 500.0f);//need limit range
}

void xp_opengl::PrebuildVABEO(std::function<void()> settings)
{
    GLuint vao;
    glGenVertexArrays(1,&vao);
    glBindVertexArray(vao);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    unsigned int ebo;
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    VBO.push_back(vbo);
    VAO.push_back(vao);
    EBO.push_back(ebo);
    settings();
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void xp_opengl::Draw(int program_index,int vabeo_index,std::function<void()> settings)
{
    glUseProgram(PROGRAM[program_index]);
    if(vabeo_index>=VAO.size() && vabeo_index>=VBO.size() && vabeo_index>=EBO.size())
        return;
    glBindVertexArray(VAO[vabeo_index]);
    glBindBuffer(GL_ARRAY_BUFFER,VBO[vabeo_index]);
    settings();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO[vabeo_index]);
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glUseProgram(0);
}

int xp_opengl::ReleaseVABEO(int vabeo_index)
{
    if(vabeo_index>=VAO.size() && vabeo_index>=VBO.size())
        return -1;
    glDeleteVertexArrays(1, &VAO[vabeo_index]);
    glDeleteBuffers(1, &VBO[vabeo_index]);
    return -1;
}

GLuint xp_opengl::BuildProgram(const char *vertex,const char *fragment)
{
    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertex, NULL);
    glCompileShader(vertexShader);
    int  success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragment, NULL);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    GLuint shaderProgram;
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if(!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glUseProgram(shaderProgram);
    PROGRAM.push_back(shaderProgram);
    return shaderProgram;
}

int xp_opengl::TextTextureInit()
{
#if defined __APPLE__
    const char *vertexShader =
            "#version 330 core\n"
            "layout(location = 0) in vec3 vPosition;\n"
            "layout (location = 1) in vec3 mycolor;\n"
            "layout (location = 2) in vec2 aTexCoord;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out vec2 TexCoord;\n"
            "out vec3 color;\n"
            "void main(){\n"
            "gl_Position =projection*view*model*vec4(vPosition, 1.0);\n"
            "TexCoord = aTexCoord;\n"
            "color=mycolor;\n"
            "}\n";
    const char *fragmentShader =
            "#version 330 core\n"
            "in highp vec2 TexCoord;\n"
            "in highp vec3 color;\n"
            "out highp vec4 FragColor;\n"
            "uniform sampler2D texture1;\n"
            "void main() {\n"
            "FragColor = vec4(color,texture(texture1, TexCoord).r);\n"
            "}\n";
#else
    const char *vertexShader =
            "#version 300 es\n"
            "layout(location = 0) in vec3 vPosition;\n"
            "layout (location = 1) in vec3 mycolor;\n"
            "layout (location = 2) in vec2 aTexCoord;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out vec2 TexCoord;\n"
            "out vec3 color;\n"
            "void main(){\n"
            "gl_Position =projection*view*model*vec4(vPosition, 1.0);\n"
            "TexCoord = aTexCoord;\n"
            "color=mycolor;\n"
            "}\n";
    const char *fragmentShader =
            "#version 300 es\n"
            "in highp vec2 TexCoord;\n"
            "in highp vec3 color;\n"
            "out highp vec4 FragColor;\n"
            "uniform sampler2D texture1;\n"
            "void main() {\n"
            "FragColor = vec4(color,texture(texture1, TexCoord).r);\n"
            "}\n";
#endif
    BuildProgram(vertexShader,fragmentShader);
    PrebuildVABEO([=]{
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
    });
    if (FT_Init_FreeType(&ft))
    {
        std::cout << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
        return -1;
    }
#ifdef _WIN32
    std::string str="C:\\WINDOWS\\Fonts\\simhei.ttf";
#elif defined __linux__
    std::string str="/usr/share/fonts/truetype/freefont/FreeMono.ttf";
#elif defined __APPLE__
    std::string str="/System/Library/Fonts/Avenir.ttc";
#endif
    if (FT_New_Face(ft, str.c_str(), 0, &face))
    {
        std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;
        return -1;
    }
    FT_Set_Pixel_Sizes(face, 0, 48);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1); //禁用字节对齐限制
    return 0;
}

//text:传入宽字符，使用L"XX"格式
//scale:缩放
//position:位置
//color:颜色
//WldorWin:是否绘制在空间中，为0则不会跟随视角变换
//offset:矩阵变换，旋转字符等高级操作
void xp_opengl::Text(std::wstring text,GLfloat scale, glm::vec3 position,glm::vec3 color,int WldorWin,glm::mat4 offset)
{
    static int state=0;
    std::wstring::const_iterator c;
    glUseProgram(PROGRAM[0]);
    //    _wsetlocale(LC_ALL, L"chs");
    GLfloat x_origin=0,y_origin=0;
    for (c = text.begin(); c != text.end(); c++)
    {
        if(Characters.count(*c)==0)
        {
            if (FT_Load_Char(face, *c, FT_LOAD_RENDER) && state==0)
            {
                state=1;
                continue;
            }
            GLuint texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(
                        GL_TEXTURE_2D,
                        0,
                        GL_RED,
                        face->glyph->bitmap.width,
                        face->glyph->bitmap.rows,
                        0,
                        GL_RED,
                        GL_UNSIGNED_BYTE,
                        face->glyph->bitmap.buffer
                        );
            // 设置纹理选项
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // 储存字符供之后使用
            Character character = {
                texture,
                glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
                glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
                (int)face->glyph->advance.x
            };
            Characters.insert(std::pair<int, Character>(*c, character));
        }
        Character ch = Characters[*c];
        GLfloat xpos =x_origin + ch.Bearing.x * scale;
        GLfloat ypos =y_origin - (ch.Size.y - ch.Bearing.y) * scale;
        GLfloat w = ch.Size.x * scale;
        GLfloat h = ch.Size.y * scale;
        GLfloat chvertices[6][8] = {
            { xpos,     ypos + h,0,color[0],color[1],color[2],0.0, 0.0 },
            { xpos,     ypos,    0,color[0],color[1],color[2],0.0, 1.0 },
            { xpos + w, ypos,    0,color[0],color[1],color[2],1.0, 1.0 },
            { xpos,     ypos + h,0,color[0],color[1],color[2],0.0, 0.0 },
            { xpos + w, ypos,    0,color[0],color[1],color[2],1.0, 1.0 },
            { xpos + w, ypos + h,0,color[0],color[1],color[2],1.0, 0.0 }
        };
        Draw(0,0,[=]{
            if(WldorWin==1)
            {
                glm::mat4 model = glm::mat4(1.0f);
                model*=offset;
                model = glm::translate(model, position);
                model = glm::rotate(model, glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
                glUniformMatrix4fv(glGetUniformLocation(PROGRAM[0],"model"),1,GL_FALSE,glm::value_ptr(model));
                glUniformMatrix4fv(glGetUniformLocation(PROGRAM[0],"view"),1,GL_FALSE,glm::value_ptr(glview));
                glUniformMatrix4fv(glGetUniformLocation(PROGRAM[0],"projection"),1,GL_FALSE,glm::value_ptr(glprojection));
            }
            else
            {
                glm::mat4 model = glm::mat4(1.0f);
                model = glm::translate(model, position);
                glUniformMatrix4fv(glGetUniformLocation(PROGRAM[0],"model"),1,GL_FALSE,glm::value_ptr(model));
                glUniformMatrix4fv(glGetUniformLocation(PROGRAM[0],"view"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
                glUniformMatrix4fv(glGetUniformLocation(PROGRAM[0],"projection"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
            }
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, ch.TextureID);
            glBufferData(GL_ARRAY_BUFFER, sizeof(chvertices), chvertices, GL_DYNAMIC_DRAW);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        });
        x_origin += (ch.Advance >> 6) * scale; // Bitshift by 6 to get value in pixels (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
}

GLuint xp_opengl::xp_LoadTextureFromImg(std::string path,GLenum format)
{
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    int width, height, nrChannels;
    stbi_set_flip_vertically_on_load(true);
    unsigned char *data = stbi_load(path.c_str(), &width, &height, &nrChannels, 0);
    if (data)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        stbi_image_free(data);
    }
    else
    {
        stbi_image_free(data);
        return 0;
    }
    return texture;
}

void xp_opengl::paintGL()
{
    //清除，写入，绘点
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    if(Key_C==1)//视角跟踪模式
    {
        cameraPos.x=INS_POSITION[0];
        cameraPos.y=INS_POSITION[1]-20;
        //        cameraPos.y=INS_POSITION[1]-30;
        //       pitch=-30.0f;
        cameraPos.z=30;
        pitch=-25.0f;
        yaw=90.0f;
    }
    if(Key_R==1)
    {
        cameraPos   = glm::vec3(0.0f,-5.5f, 2.24f);//相机位置
        cameraFront = glm::vec3(0.0f, 1.0f, 0.0f);//方向向量
        cameraUp    = glm::vec3(0.0f, 0.0f,  1.0f);//上向量，非头顶方向
        yaw = 90.0f;       //偏航角
        pitch = 0.0f;     //俯仰角
        Key_R=0;
        Key_C=0;
    }
    dotCamera=cameraPos;
    Euler=glm::vec3(pitch,yaw,0);
    glm::vec3 front;
    front.x = -cos(glm::radians(pitch)) * cos(glm::radians(yaw));
    front.z = sin(glm::radians(pitch));
    front.y = cos(glm::radians(pitch)) * sin(glm::radians(yaw));
    cameraFront = glm::normalize(front);
    glview=glm::lookAt(dotCamera, dotCamera + cameraFront, cameraUp);


    if(Key_B==0)//绘制标尺
    {
        static uint8_t loaded=0;
        Draw(2,2,[=]{
            //program属性每帧设定一次
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"model"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"view"),1,GL_FALSE,glm::value_ptr(glview));
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"projection"),1,GL_FALSE,glm::value_ptr(glprojection));
            if(loaded==0 && GPU_RULERS.size())
            {
                loaded++;
                glBufferData(GL_ARRAY_BUFFER,(GPU_RULERS.size()+6)*sizeof(pointattrs), &GPU_RULERS[0], GL_STATIC_DRAW);
            }
            glDrawArrays(GL_LINES, 0, GPU_RULERS.size()-4);//画标尺
            glDrawArrays(GL_TRIANGLE_STRIP ,GPU_RULERS.size()-4,4);//填充矩形颜色
            //雷达位置向量
            QVector<pointattrs> points=Genrate_DirVec();//生成方向向量
            glBufferSubData(GL_ARRAY_BUFFER,GPU_RULERS.size()*sizeof(pointattrs),6*sizeof(pointattrs), &points[0]);

        });
    }

    //pcl
    static QVector<pointattrs> GPU_PCD;
    if(g_lidarStatus == true && Key_V==1 && (GPU_PCD.size()||LIDAR_PCD.size()))//绘制PCL点云 ，如果只绘制目标，LIDAR_PCD换成LIDAR_PCDTARGET
    {
        if(LIDAR_PCD.size())
        {
            GPU_PCD.swap(LIDAR_PCD);
            LIDAR_PCD.clear();
        }
        static int already=0;//已分配显存状态指示
        static long int num=0;//点的总个数
        static int index=0;//帧索引
        static float movement=0;//位移
#ifndef ARM
        glPointSize(2);//定义点云大小
#endif
        Draw(2,5,[=]{
            Record record;
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"model"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"view"),1,GL_FALSE,glm::value_ptr(glview));
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"projection"),1,GL_FALSE,glm::value_ptr(glprojection));
            glm::mat4 model(1.0f);
            model=glm::translate(model,glm::vec3(lidar_offset,0.0f, lidar_height));
            model*=INS_OFST;
            model=glm::translate(model,glm::vec3(-lidar_offset,0.0f, -lidar_height));
#ifdef XP_PCL
            if (Key_G == 1 && Key_L == 1)//显示全局线路
#else
            if(0)
#endif
            {
                if(already==0)//第一次分配足够的显存
                {
                    already=1;
                    glBufferData(GL_ARRAY_BUFFER,GPU_MEMMAX,NULL, GL_STATIC_DRAW);
                }
                else
                {
                    if(fabs(INS_MOVEMENT-movement)>=1)//相对位移超过1m
                    {
                        movement=INS_MOVEMENT;
                        if((num+GPU_PCD.size())*sizeof(pointattrs)>GPU_MEMMAX)//大于显存分配上限，从头开始
                        {
                            index=0;
                            num=0;
                        }
                        //定义帧的相关数据
                        record.num=GPU_PCD.size();
                        record.offset=model;
                        record.start=num;
                        record.dis=(long int)movement;
                        records[index++]=record;
                        num+=GPU_PCD.size();
                        glBufferSubData(GL_ARRAY_BUFFER,num*sizeof(pointattrs),GPU_PCD.size()*sizeof(pointattrs), &GPU_PCD[0]);
                        int i=0;
                        while(records.count(index+i)==1)//从下一帧判断是否起始索引被新数据覆盖，覆盖了就丢弃这一帧
                        {
                            if((GPU_PCD.size()+num)>records[index+i].start)
                                records.erase(index+i);
                            i++;
                        }
                    }
                    map<int, Record>::iterator iter;
                    for(iter = records.begin(); iter != records.end(); iter++)//遍历绘图
                    {
                        glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"model"),1,GL_FALSE,glm::value_ptr(iter->second.offset));
                        glDrawArrays(GL_POINTS,iter->second.start,iter->second.num);
                    }
                }
            }
            else //实时模式
            {
                already=0;
                num=0;
                index=0;
                movement=0;
                records.clear();
                glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"model"),1,GL_FALSE,glm::value_ptr(model));
                glBufferData(GL_ARRAY_BUFFER,GPU_PCD.size()*sizeof(pointattrs),&GPU_PCD[0], GL_DYNAMIC_DRAW);
                glDrawArrays(GL_POINTS,0,GPU_PCD.size());
            }
        });
    }
    //targets 画目标
    //std::cout<<"GPU_TARGETS.size():"<<GPU_TARGETS.size()<<std::endl;
    if(g_lidarStatus == true && GPU_TARGETS.size() && Key_T==1)
    {
        Draw(2,3,[=]{
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"model"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"view"),1,GL_FALSE,glm::value_ptr(glview));
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"projection"),1,GL_FALSE,glm::value_ptr(glprojection));
            //长方体索引绘图
            //            unsigned int indices[]={3,7,1,5,0,4,2,6,0,1,2,3,6,7,4,5};
            unsigned int indices[]={3,7,1,5,0,4,2,6,0,1,2,3,6,7,4,5,0,2,1,3,5,7,4,6};
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_DYNAMIC_DRAW);
            for(int i=0;i<GPU_TARGETS.size();i+=8)
            {
                glBufferData(GL_ARRAY_BUFFER,8*sizeof(pointattrs), &GPU_TARGETS[i], GL_DYNAMIC_DRAW);
                //                glDrawElements(GL_TRIANGLE_STRIP, 8, GL_UNSIGNED_INT, 0);
                //                glDrawElements(GL_TRIANGLE_STRIP, 8, GL_UNSIGNED_INT,(const GLvoid *)32);//8*4,索引*字节
                glDrawElements(GL_LINES, 16, GL_UNSIGNED_INT, 0);
                glDrawElements(GL_LINES, 16, GL_UNSIGNED_INT,(const GLvoid *)32);//8*4,索引*字节
            }
            //显示距离信息，未用
#ifdef XP_DEBUG
            wchar_t buff[100];
            for(int i=7;i<GPU_TARGETS.size();i+=8)
            {
                if(Key_G==1 && Key_L==1)
                {
#if defined  __linux__ || defined __APPLE__
                    swprintf(buff,sizeof(buff)/sizeof(wchar_t),L"%.1f",GPU_TARGETS[i].rgba[3]);
#elif defined _WIN32
                    swprintf(buff,L"%.1f",GPU_TARGETS[i].rgba[3]);
#endif
                }
                else
                {
#if defined  __linux__ || defined __APPLE__
                    swprintf(buff,sizeof(buff)/sizeof(wchar_t),L"%.1f",GPU_TARGETS[i].xyz[1]);
#elif defined _WIN32
                    swprintf(buff,L"%.1f",GPU_TARGETS[i].xyz[1]);
#endif
                }
                Text(buff,0.01f,glm::vec3(GPU_TARGETS[i].xyz[0], GPU_TARGETS[i].xyz[1], GPU_TARGETS[i].xyz[2]),\
                        glm::vec3(1, 1, 1),1,glm::mat4(1.0f));
            }
#endif
        });
    }
    if(Key_N==1 && Edges.size()>1)
    {
        Draw(2,4,[=]{
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"model"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"view"),1,GL_FALSE,glm::value_ptr(glview));
            glUniformMatrix4fv(glGetUniformLocation(PROGRAM[2],"projection"),1,GL_FALSE,glm::value_ptr(glprojection));
            glBufferData(GL_ARRAY_BUFFER,10*Edges.size()*sizeof(Edge), NULL, GL_DYNAMIC_DRAW);
            //rails
            for(int i=0;i<Edges.size();i++)
            {
                glBufferSubData(GL_ARRAY_BUFFER,i*sizeof(pointattrs),sizeof(pointattrs),&Edges[i].rails[0]);
            }
            for(int i=0;i<Edges.size();i++)
            {
                glBufferSubData(GL_ARRAY_BUFFER,(Edges.size()+i)*sizeof(pointattrs),sizeof(pointattrs),&Edges[i].rails[1]);
            }
            //danger
            for(int i=0;i<Edges.size()*2;i++)
            {
                if(i%2==0)
                    glBufferSubData(GL_ARRAY_BUFFER,(Edges.size()*2+i)*sizeof(pointattrs),sizeof(pointattrs),&Edges[i/2].danger[0]);
                else
                    glBufferSubData(GL_ARRAY_BUFFER,(Edges.size()*2+i)*sizeof(pointattrs),sizeof(pointattrs),&Edges[i/2].danger[1]);
            }
            //warning
            for (auto j=0;j<(sizeof(Edge::warning)/sizeof(pointattrs)-1);j++)
            {
                for(int i=0;i<Edges.size()*2;i++)
                {
                    if(i%2==0)
                        glBufferSubData(GL_ARRAY_BUFFER,(Edges.size()*(4+j*2)+i)*sizeof(pointattrs),sizeof(pointattrs),&Edges[i/2].warning[j]);
                    else
                        glBufferSubData(GL_ARRAY_BUFFER,(Edges.size()*(4+j*2)+i)*sizeof(pointattrs),sizeof(pointattrs),&Edges[i/2].warning[j+1]);
                }
            }
            //rails
            glLineWidth(4);
            glDrawArrays(GL_LINE_STRIP ,0, Edges.size());
            glDrawArrays(GL_LINE_STRIP ,Edges.size(), Edges.size());
            //danger
            glDrawArrays(GL_TRIANGLE_STRIP ,2*Edges.size(), Edges.size()*2);
            //warning 隧道墙
            //            for (auto i=0;i<sizeof(Edge::warning)/sizeof(pointattrs)-1;i++)
            //                glDrawArrays(GL_TRIANGLE_STRIP ,(4+i*2)*Edges.size(), Edges.size()*2);
        });
    }
    //计算fps
    if(Key_P)
    {
        static long timetamps=clock();
        static long flushtimetamps=clock();
        static wchar_t tim_buff[100];
#if defined  __linux__ || defined __APPLE__
        if((clock()-flushtimetamps)>=100000 && (clock()-timetamps)>0)
#elif defined _WIN32
        if((clock()-flushtimetamps)>=1000 && (clock()-timetamps)>0)
#endif
        {
#if defined  __linux__ || defined __APPLE__
            swprintf(tim_buff,sizeof(tim_buff)/sizeof(wchar_t),L"%d",(int)(100000/(clock()-timetamps)));
#elif defined _WIN32
            swprintf_s(tim_buff,L"%d",(int)(1000/(clock()-timetamps)));
#endif
            flushtimetamps=clock();
        }
        timetamps=clock();
        Text(tim_buff,0.001f,glm::vec3(-1.0f,0.95f,0),glm::vec3(0, 1, 0),0);//绘制fps
    }
    if(OpenGLStatus[0]!=0)
        Text(OpenGLStatus,0.001f,glm::vec3(0.75f,0.95f,0),glm::vec3(1, 1, 1),0);//绘制右上角提示
    if(OpenGLHUD[0]!=0)
        Text(OpenGLHUD,0.002f,glm::vec3(-0.2f,0.8f,0),glm::vec3(1, 0, 0),0);//绘制hud提示
    if(OpenGLStart==1)
        update();
}

void xp_opengl::printContextInformation()
{
    QString glType;
    QString glVersion;
    QString glProfile;

    // Get Version Information
    glType = (context()->isOpenGLES()) ? "OpenGL ES" : "OpenGL";
    glVersion = reinterpret_cast<const char*>(glGetString(GL_VERSION));

    // Get Profile Information
#define CASE(c) case QSurfaceFormat::c: glProfile = #c; break
    switch (format().profile())
    {
    CASE(NoProfile);
    CASE(CoreProfile);
    CASE(CompatibilityProfile);
    }
#undef CASE

    // qPrintable() will print our QString w/o quotes around it.
    std::cout << qPrintable(glType) << qPrintable(glVersion) << "(" << qPrintable(glProfile) << ")"<<std::endl;
}
void xp_opengl::keyPressEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_W)
    {
        cameraPos += cameraSpeed * cameraFront;
    }
    else if(event->key()==Qt::Key_A)
    {
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    }
    else if(event->key()==Qt::Key_S)
    {
        cameraPos -= cameraSpeed * cameraFront;
    }
    else if(event->key()==Qt::Key_D)
    {
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    }
    else if(event->key()==Qt::Key_V)
    {
        if(!event->isAutoRepeat())
            Key_V=(~Key_V)&0x01;
    }
    else if(event->key()==Qt::Key_C)
    {
        if(!event->isAutoRepeat())
            Key_C=(~Key_C)&0x01;
    }
    else if(event->key()==Qt::Key_B)
    {
        if(!event->isAutoRepeat())
            Key_B=~Key_B&0x01;
    }
    else if(event->key()==Qt::Key_G)
    {
        if(!event->isAutoRepeat())
            Key_G=~Key_G&0x01;
    }
    else if(event->key()==Qt::Key_L)
    {
        if(!event->isAutoRepeat())
            Key_L=~Key_L&0x01;
    }
    else if(event->key()==Qt::Key_T)
    {
        if(!event->isAutoRepeat())
            Key_T=~Key_T&0x01;
    }
    else if(event->key()==Qt::Key_Space)
    {
        if(!event->isAutoRepeat())
            Key_Space=~Key_Space&0x01;
    }
    else if(event->key()==Qt::Key_N)
    {
        if(!event->isAutoRepeat())
            Key_N=~Key_N&0x01;
    }
    else if(event->key()==Qt::Key_R)
    {
        if(!event->isAutoRepeat())
            Key_R=~Key_R&0x01;
    }
    else if(event->key()==Qt::Key_P)
    {
        if(!event->isAutoRepeat())
            Key_P=~Key_P&0x01;
    }
    else if(event->key()==Qt::Key_X)
    {
        if(!event->isAutoRepeat())
            Key_X=1;
    }
    else if(event->key()==Qt::Key_Z)
    {
        if(!event->isAutoRepeat())
            Key_Z=1;
    }
    else if(event->key()==Qt::Key_Shift)
    {
        Key_Shift=~Key_Shift&0x01;
    }
    else if(event->key()==Qt::Key_Alt)
    {
        Key_Alt=(~Key_Alt)&0x01;
    }
    else if(event->key()==Qt::Key_Control)
        Key_Ctrl=1;
    //    qDebug()<<"down "<<event->key();
}

void xp_opengl::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_Alt)
        Key_Alt=0;
    if(event->key()==Qt::Key_Control)
        Key_Ctrl=0;
    else if(event->key()==Qt::Key_Shift)
        Key_Shift=0;
    else if(event->key()==Qt::Key_X)
    {
        if(!event->isAutoRepeat())
            Key_X=0;
    }
    else if(event->key()==Qt::Key_Z)
    {
        if(!event->isAutoRepeat())
            Key_Z=0;
    }
    //    qDebug()<<"up "<<event->key();
}
void xp_opengl::mouseMoveEvent(QMouseEvent *event)
{
    //设置光标形状
    cursor.setShape(Qt::ClosedHandCursor);
    setCursor(cursor);
    GLfloat xoffset = event->x() - lastX;
    GLfloat yoffset = event->y() - lastY;
    lastX = event->x();
    lastY = event->y();
    //鼠标左键用来实现对物体/视角的旋转功能
    if(Key_Z==0)
    {
        if (event->buttons() == Qt::LeftButton)
        {
            GLfloat sensitivity = 0.3f;     //旋转时的灵敏度
            yaw += xoffset*sensitivity;
            pitch += -yoffset*sensitivity;
            if(pitch>=360)
                pitch=0;
            if(yaw>=360)
                yaw=0;
            //可以用来设置俯仰角的上下界
            if (pitch > 89.0f)
                pitch = 89.0f;
            if (pitch < -89.0f)
                pitch = -89.0f;
        }
        else if (event->buttons() == Qt::RightButton)
        {
            GLfloat sensitivity = 0.05f;
            if(xoffset!=0)
            {
                glm::mat4 trans = glm::mat4(1.0f);
                trans = glm::scale(trans, glm::vec3(xoffset*sensitivity, xoffset*sensitivity, xoffset*sensitivity));//比例缩放
                cameraPos -= glm::vec3(trans*glm::vec4(glm::normalize(glm::cross(cameraFront, cameraUp)),1.0f));//以右向量方向移动
            }
            if(yoffset!=0)
            {
                glm::mat4 trans = glm::mat4(1.0f);
                trans = glm::scale(trans, glm::vec3(yoffset*sensitivity, yoffset*sensitivity, yoffset*sensitivity));//比例缩放
                cameraPos += glm::vec3(trans*glm::vec4(glm::cross(glm::normalize(glm::cross(cameraFront, cameraUp)),cameraFront),1.0f));//以视角头顶方向向量移动
            }
        }
    }
    else if(Key_Z==1)//移动模型
    {
        if (event->buttons() == Qt::LeftButton)//修改全局变量lidar_*
        {
            GLfloat sensitivity = 0.02f;     //旋转时的灵敏度
            if(Key_X==0)
            {
                lidar_yaw += -xoffset*sensitivity;
                lidar_pitch+= -yoffset*sensitivity;
            }
            else {
                lidar_roll += yoffset*sensitivity;
            }
            if(lidar_pitch>=360)
                lidar_pitch=0;
            if(lidar_yaw>=360)
                lidar_yaw=0;
        }
        else if (event->buttons() == Qt::RightButton)//鼠标右键用来实现对移动物体（即局部坐标在世界坐标中的移动）
        {
            GLfloat sensitivity = 0.002f;    //移动时的灵敏度
            lidar_height+=-yoffset*sensitivity;
            lidar_offset+=xoffset*sensitivity;
        }
    }
}

void xp_opengl::wheelEvent(QWheelEvent *event)
{
    QPoint numDegrees = event->angleDelta() / 8;
    if(numDegrees.y()>0)
        cameraPos += 0.5f*cameraSpeed * cameraFront;
    else
        cameraPos -= 0.5f*cameraSpeed * cameraFront;
}

void xp_opengl::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        //设置光标形状
        cursor.setShape(Qt::PointingHandCursor);
        setCursor(cursor);
    }
    else if (event->button() == Qt::RightButton)
    {
        //设置光标形状
        cursor.setShape(Qt::SizeAllCursor);
        setCursor(cursor);
    }
}

void xp_opengl::mouseReleaseEvent(QMouseEvent *event)
{
    //设置光标形状
    cursor.setShape(Qt::ArrowCursor);
    setCursor(cursor);
}

