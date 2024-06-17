#ifndef XP_OPENGL_H
#define XP_OPENGL_H

#ifdef _WIN32
#define Delay(x) Sleep(1000*x);
#define Delayms(x) Sleep(x);
#ifdef _WIN64
//define something for Windows (64-bit only)
#else
//define something for Windows (32-bit only)
#endif
#elif defined  __linux__ || defined __APPLE__
#define Delay(x) sleep(x);
#define Delayms(x) usleep(x*1000);
#else
#   error "Unknown compiler"
#endif

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLExtraFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMoveEvent>
#include <QKeyEvent>
#include <QtCore>
#include <QString>
#include <QGLFormat>
#include <freetype/config/ftheader.h>
#include FT_FREETYPE_H
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <string.h>
#include <QCoreApplication>
#include <map>

QT_FORWARD_DECLARE_CLASS(QOpenGLContext)
QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

struct Character {
    GLuint     TextureID;  // 字形纹理的ID
    glm::ivec2 Size;       // 字形大小
    glm::ivec2 Bearing;    // 从基准线到字形左部/顶部的偏移值
    int     Advance;    // 原点距下一个字形原点的距离
};

struct Record{
    long int num;//点个数
    long int start;//点的起始索引
    long int dis;//帧对应的全局距离
    glm::mat4 offset;//矩阵
};

class xp_opengl : public QOpenGLWidget,protected QOpenGLExtraFunctions
{
    Q_OBJECT
    std::map<int, Character> Characters;//字体缓存
    FT_Library ft;
    FT_Face face;
    QVector<GLuint> TEXTURE,EBO,VBO,VAO,PROGRAM;
public:
    xp_opengl(QWidget *parent = 0);
    ~xp_opengl();
    int TextTextureInit();//初始化字体
    GLuint BuildProgram(const char *vertex,const char *fragment);//绑定编译program
    void PrebuildVABEO(std::function<void()> settings);//创建VAO并绑定VBO,VEO，支持传入数据，自动处理绑定关系
    int ReleaseVABEO(int vabeo_index);//释放VAO,VBO,VEO
    void Draw(int program_index,int vabeo_index,std::function<void()> settings);//绘图，支持重新传入数据，自动处理绑定关系
    GLuint xp_LoadTextureFromImg(std::string path,GLenum format=GL_RGB);//加载纹理
    void Text(std::wstring text,GLfloat scale, glm::vec3 position,glm::vec3 color,int WldorWin=1,glm::mat4 offset=glm::mat4(1.0f));//WldorWin:是否锁定屏幕，使用L"XX"格式
    void printContextInformation();//打印显卡信息
protected:
    void initializeGL();
    void resizeGL(int w,int h);
    void paintGL();
protected:
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
private:
    QCursor cursor; // 管理光标形状
    glm::vec3 cameraPos   = glm::vec3(0.0f,-5.5f, 2.24f);//相机位置
    glm::vec3 cameraFront = glm::vec3(0.0f, 1.0f, 0.0f);//方向向量
    glm::vec3 cameraUp    = glm::vec3(0.0f, 0.0f,  1.0f);//上向量，非头顶方向
    //属性
    GLfloat yaw = 90.0f;       //偏航角
    GLfloat pitch = 0.0f;     //俯仰角
    GLfloat lastX = 0;        //光标上次x值
    GLfloat lastY = 0;        //光标上次y值
    //program uniform属性
    glm::mat4 glview = glm::mat4(1.0f);
    glm::mat4 glprojection = glm::mat4(1.0f);
    glm::mat4 glmodel = glm::mat4(1.0f);
    float cameraSpeed = 3.0f;
    //GPU缓存区
    int GPU_MEMMAX=100*1024*1024;//定义最大使用的显存，若L/G显示不正确，尝试改小此数值
    std::map<int,Record> records;
};

extern int Key_V;//显示点云
extern int Key_L;//录制线路点云
extern int Key_T;//显示目标
extern int Key_G;//实时/全局线路使能
extern int Key_B;//显示背景
extern int Key_N;//显示隧道线阶
extern int Key_C;//录制/回放时自动调整camera
extern int Key_R;//视角重置
extern int Key_P;//显示fps
extern int Key_Z;//旋转移动模型，+左键旋转，+右键移动
extern int Key_X;//z+x旋转模型另一轴
extern int Key_Space;//回放时播放和暂停
extern int Key_Shift;
extern int Key_Alt;
extern int Key_Ctrl;

extern glm::vec3 dotCamera;
extern glm::vec3 Euler;

extern int OpenGLStart;//刷新使能，linux需要关闭使能才能打开file窗口
extern wchar_t OpenGLStatus[100];//右上角提示信息
extern wchar_t OpenGLHUD[100];//HUD提示信息
#endif // MOPENGL_H
