#define _CRT_SECURE_NO_WARNINGS
#include <GL/glut.h>

#include <iostream>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#define _MAIN
#include "global.h"
using namespace glm;
using namespace std;
Rod rod;
#define _BASE_PBD
#ifdef _BASE_PBD
static const vec3 g(0.0f,-0.98f,0.0f);
static int stop = 0;
Plane plane;
float realtime;
void init(int argc, char* argv[]){
  glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
  // glEnable(GL_CULL_FACE);

  realtime = (float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
}
void display(void){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glDepthFunc(GL_LESS); 
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_NORMALIZE);
  // plane.draw();
  rod.draw();

  glutSwapBuffers();
}

void reshape(int width, int height){
  static GLfloat lightPosition[4] = {0.0f,  2.5f,  5.5f, 1.0f};
  static GLfloat lightDiffuse[3]  = {1.0f,  1.0f,  1.0f      };
  static GLfloat lightAmbient[3]  = {0.75f, 0.75f, 0.75f     };
  static GLfloat lightSpecular[3] = {1.0f,  1.0f,  1.0f      };

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glShadeModel(GL_SMOOTH);

  glViewport(0, 0, width, height);
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(30.0, (double)width / (double)height, 0.0001f, 1000.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(0.0f, 2.0f, 5.0f, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0); // pos, tgt, up

  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightDiffuse);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  lightAmbient);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
}
static float speeds[]={1.0f,5.0f,25.0f,125.0f};
static float weights[]={0.002f,0.001f,0.0001f};
static int cspeed=3,cweight=1;
static float SlowMotion=speeds[cspeed];
void idle(void){
  float t = (float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
  float dt = t - realtime;

  dt = (dt > 0.033f) ? 0.033f : dt; // keep 30fps

  // cloth.step(0);
  // cloth.step(dt/SlowMotion);

  // rod.step(stop? 0.0f: dt / SlowMotion);
  if(!stop)
    rod.step(dt / SlowMotion);
  realtime=t;
  glutPostRedisplay();
}

void keyboard(unsigned char key , int x , int y){
  switch(key){
    case 27: exit(0); break; // esc
    case 'r':case 'R':rod.reset(); break;
    case 's':case 'S':stop = !stop; break;
    case 'j':rod.i += 1; break;
    case 'k':rod.i -= 1; break;
    case ' ':break;
  }
}
void special(int key, int x, int y){
  if (key == GLUT_KEY_UP) {
    cspeed=(cspeed+1)%3;
    SlowMotion=speeds[cspeed];
    cout<<"Slow Motion: "<<SlowMotion<<endl;
  }
  if (key == GLUT_KEY_DOWN) {
    if(cspeed)
      cspeed--;
    else cspeed=2;
    SlowMotion=speeds[cspeed];
    cout<<"Slow Motion: "<<SlowMotion<<endl;
  }
  // if (key == GLUT_KEY_LEFT) {
  //   if(cweight)cweight--;
  //   else cweight=2;
  //   // cout<<"mass: "<<1.0f/ball.w<<endl;
  // }
  // if (key == GLUT_KEY_RIGHT) {
  //   cweight=(cweight+1)%3;
  //   // cout<<"mass: "<<1.0f/ball.w<<endl;
  // }
}


int main(int argc, char* argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(800  , 600);
  glutCreateWindow("Position-Based Dynamics");

  init(argc, argv);

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  glutSpecialFunc(special);
  glutMainLoop();
  return 0;
}

#else
#include <math.h>
// #define _DEBUG_1
#define PI 3.1415926535
static int du = 90, OriX = -1, OriY = -1;   //du是视点和x轴的夹角
static float r = 1.5, h = 0.0;   //r是视点绕y轴的半径，h是视点高度即在y轴上的坐标
static float c = PI / 180.0;    //弧度和角度转换参数
static bool mode = false;
void startPicking(int x,int y);
void stopPicking();
void renderScene(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //清除缓冲，GL_COLOR_BUFFER_BIT ：颜色缓冲标志位
	glLoadIdentity();                                       //重置当前矩阵为4*4的单位矩阵
	gluLookAt(r * cos(c * du), h, r * sin(c * du), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);   //从视点看远点
	if(mode)glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		else glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_pos[] = { 5,5,5,1 };

	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, white);
	glEnable(GL_LIGHT0);

	glColor3f(1.0, 1.0, 1.0);
	// glutSolidTeapot(0.5f);
  rod.draw();
	// glutSolidSphere(0.5,20,20);
	glutSwapBuffers();                                      //交换两个缓冲区指针
}

void Mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN){
		OriX = x, OriY = y;
		startPicking(x,y);
		renderScene();
		stopPicking();

	}  //记录鼠标按下位置
}

void onMouseMove(int x, int y)   //处理鼠标拖动
{
	du += x - OriX; //鼠标在窗口x轴方向上的增量加到视点与x轴的夹角上，就可以左右转
	h += 0.03 * (y - OriY);  //鼠标在窗口y轴方向上的改变加到视点y的坐标上，就可以上下转
	if (h > 1.0)   h = 1.0;  //对视点y坐标作一些限制，不会使视点太奇怪
	else if (h < -1.0) h = -1.0;
	OriX = x, OriY = y;  //将此时的坐标作为旧值，为下一次计算增量做准备
}

void init()
{
	glEnable(GL_DEPTH_TEST);    //启用深度，根据坐标的远近自动隐藏被遮住的图形（材料）
}
static int scr_width,scr_height;
void reshape(int w, int h)
{
	glViewport(0, 0, w, h);    //截图;1、2为视口的左下角;3、4为视口的宽度和高度
	glMatrixMode(GL_PROJECTION);    //将当前矩阵指定为投影矩阵
	glLoadIdentity();
	gluPerspective(75.0, (float)w / h, 1.0, 1000.0); //1、视野在Y-Z平面的角度[0,180];2、投影平面宽度与高度的比率;3、近截剪面到视点的距离;4、远截剪面到视点的距离
	glMatrixMode(GL_MODELVIEW);     //对模型视景矩阵堆栈应用随后的矩阵操作.
	scr_width=w;
	scr_height=h;
}
#define BUFSIZE 512
GLuint selectBuf[BUFSIZE];
void stopPicking() {
	int hits;

	// restoring the original projection matrix
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glFlush();

	// returning to normal rendering model
	hits = glRenderMode(GL_RENDER);

	// if there are hits process them
	if (hits != 0)mode = !mode;
		// processHits(hits, selectBuf);
}

void startPicking(int cursorX, int cursorY)
{
     GLint viewport[4];
     glSelectBuffer(BUFSIZE, selectBuf);
     glRenderMode(GL_SELECT);

     glMatrixMode(GL_PROJECTION);
     glPushMatrix();
     glLoadIdentity();

     glGetIntegerv(GL_VIEWPORT, viewport);
    //  gluPickMatrix(cursorX, cursorY, 5, 5, viewport);
     gluPickMatrix(cursorX, (viewport[3]-cursorY), 1, 1, viewport);
     gluPerspective(75, (float)scr_width/scr_height, 0.1, 1000);
     glMatrixMode(GL_MODELVIEW);
     glInitNames();
}
int main(int argc, char* argv[])
{
	glutInit(&argc, argv);                                          //初始化glut库
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);      //设置初始显示模式
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(600, 600);
	glutCreateWindow("***************");
	init();
	glutReshapeFunc(reshape);       //
	glutDisplayFunc(renderScene);           //
	glutIdleFunc(renderScene);          //设置不断调用显示函数
	glutMouseFunc(Mouse);
	glutMotionFunc(onMouseMove);

	//glutDisplayFunc(renderScene);                                 //调用函数
	glutMainLoop();//enters the GLUT event processing loop.
	return 0;
}
#endif