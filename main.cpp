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
static const vec3 g(0.0f,-0.98f,0.0f);
Plane plane;
Ball_Dynamic ball(0.3f,vec3(0.0f,1.5f,0.0f),g,0.001f);
// Ball_Dynamic ball(0.6f,vec3(0.0f,0.0f,0.0f),vec3(0.0f),0.0f);
Cloth cloth(2.0f,2.0f,50,50,1.0f,0.0f,true);
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
  plane.draw();
  ball.draw();
  cloth.draw();

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
static int cspeed=1,cweight=1;
static float SlowMotion=speeds[cspeed];
void idle(void){
  float t = (float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
  float dt = t - realtime;

  dt = (dt > 0.033f) ? 0.033f : dt; // keep 30fps

  // cloth.step(0);
  cloth.step(dt/SlowMotion);

  realtime=t;
  glutPostRedisplay();
}

void keyboard(unsigned char key , int x , int y){
  switch(key){
    case 27: exit(0); break; // esc
    case 'r':case 'R':cloth.reset();cloth.pin();ball.reset();break;
    case ' ':cloth.pin(false);break;
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
  if (key == GLUT_KEY_LEFT) {
    if(cweight)cweight--;
    else cweight=2;
    ball.w=weights[cweight];
    cout<<"mass: "<<1.0f/ball.w<<endl;
  }
  if (key == GLUT_KEY_RIGHT) {
    cweight=(cweight+1)%3;
    ball.w=weights[cweight];
    cout<<"mass: "<<1.0f/ball.w<<endl;
  }
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
