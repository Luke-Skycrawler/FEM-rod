#include <glm/glm.hpp>
#include <GL/glut.h>
#include <vector>
struct Ball{
  float radius;
  Ball(float radius=0.6f):radius(radius){}
  static const glm::vec3 color;
  void draw(glm::vec3 pos=glm::vec3(0.0f)){
    glColor3fv((float*)&color);
    glutSolidSphere(radius,30,30);
  }
};
struct Particle{
  float w;    // 1/mass
  glm::vec3 pos,acc,v,tmp;
  // bool v_updated;
  Particle(glm::vec3 pos,glm::vec3 gravity,float w)
  :w(w),v(0.0f),pos(pos),acc(gravity){}
  Particle():w(w),v(0.0f),pos(0.0f),acc(0.0f){}
  bool collision[2];
  // FIXME: only for convenience, attributed to Constrain
  void prelaunch(){collision[0]=collision[1]=false;}
};

struct Ball_Dynamic : Particle{
  Ball ball;
  const glm::vec3 pos_init;
  Ball_Dynamic(float radius,glm::vec3 pos,glm::vec3 acc,float w)
  :ball(radius),Particle(pos,acc,w),pos_init(pos){
    
  }
  void draw();
  void reset(){
    pos=pos_init;
    v=glm::vec3(0.0f);
  }
};
struct Constrain{
  std::vector<Particle*> m;
  Particle * ext_obj;
  float k,len_slack;
  Constrain(Particle &m1,Particle *ext_obj=NULL):ext_obj(ext_obj){
    // collision constrain with external object
    m.push_back(&m1);
  }
  Constrain(Particle &m1,Particle &m2,float k)
  :k(k),len_slack(glm::length(m1.pos-m2.pos)),ext_obj(NULL){
    m.push_back(&m1);
    m.push_back(&m2);
  }
  void solve();
};
struct Vertex{
  glm::vec3 pos,f,v;
  Vertex(const glm::vec3 &pos, float M_inv = 1.0f):pos(pos), f(0.0f), v(0.0f), M_inv(M_inv){}
  // Vertex(Vertex &a):pos(a.pos), f(a.f){}
  Vertex():pos(0.0f), f(0.0), v(0.0f), M_inv(1.0f){}
  float M_inv;
};
struct Tetrahedron{
  Vertex &i,&j,&k,&l;
  glm::mat3 Bm;
  float W;
  Tetrahedron(std::vector<Vertex> &v,int i,int j,int k,int l):i(v[i]),j(v[j]),k(v[k]),l(v[l]){
    precomputation();
  }
  Tetrahedron(const Tetrahedron &a):i(a.i),j(a.j),k(a.k),l(a.l){
    precomputation();
  }
  void precomputation();
  void compute_elastic_forces();
  glm::mat3 piola_tensor(glm::mat3 &F);
  inline void draw(){
    static glm::vec3 color[4]={
      glm::vec3(1.0f,1.0f,1.0f),
      glm::vec3(1.0f,0.6f,0.6f),
      glm::vec3(0.5f,0.5f,0.5f),
      glm::vec3(.0f,0.0f,0.0f)
    };
  glBegin(GL_TRIANGLES);
    glColor3fv((float*)&color[0]);
    glVertex3fv((float*)&i);
    glVertex3fv((float*)&j);
    glVertex3fv((float*)&k);
  glEnd();

  glBegin(GL_TRIANGLES);
    glColor3fv((float*)&color[1]);
    glVertex3fv((float*)&j);
    glVertex3fv((float*)&k);
    glVertex3fv((float*)&l);
  glEnd();

  glBegin(GL_TRIANGLES);
    glColor3fv((float*)&color[2]);
    glVertex3fv((float*)&k);
    glVertex3fv((float*)&l);
    glVertex3fv((float*)&i);
  glEnd();

  glBegin(GL_TRIANGLES);
    glColor3fv((float*)&color[3]);
    glVertex3fv((float*)&l);
    glVertex3fv((float*)&i);
    glVertex3fv((float*)&j);
  glEnd();
  }
};
// glm::mat3 operator *(glm::mat3 A, double x){
//   return A;
// }
struct Rod{
  float radius, length;
  int N_diagon, N_partition;
  std::vector<Tetrahedron> ttns;
  std::vector<Vertex> vtxs;
  void vertex();
  void connectivity();
  // Rod(float radius = 0.6f, float length = 2.0f,int N_diagon = 8,int N_partition = 4):radius(radius),N_diagon(N_diagon),N_partition(N_partition),length(length){
  Rod(float radius = 0.1f, float length = 1.0f,int N_diagon = 8,int N_partition = 20):radius(radius),N_diagon(N_diagon),N_partition(N_partition),length(length){
    vertex();
    connectivity();
    // reset();
  }
  static const glm::vec3 color;
  void draw();
  void reset();
  void step(float dt);
  int i = 1;
};

struct Cloth{
  float x,y,kbend,kstretch;
  static const int iterations=20;
  int slicex,slicey,gerneric_constrains_cnt;

  std::vector<Constrain> constrains,collisions;
  std::vector<Particle> particles;
  
  Cloth(float x,float y,int slicex,int slicey,float kstretch=1.0f,float kbend=0.0f,bool pinned=false):
  x(x),y(y),slicex(slicex),slicey(slicey),kstretch(kstretch),kbend(kbend){
    particles.resize(slicex*slicey);
    reset();
    if(pinned)pin();
    gen();
  }
  void pin(bool _pin=true);
  void gen();
  void reset();
  void draw();
  void step(float dt);
  private:
  inline void DrawTriangle(Particle &p1,Particle &p2,Particle &p3,int cid){
    static glm::vec3 color[2]={
      glm::vec3(1.0f,1.0f,1.0f),
      glm::vec3(1.0f,0.6f,0.6f)
    };
    glColor3fv((float*)&color[cid]);
    glVertex3fv((float*)&p1.pos);
    glVertex3fv((float*)&p2.pos);
    glVertex3fv((float*)&p3.pos);
  }
  inline Particle &GetParticle(int i,int j){
    return particles[i*slicey+j];
  }
};

struct Plane{
  Plane(float w=5.0f):w(w){}
  const float w;
  void draw(){
    glColor3f(0.7f,0.7f,0.7f);
    glBegin(GL_TRIANGLE_STRIP);
    glVertex3f(-w,0.0f, w);
    glVertex3f( w,0.0f, w);
    glVertex3f(-w,0.0f,-w);
    glVertex3f( w,0.0f,-w);
    glEnd();
  }
};
// global variables
static const float E = 4e4f, nu = 0.2f;  // Young's modulus and Poisson's ratio
static const float mu = E / 2 / (1 + nu), lambda = E * nu / (1 + nu) / (1 - 2 * nu);  // Lame parameters
static const float gravity = 4.f;
#ifdef _MAIN

#else
extern Ball_Dynamic ball;
extern Cloth cloth;
#endif