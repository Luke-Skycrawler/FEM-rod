#include <glm/glm.hpp>
#include "Eigen/Eigen"
#include "Eigen/Sparse"
#include <GL/glut.h>
#include <vector>
using namespace Eigen;
struct Vertex{
  glm::vec3 pos,f,v,v_old;
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
    rem_index(i, j, k, l);
  }
  Tetrahedron(const Tetrahedron &a):i(a.i),j(a.j),k(a.k),l(a.l){
    precomputation();
    for(int i=0; i < 4; i++){
      index[i] = a.index[i];
    }
  }
  inline void rem_index(int i,int j,int k,int l){
    index[0] = i;
    index[1] = j;
    index[2] = k;
    index[3] = l;
  }
  void precomputation();
  void compute_elastic_forces();
  void compute_force_differentials(int j, SparseMatrix<float> &K,std::vector<Vertex> &v);
  glm::mat3 piola_tensor(glm::mat3 &F);
  glm::mat3 differential_piola(glm::mat3 &F, glm::mat3 &dF);
  glm::vec3 df[4];
  int index[4];
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
struct Rod{
  float radius, length;
  int N_diagon, N_partition;
  std::vector<Tetrahedron> ttns;
  std::vector<Vertex> vtxs;
  void vertex();
  void connectivity();
  void add_dx(float dt);
  void derive_and_add_dv();
  // Rod(float radius = 0.6f, float length = 2.0f,int N_diagon = 8,int N_partition = 4):radius(radius),N_diagon(N_diagon),N_partition(N_partition),length(length){
  Rod(float radius = 0.1f, float length = 1.0f,int N_diagon = 8,int N_partition = 20):radius(radius),N_diagon(N_diagon),N_partition(N_partition),length(length),n(N_partition*(N_diagon + 1) * 3),x(n),b(n){
    vertex();
    connectivity();
    // reset();
  }
  int n;
  VectorXf x, b;
  SparseMatrix<float> A;
  SparseLU<SparseMatrix<float>, COLAMDOrdering<int> >   solver;

  void build_sparse();
  static const int implicit = 0;
  // fill A and b;
  void solve(){
    // Compute the ordering permutation vector from the structural pattern of A
    solver.analyzePattern(A); 
    // Compute the numerical factorization 
    solver.factorize(A); 
    //Use the factors to solve the linear system 
    x = solver.solve(b);  
  }
  static const glm::vec3 color;
  void draw();
  void reset();
  void step(float dt);
  int i = 1;
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
// #ifdef _MAIN

// #else
// extern Ball_Dynamic ball;
// extern Cloth cloth;
// #endif