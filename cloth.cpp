#include "global.h"
#include <GL/glut.h>
#include <iostream>
// #define _USE_MATH_DEFINES
// #include <cmath>

#define _DEBUG_2
#define M_PI       3.14159265358979323846   // pi

using namespace std;
using namespace glm;
static const glm::vec3 g(0.0f,-0.98f,0.0f);
// static const float dh=0.02f,epsilon=1e-6f;

void Rod::vertex(){
  vtxs.resize((N_partition + 1) * (N_diagon + 1));
  float z = length / N_partition;
  for(int i=0;i<N_partition+1;i++){
    float M_inv = i < N_partition? 1.0f:0.0f;
    vtxs[i * (N_diagon +1)] = Vertex(glm::vec3(0.f, i * z, 0.f), M_inv);
    for(int j =0;j<N_diagon;j++){
      int I = i * (N_diagon+1) + j + 1;
      vtxs[I] = Vertex(glm::vec3(cos(j * M_PI * 2 / N_diagon) * radius, i * z, -sin(j * M_PI * 2 / N_diagon) * radius), M_inv);
    }
  }
  #ifdef _DEBUG_0
  for(auto v:vtxs){
    cout << v.x << v.y << v.z<< endl;
  }
  #endif
}
void Rod::connectivity(){
  for(int i=0;i<N_partition;i++){
    for(int j = 0;j<N_diagon;j++){
      int K = i * (N_diagon +1);
      int I = K + j + 1;
      int J = j == N_diagon - 1? K+1 : I +1;
      ttns.push_back(Tetrahedron(vtxs, I, J, K, I + N_diagon + 1));
      ttns.push_back(Tetrahedron(vtxs, K, I + N_diagon + 1, J + N_diagon + 1, K + N_diagon + 1));
      ttns.push_back(Tetrahedron(vtxs, J, K, I + N_diagon + 1, J + N_diagon + 1));
    }
  }
  #ifdef _DEBUG_1
  cout<< ttns.size()<<endl;
  #endif
}

void Rod::reset(){
  vertex();
  connectivity();
}

void Rod::draw(){
  // glBegin(GL_TRIANGLES);
  #ifdef _DEBUG_1
  ttns[i].draw();
  #else
  for(auto &t: ttns){
    t.draw();
  }
  #endif
  // glEnd();
}

void Rod::step(float dt){
  for(auto &v: vtxs){
    v.f = glm::vec3(gravity, 0.0f, 0.0f);
  }
  for(auto &e: ttns){
    e.compute_elastic_forces();
  }
  for(auto &v:vtxs){
    v.pos += dt * v.v;
    v.v += dt * v.M_inv * v.f;
  }
  // implicit
  if (implicit){
    build_sparse();
    for(int i=0;i<n / 3;i++){
      auto &vtx = vtxs[i];
      auto dv = vtx.v_old - vtx.v;
      for(int k=0;k<3;k++){
        b.coeffRef(i * 3 + k) = 1.0f / dt * dv[k] + vtx.f[k];
        // b[i * 3 + k] = 1.0f / dt * dv[k] + vtx.f[k];
      }
    }
    solve();
    add_dx(dt);
    derive_and_add_dv();
    for(int iters =0;iters < 20; iters++){
      // newton
    }
    for(auto &v:vtxs){
      v.v_old = v.v;
    }
  }
}

void Rod::add_dx(float dt){
  for(int i=0;i<n / 3;i++){
    glm::vec3 tmp(
      x.coeff(i+0), 
      x.coeff(i+1), 
      x.coeff(i+2)
    );
    vtxs[i].pos += tmp;
    vtxs[i].v += tmp/dt;
  }
}

void Rod::derive_and_add_dv(){
  // 
}

void Rod::build_sparse(){
  for(auto &e: ttns){
    for(int j=0; j < 4; j++){
      e.compute_force_differentials(j,A, vtxs);
      // FIXME: no use setting the matrix at the pinned points 
    }
  }
  for(int i=0;i<n;i++){
    // K_ii += 1 for mass matrix
    A.coeffRef(i,i) += 1.0f;
  }
}

void Tetrahedron::precomputation(){
  glm::mat3 Dm(i.pos-l.pos, j.pos-l.pos, k.pos-l.pos);
  Bm = glm::inverse(Dm); 
  W = abs(1.0f / 6 * glm::determinant(Dm));
}
void Tetrahedron::compute_elastic_forces(){
  glm::mat3 Ds(i.pos-l.pos, j.pos-l.pos, k.pos-l.pos);
  glm::mat3 F = Ds * Bm;
  glm::mat3 P = piola_tensor(F);
  glm::mat3 H = - W * P * glm::transpose(Bm);
  i.f += H[0];
  j.f += H[1];
  k.f += H[2];
  l.f += -(H[0] + H[1] + H[2]);
}

inline void put(SparseMatrix<float> &K, int i, int col, glm::vec3 &df){
  for(int k=0;k<3;k++){
    int row = 3* i + k;
    K.insert(row, col) = df[k];
  }
}
void Tetrahedron::compute_force_differentials(int _j,  SparseMatrix<float> &K, std::vector<Vertex> &v){
  // batch computation parital f_i for all v_i adjacent to v_j (in this tet) 
  glm::mat3 Ds(i.pos-l.pos, j.pos-l.pos, k.pos-l.pos);
  glm::mat3 F = Ds * Bm;
  for(int k = 0; k < 3; k++){
    // like ti.static
    glm::vec3 d_v[4];
    d_v[_j][k] = 1.0f;
    glm::mat3 d_Ds(d_v[0] - d_v[3], d_v[1] - d_v[3], d_v[2] - d_v[3]);
    glm::mat3 dF = d_Ds * Bm;
    glm::mat3 dP = differential_piola(F, dF);
    glm::mat3 dH = - W * dP *glm::transpose(Bm);
    df[0] = dH[0];
    df[1] = dH[1];
    df[2] = dH[2];
    df[3] = -(dH[0] + dH[1] + dH[2]);

    /* put vec3 to 
    row = [3i, 3i+2]
    col = 3j + k
    */
   int J = index[_j];
   if(v[J].M_inv == 0) return;
   int col = 3 * J + k;
   for(int _i=0;_i<4; _i ++){
    int i = index[_i];
    if(v[i].M_inv == 0) continue;
    put(K, i , col, df[_i]);
   }
  }
}
glm::mat3 Tetrahedron::piola_tensor(glm::mat3 &F){
  // linear elasticity
  // return (F + glm::transpose(F) - glm::mat3(1.0f) * 2.0f) * mu + glm::mat3(1.0f) * lambda * (F[0][0] + F[1][1] + F[2][2] - 3.0f);
  // neo-hookean
  auto Fjnv_T = glm::transpose(glm::inverse(F));
  return mu * (F - Fjnv_T) + lambda * log(glm::determinant(F)) / 2.0f * Fjnv_T;
}

glm::mat3 Tetrahedron::differential_piola(glm::mat3 &F, glm::mat3 &dF){
  auto Fjnv_T = glm::transpose(glm::inverse(F));
  glm::mat3 B = glm::inverse(F) * dF;
  float tr = B[0][0] + B[1][1] + B[2][2];
  return mu * dF + (mu - lambda * log(glm::determinant(F))) * Fjnv_T * glm::transpose(dF) * Fjnv_T + lambda * tr * Fjnv_T;
}

