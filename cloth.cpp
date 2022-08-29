#include "global.h"
#include <GL/glut.h>
#include <iostream>
// #define _USE_MATH_DEFINES
// #include <cmath>

// #define _DEBUG_1
#define M_PI 3.14159265358979323846 // pi

using namespace std;
using namespace glm;
static const vec3 gravity(4.0f, 0.0f, 0.0f);

void Rod::vertex()
{
  vtxs.resize((N_partition + 1) * (N_diagon + 1));
  float z = length / N_partition;
  for (int i = 0; i < N_partition + 1; i++)
  {
    float M_inv = i < N_partition ? 1.0f : 0.0f;
    int I = i * (N_diagon + 1);
    vtxs[I] = Vertex(vec3(0.f, i * z, 0.f), M_inv);
    for (int j = 0; j < N_diagon; j++)
    {
      int J = I + 1 + j;
      vtxs[J] = Vertex(vec3(cos(j * M_PI * 2 / N_diagon) * radius, i * z, -sin(j * M_PI * 2 / N_diagon) * radius), M_inv);
    }
  }
#ifdef _DEBUG_0
  for (auto v : vtxs)
  {
    cout << v.x << v.y << v.z << endl;
  }
#endif
}
void Rod::connectivity()
{
  for (int i = 0; i < N_partition; i++)
  {
    for (int j = 0; j < N_diagon; j++)
    {
      int H = N_diagon + 1;
      int K = i * H;
      int I = K + j + 1;
      int J = j == N_diagon - 1 ? K + 1 : I + 1;
      ttns.push_back(Tetrahedron(vtxs, I, J, K, I+ H));
      ttns.push_back(Tetrahedron(vtxs, I + H, K, K+H, J));
      ttns.push_back(Tetrahedron(vtxs, J, I+H, J+H, K+H));
      // ttns.push_back(Tetrahedron(vtxs, I, J, K, I + N_diagon + 1));
      // ttns.push_back(Tetrahedron(vtxs, K, I + N_diagon + 1, J + N_diagon + 1, K + N_diagon + 1));
      // ttns.push_back(Tetrahedron(vtxs, J, K, I + N_diagon + 1, J + N_diagon + 1));
    }
  }
#ifdef _DEBUG_1
  cout << ttns.size() << endl;
#endif
}

void Rod::reset()
{
  vertex();
  connectivity();
}

void Rod::draw()
{
  glBegin(GL_TRIANGLES);
#ifdef _DEBUG_1
  ttns[index_visible].draw();
#else
  for (auto &t : ttns)
  {
    t.draw();
  }
#endif
  glEnd();
}

void Rod::compute_f()
{
  for (auto &v : vtxs)
  {
    v.f = gravity;
  }
  for (auto &e : ttns)
  {
    e.compute_elastic_forces();
  }
}

void Rod::step(float dt)
{
  // explicit, or used as initial guess for newton iteration
  compute_f();
  for (auto &v : vtxs)
  {
    v.x += dt * v.v;
    v.v += dt * v.M_inv * v.f;
  }
#ifdef IMPLICIT
  for (int k = 0; k < 2; k++)
  {
    // FIXME: specify a tolerance and max iteration count
    // newton method

    build_sparse(dt);

    compute_f();
    for (int i = 0; i < n; i++)
    {
      auto &vtx = vtxs[i];
      vec3 dv = vtx.v_n - vtx.v;
      for (int d = 0; d < 3; d++)
      {
        b.coeffRef(i * 3 + d) = 1.0f / dt * dv[d] + vtx.f[d];
      }
    }
    #ifdef RESIDUE
    float sum = 0.0f, tot = 0.0f, x_norm = 0.0f;
    for (auto &v:vtxs){
      vec3 rhs = v.v_n + dt * v.f;
      vec3 r = rhs - v.v;
      tot += dot(rhs ,rhs);
      sum += dot(r , r);
      x_norm += dot(v.x, v.x);
    }
    cout<< "iter" << k << ", error = "<<sqrt(sum/ tot) << endl;
    cout<< "tot = "<<sqrt(tot) << endl;
    #endif

    solve();
    #ifdef RESIDUE
    cout<< "norm: "<< x.norm() << endl;
    cout<< "b norm: "<< b.norm() << endl;

    #endif
    add_dx_dv(dt);
  }
  for (auto &v : vtxs)
  {
    v.v_n = v.v;
  }
  clean();
#endif
}

void Rod::clean()
{
  A.setZero();
  b.setZero();
  // b.data();
}

void Rod::add_dx_dv(float dt)
{
  for (int i = 0; i < n; i++)
  {
    int I = 3 * i;
    vec3 tmp(
        x.coeff(I + 0),
        x.coeff(I + 1),
        x.coeff(I + 2));
    vtxs[i].x += tmp;
    vtxs[i].v += tmp / dt;
  }
}

void Rod::build_sparse(float dt)
{
  A.setZero();
  for (auto &e : ttns)
  {
    for (int j = 0; j < 4; j++)
    {
      e.compute_force_differentials(j, A, vtxs);
    }
  }
  for (int i = 0; i < 3 * n; i++)
  {
    // K_ii += M/dt^2
    A.coeffRef(i, i) += 1.0f / (dt * dt);
  }
  #ifdef RESIDUE
  cout<<"dt ^-2: " << 1.0f / dt / dt << endl;
  #endif
}

void Tetrahedron::precomputation()
{
  mat3 Dm(i.x - l.x, j.x - l.x, k.x - l.x);
  Bm = inverse(Dm);
  W = abs(1.0f / 6 * determinant(Dm));
}

void Tetrahedron::compute_elastic_forces()
{
  mat3 Ds(i.x - l.x, j.x - l.x, k.x - l.x);
  mat3 F = Ds * Bm;
  mat3 P = piola_tensor(F);
  mat3 H = -W * P * transpose(Bm);
  // NOTE: glm matrix is colum-major, i.e. H[i][j] gets element h_ji
  i.f += H[0];
  j.f += H[1];
  k.f += H[2];
  l.f += -(H[0] + H[1] + H[2]);
}

inline void put(SparseMatrix<float> &K, int i, int col, vec3 &df)
{
  for (int k = 0; k < 3; k++)
  {
    int row = 3 * i + k;
    // K.insert(row, col) = df[k];
    K.coeffRef(row, col) += df[k];
  }
}
void Tetrahedron::compute_force_differentials(int _j, SparseMatrix<float> &K, vector<Vertex> &v)
{
  // batch computation parital f_i for all v_i adjacent to v_j (in this tet)
  mat3 Ds(i.x - l.x, j.x - l.x, k.x - l.x);
  mat3 F = Ds * Bm;
  int J = index[_j];
  if (v[J].M_inv == 0.0f)
    return;
  #ifdef RESIDUE
  float tmp  = 0.0f, t2 = 0.0f;
  #endif
  for (int k = 0; k < 3; k++)
  {
    // like ti.static
    vec3 d_v[4] = {vec3(0.0f),vec3(0.0f),vec3(0.0f),vec3(0.0f)}, df[4] = {vec3(0.0f),vec3(0.0f),vec3(0.0f),vec3(0.0f)};
    d_v[_j][k] = -1.0f;
    mat3 d_Ds(d_v[0] - d_v[3], d_v[1] - d_v[3], d_v[2] - d_v[3]);
    mat3 dF(d_Ds * Bm);
    mat3 dP(differential_piola(F, dF));
    mat3 dH(-W * dP * transpose(Bm));
    df[0] = dH[0];
    df[1] = dH[1];
    df[2] = dH[2];
    df[3] = -(dH[0] + dH[1] + dH[2]);

    /* put vec3 to
    row = [3i, 3i+2]
    col = 3j + k
    */
    int col = 3 * J + k;
    for (int _i = 0; _i < 4; _i++)
    {
      int I = index[_i];
      if (v[I].M_inv == 0.0f)
        continue;
      put(K, I, col, df[_i]);
      #ifdef RESIDUE
      tmp += dot(df[_i], df[_i]);
      #endif
    }
  }
    #ifdef RESIDUE
    // cout<< sqrt(tmp) << endl;
    #endif
}
mat3 Tetrahedron::piola_tensor(mat3 &F)
{
  // linear elasticity
  // return (F + transpose(F) - mat3(1.0f) * 2.0f) * mu + mat3(1.0f) * lambda * (F[0][0] + F[1][1] + F[2][2] - 3.0f);
  // neo-hookean
  auto F_inv_T = transpose(inverse(F));
  return mu * (F - F_inv_T) + lambda * log(determinant(F)) / 2.0f * F_inv_T;
}

mat3 Tetrahedron::differential_piola(mat3 &F, mat3 &dF)
{
  auto F_inv_T = transpose(inverse(F));
  mat3 B = inverse(F) * dF;
  float tr = B[0][0] + B[1][1] + B[2][2];
  return mu * dF + (mu - lambda * log(determinant(F))) * F_inv_T * transpose(dF) * F_inv_T + lambda * tr * F_inv_T;
}
