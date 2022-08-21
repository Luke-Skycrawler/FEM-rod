#include "global.h"
#include <GL/glut.h>
#include <iostream>
// #define _USE_MATH_DEFINES
// #include <cmath>

// #define _DEBUG_1
#define M_PI       3.14159265358979323846   // pi

using namespace std;
using namespace glm;
static const glm::vec3 g(0.0f,-0.98f,0.0f);
static float r=ball.ball.radius;
static const float dh=0.02f,epsilon=1e-6;

void Rod::vertex(){
  vtxs.resize((N_partition + 1) * (N_diagon + 1));
  // FIXME: resize?
  float z = length / N_partition;
  for(int i=0;i<N_partition+1;i++){
    vtxs[i * (N_diagon +1)] = Vertex(glm::vec3(0.f, i * z, 0.f));
    for(int j =0;j<N_diagon;j++){
      int I = i * (N_diagon+1) + j + 1;
      vtxs[I] = Vertex(glm::vec3(cos(j * M_PI * 2 / N_diagon) * radius, i * z, -sin(j * M_PI * 2 / N_diagon) * radius));
    }
  }
  #ifdef _DEBUG_0
  for(auto v:vtxs){
    cout << v.x << v.y << v.z<< endl;
  }
  #endif
}
void Rod::connectivity(){
  // ttns.resize(3 * N_diagon * N_partition);
  // FIXME: no default constructor
  for(int i=0;i<N_partition;i++){
    for(int j = 0;j<N_diagon;j++){
      int K = i * (N_diagon +1);
      int I = K + j + 1;
      int J = I +1;
      if (j == N_diagon - 1) J = K + 1;
      // int T = I * 3;
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

}

void Rod::draw(){
  // glBegin(GL_TRIANGLES);
  #ifdef _DEBUG_1
  ttns[i].draw();
  #else
  for(auto t: ttns){
    t.draw();
  }
  #endif
  // glEnd();
}

void Rod::step(float dt){
  for(auto v: vtxs){
    v.f = glm::vec3(0.0);
  }
  for(auto e: ttns){
    e.compute_elastic_forces();
  }
}

void Tetrahedron::precomputation(){
  glm::mat3 Dm(i.pos-l.pos, j.pos-l.pos, k.pos-l.pos);
  Bm = glm::inverse(Dm); 
  W = 1.0f / 6 * glm::determinant(Dm);
}
void Tetrahedron::compute_elastic_forces(){
  glm::mat3 Ds(i.pos-l.pos, j.pos-l.pos, k.pos-l.pos);
  glm::mat3 F = Ds * Bm;
  // glm::mat3 P = P(F);
  glm::mat3 H = W * glm::transpose(P(Bm));
  i.f += H[0];
  j.f += H[1];
  k.f += H[2];
  l.f += -(H[0] + H[1] + H[2]);
}


glm::mat3 Tetrahedron::Piona_tensor(glm::mat3 &F){
  return (F + glm::transpose(F) - glm::mat3(1.0f) * 2.0f) * mu + glm::mat3(1.0f) * lambda * (F[0][0] + F[1][1] + F[2][2] - 3.0f);
}

void Cloth::reset(){
  float w=0.1f;
  for(int i=0;i<slicex;i++)
    for(int j=0;j<slicey;j++){
      glm::vec3 t(i*x/slicex-0.5f*x,0.7f,-j*y/slicey+0.5f*y);
      particles[i*slicey+j]=Particle(t,g,w);
    }
}
void Cloth::pin(bool _pinned){
  if(_pinned){
    GetParticle(0,0).               w=0.0f;
    GetParticle(0,slicey-1).        w=0.0f;
    GetParticle(slicex-1,0).        w=0.0f;
    GetParticle(slicex-1,slicey-1). w=0.0f;
    GetParticle(0,0).               acc=vec3(0.0f);
    GetParticle(0,slicey-1).        acc=vec3(0.0f);
    GetParticle(slicex-1,0).        acc=vec3(0.0f);
    GetParticle(slicex-1,slicey-1). acc=vec3(0.0f);
  }
  else{
    GetParticle(0,0).               w=0.1f;
    GetParticle(0,slicey-1).        w=0.1f;
    GetParticle(slicex-1,0).        w=0.1f;
    GetParticle(slicex-1,slicey-1). w=0.1f;
    GetParticle(0,0).               acc=g;
    GetParticle(0,slicey-1).        acc=g;
    GetParticle(slicex-1,0).        acc=g;
    GetParticle(slicex-1,slicey-1). acc=g;
  }
}
void Cloth::gen(){
  for(int i=0;i<slicex;i++)
    for(int j=0;j<slicey;j++){
      // stretch constrains
      if(i<slicex-1)    constrains.push_back(Constrain(GetParticle(i,j),GetParticle(i+1,j),kstretch));
      if(j<slicey-1)    constrains.push_back(Constrain(GetParticle(i,j),GetParticle(i,j+1),kstretch));
      if(i&&j<slicey-1) constrains.push_back(Constrain(GetParticle(i,j),GetParticle(i-1,j+1),kstretch));
      // TODO: bend constrains
      if(i<slicex-1&&j) constrains.push_back(Constrain(GetParticle(i,j),GetParticle(i+1,j-1),kbend));
      if(i<slicex-2)    constrains.push_back(Constrain(GetParticle(i,j),GetParticle(i+2,j),kbend));
      if(j<slicex-2)    constrains.push_back(Constrain(GetParticle(i,j),GetParticle(i,j+2),kbend));
      // collision constrains
      constrains.push_back(Constrain(particles[i*slicey+j]));
    }
    // gerneric_constrains_cnt=constrains.size();
}
void Cloth::draw(){
  int col = 0;
  glBegin(GL_TRIANGLES);
  for(int w = 0; w < slicex - 1; w++){
    for(int h = 0; h < slicey - 1; h++){
      DrawTriangle(GetParticle(w+1,h  ), GetParticle(w,   h), GetParticle(w, h+1), col);
      DrawTriangle(GetParticle(w+1,h+1), GetParticle(w+1, h), GetParticle(w, h+1), col);
      col=!col;
    }
  }
  glEnd();
}
void Cloth::step(float dt){
  ball.v+=ball.acc;
  ball.tmp=ball.pos+dt*ball.v;
  // ugly
  int init=1;
  // cout<<"size before:"<<collisions.size()<<endl;
  for(auto it=particles.begin();it!=particles.end();it++){
    it->v+=dt*it->acc;
    // damp(it->v);
    it->tmp=it->pos+dt*it->v;
    it->prelaunch();
    if(length(it->tmp-ball.tmp)<r+dh){
      if(init){
        init=0;
        collisions.push_back(Constrain(*it,&ball));
        // cout<<"#particles:"<<collisions[0].m.size()<<endl;
        // // cout<<"#particles:"<<collisions[0].m.size()<<endl;
      }
      else collisions[0].m.push_back(&*it);
      it->collision[0]=true;
    }
  }
  for(int i=0;i<iterations;i++){
    if(collisions.size())collisions[0].solve();
    // end of collision constrains
    for(auto it=constrains.begin();it!=constrains.end();it++){
      it->solve();
    }    
  }
  vec3 dp(0.0f);
  for(auto it=particles.begin();it!=particles.end();it++){
    it->v=(it->tmp-it->pos)/dt;
    // collision , reflect velocity 
    if(it->collision[0]){
      it->v-=         2.0f*normalize(it->tmp-ball.tmp)*((it->v-ball.v)*normalize(it->tmp-ball.tmp));
      dp+=1.0f/it->w* 2.0f*normalize(it->tmp-ball.tmp)*((it->v-ball.v)*normalize(it->tmp-ball.tmp));
    }
    if(it->collision[1])it->v.y=0.0f;
    it->pos=it->tmp;
    // it->v_updated=false;
  }  
  ball.v=(ball.tmp-ball.pos)/dt+dp*ball.w;
  ball.pos=ball.tmp;
  while(collisions.size())collisions.pop_back();
  // cout<<"size after:"<<collisions.size()<<endl;
}
void Constrain::solve(){
  float l,C0,C1;
  if(ext_obj){
    // collision constrains
    float C=0.0f;
    vec3 cm(0.0f);
    for(auto it=m.begin();it!=m.end();it++){
      vec3 d=(*it)->tmp-ext_obj->tmp;
      if(length(d)<r+dh){
      // inequity constrain
        (*it)->tmp+=        normalize(d)*(r+dh-length(d));
        cm-=1.0f/(*it)->w*  normalize(d)*(r+dh-length(d));
        C+= 1.0f/(*it)->w;
      }
    }
    if(ext_obj->w>epsilon){
      C+=1.0f/ext_obj->w;
      ext_obj->tmp+=cm/C;
      for(auto it=m.begin();it!=m.end();it++){
        (*it)->tmp+=cm/C;
      }
    }
    return;
  }
  switch(m.size()){
    // collision with ball and floor only
    case 1:
      // l=length(m[0]->tmp-ext_obj->tmp);
      // C0=m[0]->w/(m[0]->w+ext_obj->w);C1=1.0f-C0;
      // // FIXME: null , 
      // if(l<r+dh){
      //   m[0]->tmp+=m[0]->tmp*(r+dh-l)/l*C0;
      //   ext_obj->tmp-=m[0]->tmp*(r+dh-l)/l*C1;
      //   m[0]->collision[0]=true;
      // }
      
      if(m[0]->tmp.y<dh){
        m[0]->tmp.y=dh;
        m[0]->collision[1]=true;
      }
      break;
    case 2:
      vec3 p10=m[0]->tmp-m[1]->tmp;
      l=length(p10);
      C0=m[0]->w/(m[0]->w+m[1]->w);C1=1.0f-C0;
      m[0]->tmp+=C0*k*(len_slack-l)*(p10)/l;
      m[1]->tmp-=C1*k*(len_slack-l)*(p10)/l;
      break;
    // case 4:
  }
}
const vec3 Ball::color(0.4f,0.4f,0.8f);
void Ball_Dynamic::draw(){
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  // glLoadIdentity();
  glTranslatef(pos.x,pos.y,pos.z);
  ball.draw(pos);
  glPopMatrix();
  // glPopMatrix();
}
