//CSCI 5611
//Boids Implementation
//Yuxuan Huang

import java.lang.Math;
import java.util.PriorityQueue;
import java.util.Comparator;

int agts = 200; //number of agents

PVector[] agtPos;  // Agents positions
PVector[] agtVel;  // Agents velocities


int windowsize = 600;
//Create Window
void setup() {
  size(600, 600, P2D);
  surface.setTitle("Boids");
  init();
}

float v = 10.0;
void init() {
  // initialize agents
  agtPos = new PVector[agts];
  agtVel = new PVector[agts];
  for (int i=0; i<agts; i++){
    agtPos[i] = new PVector(windowsize*(float)Math.random(), windowsize*(float)Math.random());
    float theta = 360*(float)Math.random();
    agtVel[i] = new PVector((float)Math.cos(theta), (float)Math.sin(theta));
    agtVel[i].mult(v);
  }
}


float sep_radius = 30;
float sep_str = 0.5;
float ali_radius = 10;
float ali_str = 0.1;
float coh_radius = 300;
float coh_str = 0.5;
float obs_thres = 100;
float obs_str = 20;
float goal_str = 1;

void update(float dt){
  // update position
  for (int i=0; i<agts; i++){
    agtPos[i].add(PVector.mult(agtVel[i], dt));
  }
  // compute force
  PVector[] f = new PVector[agts];
  for (int i=0; i<agts; i++){
    f[i] = new PVector();
    PVector sep_force = new PVector(); // separation force
    PVector ali_force = new PVector(); // alignment force
    PVector coh_force = new PVector(); // cohesion force
    PVector obs_force = new PVector(); // obstacle force
    PVector goal_force = new PVector(); // goal force
    
    float k_ali = 1;
    int coh_count = 1; // itself
    PVector coh_center = new PVector(); // cohesion group center
    
    for (int j=0; j<agts; j++){
      if (j==i) continue;
      PVector tmp = new PVector();
      float dis = PVector.sub(agtPos[i],agtPos[j]).mag();
      
      // compute separation force
      if (dis <= sep_radius) { // receive separation force from this neighbor
        tmp = PVector.sub(agtPos[i], agtPos[j]).normalize();
        if (8.0/(dis*dis+0.0001) < 50) tmp.mult(8/dis*dis);
        else tmp.mult(50);
        
        tmp.mult(sep_str);
        sep_force.add(tmp);
      }
      // compute alignment force
      if (dis <= ali_radius) { // receive separation force from this neighbor
        tmp = PVector.sub(agtVel[j], agtVel[i]).mult(k_ali);
        if (1.0/(dis*dis+0.0001) < 1) tmp.mult(1/dis*dis);
        else tmp.mult(1);
        
        tmp.mult(ali_str);
        ali_force.add(tmp);
      }
      
      // compute cohesion center
      if (dis <= coh_radius){
        coh_center.add(agtPos[j]);
        coh_count += 1;
      }
    }
    
    // compute cohesion force
    coh_center.add(agtPos[i]);
    coh_center.div(coh_count);
    coh_force = PVector.sub(coh_center, agtPos[i]);
    float dis = coh_force.mag();
    coh_force.normalize();
    if (1/(dis*dis+0.0001) < 100) coh_force.mult(1/dis*dis);
    else coh_force.mult(100);
    coh_force.mult(coh_str);
    
    // compute obstacle force
    float xdis = windowsize - agtPos[i].x;
    float ydis = windowsize - agtPos[i].y;
    if (agtPos[i].x < obs_thres) {
      if (1.0/(agtPos[i].x+0.0001) < 2) obs_force.x += (1.0/agtPos[i].x);
      else obs_force.x += 2;
    }
    else if (xdis < obs_thres) {
      if (1.0/(xdis+0.0001) < 2) obs_force.x -= (1.0/xdis);
      else obs_force.x -= 2;
    } 
    if (agtPos[i].y < obs_thres) {
      if (1.0/(agtPos[i].y+0.0001) < 2) obs_force.y += (1.0/agtPos[i].y);
      else obs_force.y += 2;
    }
    else if (ydis < obs_thres) {
      if (1.0/(ydis+0.0001) < 2) obs_force.y -= (1.0/ydis);
      else obs_force.y -= 2;
    }
    obs_force.mult(obs_str);
    
    //dis = PVector.
    
    
    f[i].add(sep_force).add(ali_force).add(coh_force).add(obs_force);
  }
  
  
  // update speed
  float m = 0.005;
  for (int i=0; i<agts; i++){
    agtVel[i].add(PVector.mult(PVector.mult(f[i], (1/m)), dt));
    //agtPos[i].add(PVector.mult(agtVel[i], dt));
    
    // position robustness check
    
    if ((agtPos[i].x < 0 && agtVel[i].x < 0)
    || (agtPos[i].x > windowsize && agtVel[i].x > 0)) agtVel[i].x *= -0.6;
    if ((agtPos[i].y < 0 && agtVel[i].y < 0)
    || (agtPos[i].y > windowsize && agtVel[i].y > 0)) agtVel[i].y *= -0.6;
    
  }
}


void draw() {
  // Static Scene
  background(0,0,0);
  update(.01);

  // draw Agent
  stroke(255, 255, 255);
  strokeWeight(4);
  for (int i=0; i<agts; i++){
    point(agtPos[i].x, agtPos[i].y);
  }
}

void keyPressed() {
  if (keyCode == ENTER) { //<>//
    init();
    println("New Round");
  }
}
