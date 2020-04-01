//CSCI 5611
//Sheep crowd Simulation 3
//Yuxuan Huang

import java.lang.Math;
import java.util.PriorityQueue;
import java.util.Comparator;

//Description of Obstacle and Agent
int sphobs = 2; // number of spherical obstacles
int agts = 12; //number of agents

PVector[] sphobsPos = new PVector[sphobs]; // Spherical Obstacles positions
PVector[] agtPos = new PVector[agts]; // Agents positions
PVector[] agtVel = new PVector[agts]; // Agents velocities
PVector destPos;
float obsRadius = 4;
float agtRadius;

float roomscale = 600;

//Description of PRM
int numofMS = 100; // number of milestones
float param = 0.5; // neighbor radius parameter
PRM prm;
ArrayList<ArrayList<Integer>> path;

PShape[] sheep = new PShape[agts];
PShape fence;

//Create Window
void setup() {
  size(600, 600, P3D);
  surface.setTitle("Sheeps out of fence");
  camera(0, 300, 450, 0, 0, 0, 0, -1, 0);
  noStroke();
  init();
  for (int i=0; i<agts; i++) sheep[i] = loadShape("sheep.obj");
  fence = loadShape("Fence.obj");
  
}

void init() {
  // initialize spherical obstascles
  sphobsPos[0] = new PVector(60, 40);
  sphobsPos[1] = new PVector(40, -60);
  
  // initialize rectangular obstacles
  
  // initialize agents
  agtPos[0] = new PVector(-100, -120);
  agtPos[1] = new PVector(-80, -80);
  agtPos[2] = new PVector(-70, -40);
  agtPos[3] = new PVector(-100, -60);
  agtPos[4] = new PVector(-20, -90);
  agtPos[5] = new PVector(-40, -120);
  agtPos[6] = new PVector(-50, -60);
  agtPos[7] = new PVector(-120, -80);
  agtPos[8] = new PVector(-180, -40);
  agtPos[9] = new PVector(-150, -60);
  agtPos[10] = new PVector(-120, -55);
  agtPos[11] = new PVector(-200, -120);

  destPos = new PVector(-150, 150);
  agtRadius = 40;
  for (int i=0; i<agts; i++){
    agtVel[i] = new PVector();
  }
  
  prm = new PRM(numofMS);
  
  path = new ArrayList<ArrayList<Integer>>();
  for (int i=0; i<agts; i++){
    path.add(pathSearch(i));
    stepindex[i] = 1;
  }
  
}



int[] stepindex = new int[agts];

float sep_radius = 100;
float sep_str = 50;
float ali_radius = 50;
float ali_str = 20;
float coh_radius = 200;
float coh_str = 50;
float obs_thres = 50;
float obs_str = 20;
float goal_str = 400;
float k_ali = 1.0;

int coh_count = 1; // itself
PVector coh_center = new PVector(); // cohesion group center

void update(float dt){
  // update position
  for (int i=0; i<agts; i++){
    agtPos[i].add(PVector.mult(agtVel[i], dt));
  }
  // compute force
  PVector[] f = new PVector[agts];
  for(int i=0; i<agts; i++){
    f[i] = new PVector();
    PVector sep_force = new PVector(); // separation force
    PVector ali_force = new PVector(); // alignment force
    PVector coh_force = new PVector(); // cohesion force
    PVector obs_force = new PVector(); // obstacle force
    PVector goal_force = new PVector(); // goal force
    
    for (int j=0; j<agts; j++){
      if (j==i) continue;
      PVector tmp = new PVector();
      float dis = PVector.sub(agtPos[i],agtPos[j]).mag();
      
      // compute separation force
      if (dis <= sep_radius) { // receive separation force from this neighbor
        tmp = PVector.sub(agtPos[i], agtPos[j]).normalize();
        if (8000/(dis*dis+0.0001) < 10) tmp.mult(8000/(dis*dis));
        else tmp.mult(10);
        
        tmp.mult(sep_str);
        sep_force.add(tmp);
      }
      
      
      // compute alignment force
      if (dis <= ali_radius) { // receive separation force from this neighbor
        tmp = PVector.sub(agtVel[j], agtVel[i]).mult(k_ali);
        if (1.0/(dis*dis+0.0001) < 1) tmp.mult(1/(dis*dis));
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
    if (1/(dis*dis+0.0001) < 100) coh_force.mult(1/(dis*dis));
    else coh_force.mult(100);
    coh_force.mult(coh_str);
    
    
    // compute obstacle force
    float xdis = roomscale - agtPos[i].x;
    float ydis = roomscale - agtPos[i].y;
    if (agtPos[i].x < obs_thres) {
      if (1.0/(agtPos[i].x+0.0001) < 2) obs_force.x += (1.0/agtPos[i].x);
      else obs_force.x += 2;
    }
    else if (xdis < obs_thres) {
      if (1.0/(xdis+0.0001) < 2) obs_force.x -= (1.0/xdis);
      else obs_force.x -= 2;
    } 
    if (agtPos[i].y < obs_thres) {
      if (1.0/(agtPos[i].y+0.0001) < 1) obs_force.y += (1.0/agtPos[i].y);
      else obs_force.y += 1;
    }
    else if (ydis < obs_thres) {
      if (1.0/(ydis+0.0001) < 1) obs_force.y -= (1.0/ydis);
      else obs_force.y -= 1;
    }
    for (int j=0; j<sphobs; j++){
      PVector tmp = PVector.sub(agtPos[i], sphobsPos[j]);
      dis = tmp.mag();
      tmp.normalize();
      if (dis - obsRadius < obs_thres) {
        if (3000/(dis*dis+0.0001) < 10000) tmp.mult(3000/(dis*dis));
        else tmp.mult(10000);
        obs_force.add(tmp);
      }
    }
    
    if (agtPos[i].x > -agtRadius && agtPos[i].x < agtRadius-280){
      float dis1 = agtPos[i].y - 30;
      float dis2 = agtPos[i].y + 250;
      if (dis1 < obs_thres + agtRadius && dis1 > 0){
        if (50.0/(dis1+0.0001) < 20) obs_force.y += (50.0/dis1);
        else obs_force.x += 20;
      } 
      else if (dis1 > -(obs_thres + agtRadius) && dis1 < 0){
        if (50.0/(dis1+0.0001) < 20) obs_force.y -= (50.0/dis1);
        else obs_force.x -= 20;
      } 
      if (dis2 < obs_thres + agtRadius && dis2 > 0){
        if (50.0/(dis2+0.0001) < 20) obs_force.y += (50.0/dis2);
        else obs_force.x += 20;
      }
      else if (dis2 > -(obs_thres + agtRadius) && dis2 < 0){
        if (50.0/(dis2+0.0001) < 20) obs_force.y -= (50.0/dis2);
        else obs_force.x -= 20;
      }
    }
    
    
    
    obs_force.mult(obs_str);    
    
    // compute goal force
    if (stepindex[i] < path.get(i).size()){ // not reached destination
      PVector dir = PVector.sub(prm.mspos[path.get(i).get(stepindex[i])], agtPos[i]).normalize();
      //agtPos[i].add(PVector.mult(dir,v*dt));
      goal_force = PVector.mult(dir, goal_str);
      
      if (PVector.sub(prm.mspos[path.get(i).get(stepindex[i])], agtPos[i]).mag()<0.05 ||
      (stepindex[i] < path.get(i).size()-1 &&
      accessible(agtPos[i], prm.mspos[path.get(i).get(stepindex[i]+1)])))
      stepindex[i] += 1;
    }
    else goal_force = PVector.mult(PVector.sub(prm.mspos[path.get(i).get(stepindex[i]-1)], agtPos[i]).normalize(), goal_str);
    
    f[i].add(sep_force).add(ali_force).add(coh_force).add(obs_force).add(goal_force);
    //println("sheep", i,": ",f[i].x, f[i].y);
  }
  
  // update speed
  float m = 0.5;
  for (int i=0; i<agts; i++){
    agtVel[i].add(PVector.mult(PVector.mult(f[i], (1/m)), dt));
    if (agtVel[i].mag() > 50) {
      agtVel[i].normalize().mult(50);
    }
    agtPos[i].add(PVector.mult(agtVel[i], dt));
    
    // position robustness check
    /*
    if ((agtPos[i].x < 0 && agtVel[i].x < 0)
    || (agtPos[i].x > roomscale && agtVel[i].x > 0)) agtVel[i].x *= -0.6;
    if ((agtPos[i].y < 0 && agtVel[i].y < 0)
    || (agtPos[i].y > roomscale && agtVel[i].y > 0)) agtVel[i].y *= -0.6;
    */
    
  }
  
}

void draw() {
  // Static Scene
  background(255,255,255);
  ambientLight(128, 128, 128);
  directionalLight(128, 128, 128, -1, 0, -1);
  lightSpecular(0, 0, 0);
  
  update(.01);
  fill(150, 255, 150);
  noStroke();
  beginShape();
  vertex(10*roomscale/(-2.0), 0, 10*roomscale/(2.0));
  vertex(10*roomscale/(2.0), 0, 10*roomscale/(2.0));
  vertex(10*roomscale/(2.0), 0, 10*roomscale/(-2.0));
  vertex(10*roomscale/(-2.0), 0, 10*roomscale/(-2.0));
  endShape(CLOSE);
  
  fill(200, 200, 200);
  // draw spherical obstacles
  for (int i=0; i<sphobs; i++) {
    pushMatrix();
    translate(sphobsPos[i].x, 0, sphobsPos[i].y);
    tree(10, 4, 100);
    popMatrix();
  }
  
  // draw fences
  pushMatrix();
  translate(-2*102, 0, 40);
  longfence();
  translate(0, 0, -280);
  longfence();
  popMatrix();
  pushMatrix();
  translate(-280, 0, -45);
  rotateY(0.5*PI);
  longfence();
  popMatrix();
  
  //shape(fence);
  
  
  // draw Agent
  for (int i=0; i<agts; i++){
    PVector face = agtVel[i];
    //face.normalize();
    float angle = PVector.dot(new PVector(1,0,0), face);
    angle = (float)Math.acos(angle);
    pushMatrix();
    translate(agtPos[i].x, 0, agtPos[i].y);
    scale(20);
    //rotateY(angle);
    shape(sheep[i], 0, 0);
    popMatrix();
  }
  
  
  /*
  // PRM Visualization
  stroke(0, 0, 255);
  strokeWeight(4);
  for (int agt=0; agt<agts; agt++){
    for (int i=0; i<path.get(agt).size()-1; i++){
      line(prm.mspos[path.get(agt).get(i)].x, 0, prm.mspos[path.get(agt).get(i)].y, 
        prm.mspos[path.get(agt).get(i+1)].x, 0, prm.mspos[path.get(agt).get(i+1)].y);
    }
  }
  
  
  for (int i=0; i<numofMS+agts; i++){
    stroke(255, 0, 0);
    strokeWeight(10);
    point(prm.mspos[i].x, 0, prm.mspos[i].y);
    strokeWeight(1);
    
    for (int j=i+1; j<numofMS+agts+1; j++){
      if (prm.adjacent[i][j] == true) line(prm.mspos[i].x, 0, prm.mspos[i].y, 
      prm.mspos[j].x, 0, prm.mspos[j].y);
    }
    
  }
  stroke(255, 255, 255);
  strokeWeight(10);
  point(-280, 0, 0);
  point(0, 0, 0);
  point(-280, 0, -280);
  */
}



//Probablistic Roadmap Class
class PRM{
  int num;
  PVector[] mspos;
  boolean[][] adjacent;
  float[][] distance;
  float[] heuristic;

  PRM(int n){
    num = n;
    mspos = new PVector[num+agts+1];
    //adjlist = new ArrayList<ArrayList<Integer>>(num);
    adjacent = new boolean[num+agts+1][num+agts+1];
    distance = new float[num+agts+1][num+agts+1];
    heuristic = new float[num+agts+1];
    
    // Generate random positions
    for (int i=0; i<num; i++){
      mspos[i] = new PVector(roomscale*((float)Math.random()-0.5), 
      roomscale*((float)Math.random()-0.5));
      
      // robustness check against boundaries
      float boundary = 0.5*roomscale - agtRadius;
      if (mspos[i].x>boundary) mspos[i].x = boundary;
      if (mspos[i].y>boundary) mspos[i].y = boundary;
      if (mspos[i].x<(-1*boundary)) mspos[i].x = -1*boundary;
      if (mspos[i].y<(-1*boundary)) mspos[i].y = -1*boundary;
      // robustness check against spherical obstacles
      for (int j=0; j<sphobs; j++) {
        PVector dir = PVector.sub(mspos[i], sphobsPos[j]);
        float r = agtRadius + obsRadius; // sphobs radius in the configuration space
        if (dir.mag()<r){
          dir.normalize();
          mspos[i] = PVector.add(sphobsPos[j], PVector.mult(dir, 1.2*r));
        }
      }
      /*
      // robustness check against rectangular obstacles
      for (int j=0; j<rectobs; j++) {
        PVector dir = PVector.sub(mspos[i], rectobsPos[j]);
        float x_bound = obsSize[j].x/2.0 + agtRadius;
        float y_bound = obsSize[j].y/2.0 + agtRadius;
        if (dir.x < x_bound && dir.x > (-1)*x_bound && dir.y < y_bound && dir.y > (-1)*y_bound){
          if ((obsSize[j].x/2.0 - Math.abs(dir.x)) < (obsSize[j].y/2.0 - Math.abs(dir.y))){
            if (dir.x > 0) mspos[i].x = rectobsPos[j].x + x_bound * 1.2;
            else mspos[i].x = rectobsPos[j].x - x_bound * 1.2;
          }
          if (dir.y > 0) mspos[i].y = rectobsPos[j].y + y_bound * 1.2;
          else mspos[i].y = rectobsPos[j].y - y_bound * 1.2;
        }
      }*/
    }
    
    // Add start and destination to the graph
    for(int i=0; i<agts; i++){
      mspos[num + i] = new PVector(agtPos[i].x,agtPos[i].y); //starts
    }
    mspos[num + agts] = new PVector(destPos.x,destPos.y); //dest
    
    // Compute adjacency list and edge distance
    float dis;
    // intermediate nodes
    for (int i=0; i<num; i++){
      adjacent[i][i] = false;
      for (int j=i+1; j<num; j++){
        dis = PVector.sub(mspos[i], mspos[j]).mag();
        if(dis<param*roomscale && accessible(mspos[i], mspos[j])) {
          adjacent[i][j] = true;
          adjacent[j][i] = true;
          distance[i][j] = dis;
          distance[j][i] = dis;
        }
        else adjacent[i][j] = false;
      }
      heuristic[i] = PVector.sub(destPos, mspos[i]).mag();
      for (int j=0; j<agts; j++){
        if (accessible(mspos[i], agtPos[j])) {
          dis = PVector.sub(mspos[i], agtPos[j]).mag();
          adjacent[i][num + j] = true;
          adjacent[num + j][i] = true;
          distance[i][num + j] = dis;
          distance[num + j][i] = dis;
        }
      }
      if (accessible(mspos[i], destPos)) {
        dis = heuristic[i];
        adjacent[i][num+agts] = true;
        adjacent[num+agts][i] = true;
        distance[i][num+agts] = dis;
        distance[num+agts][i] = dis;
      } 
    }
    
    // start and dest
    for (int i=0; i<agts; i++){
      for (int j=0; j<agts; j++) adjacent[num+i][num+j]=false;
      if (accessible(agtPos[i], destPos)){
        adjacent[num+i][num+agts] = true;
        adjacent[num+agts][num+i] = true;
      }
      else{
        adjacent[num+i][num+agts] = false;
        adjacent[num+agts][num+i] = false;
      }
    }
    adjacent[num+agts][num+agts] = false;
  }
}

boolean accessible(PVector pos1, PVector pos2){
    // check against spherical obstacles
    for (int i=0; i<sphobs; i++){
      float r = agtRadius + obsRadius;
      PVector v1 = PVector.sub(sphobsPos[i], pos1);
      PVector v2 = PVector.sub(sphobsPos[i], pos2);
      PVector v3 = PVector.sub(pos1, pos2);
      float len = v3.mag();
      v3.normalize();
      PVector v4 = new PVector();
      PVector.mult(v3, -1, v4);
      PVector tmp = new PVector();
      PVector.cross(v3, v2, tmp);
      float len1 = Math.abs(PVector.dot(v1, v4));
      float len2 = Math.abs(PVector.dot(v2, v3));
      if (tmp.mag()<r && (len1 + len2 - len < 0.1)) return false;
    }
    // check against rectangular obstacles
    
    if (line_intersect(pos1,pos2,new PVector(-(280+agtRadius), 30+agtRadius),new PVector(+agtRadius, 30+agtRadius))) return false; //<>//
    
    if (line_intersect(pos1,pos2,new PVector(-(280+agtRadius),-(250+agtRadius)),new PVector(-(280+agtRadius),30+agtRadius))) return false;
    
    if (line_intersect(pos1,pos2,new PVector(+agtRadius,-(250+agtRadius)),new PVector(-(280+agtRadius),-(250+agtRadius)))) return false;
    
    return true;
}

boolean line_rect_intersect(PVector pos1, PVector pos2, PVector obsPos, PVector obsSize){
  float xlow = obsPos.x - obsSize.x/2.0 - agtRadius;
  float xhigh = obsPos.x + obsSize.x/2.0 + agtRadius;
  float ylow = obsPos.y - obsSize.y/2.0 - agtRadius;
  float yhigh = obsPos.y + obsSize.y/2.0 + agtRadius;
  boolean linx1, rinx1, uiny1, diny1, linx2, rinx2, uiny2, diny2;
  if (pos1.x > xlow) linx1 = true;
  else linx1 = false;
  if (pos1.x < xhigh) rinx1 = true;
  else rinx1 = false;
  if (pos1.y > ylow) diny1 = true;
  else diny1 = false;
  if (pos1.y < yhigh) uiny1 = true;
  else uiny1 = false;
  if (pos2.x > xlow) linx2 = true;
  else linx2 = false;
  if (pos2.x < xhigh) rinx2 = true;
  else rinx2 = false;
  if (pos2.y > ylow) diny2 = true;
  else diny2 = false;
  if (pos2.y < yhigh) uiny2 = true;
  else uiny2 = false;
  
  if ((!linx1 && !linx2)||(!rinx1 && !rinx2)||(!uiny1 && !uiny2)||(!diny1 && !diny2)) return false;
  if ((linx1&&rinx1&&linx2&&rinx2)&&((!diny1&&!uiny2)||(!uiny1&&!diny2)) || 
  (diny1&&uiny1&&diny2&&uiny2)&&((!linx1&&!rinx2)||(!rinx1&&!linx2))) return true;
  
  if (line_intersect(pos1, pos2, new PVector(xlow,ylow), new PVector(xhigh, ylow))||
  line_intersect(pos1, pos2, new PVector(xlow,yhigh), new PVector(xlow, ylow))||
  line_intersect(pos1, pos2, new PVector(xhigh,yhigh), new PVector(xhigh, ylow))) return true;
  else return false;
}

boolean line_intersect(PVector s1, PVector e1, PVector s2, PVector e2){
  PVector v1 = PVector.sub(e1, s1);
  PVector v2 = PVector.sub(s2, s1);
  PVector v3 = PVector.sub(e2, s1);
  PVector r1 = new PVector();
  PVector r2 = new PVector();
  PVector.cross(v1, v2, r1);
  PVector.cross(v1, v3, r2);
  if (r1.z * r2.z > 0) return false;
  v1 = PVector.sub(e2, s2);
  v2 = PVector.sub(s1, s2);
  v3 = PVector.sub(e1, s2);
  PVector.cross(v1, v2, r1);
  PVector.cross(v1, v3, r2); 
  if (r1.z * r2.z > 0) return false;
  return true;
}


class Step implements Comparable<Step>{
  int key;
  float cost; // cost so far
  float ttcost; // cost + heuristic
  
  Step(int k, float c, float ttc){
    key = k;
    cost = c;
    ttcost = ttc;
  }
  public int compareTo(Step s){
    return int(ttcost-s.ttcost);
  }
  
}

//Path-searching function
ArrayList<Integer> pathSearch(int agt_index){
  ArrayList<Integer> result = new ArrayList<Integer>();
  int[] pstep = new int[numofMS+agts+1];
  for (int i=0; i<numofMS+agts+1; i++) pstep[i] = -2; // pstep = -2 mean not visited before
  PriorityQueue<Step> fringe = new PriorityQueue<Step>();
  
  
  // first step
  for (int i = 0; i<numofMS+agts+1; i++){
    if (prm.adjacent[numofMS+agt_index][i]) {
      fringe.add(new Step(i, prm.distance[numofMS+agt_index][i], prm.distance[numofMS+agt_index][i]+prm.heuristic[i]));
      pstep[i] = -1; // pstep = -1 mean it is the first step
    }
  }
  while(true){
    Step currentStep = fringe.poll();
    if (currentStep.key == numofMS+agts) break;
    for (int i = 0; i<numofMS+agts+1; i++){
      if (prm.adjacent[currentStep.key][i]){ // if adjacent
        if (pstep[i] == -2) { // if i not reached before
          fringe.add(new Step(i, currentStep.cost + prm.distance[currentStep.key][i],
          currentStep.cost + prm.distance[currentStep.key][i]+prm.heuristic[i]));
          pstep[i] = currentStep.key;
        }
      }
    }  
  }
  int tmp = numofMS+agts;
  while(tmp != -1){
    result.add(0, tmp);
    tmp = pstep[tmp];
  }
  result.add(0,numofMS+agt_index);
  return result;
}



//A function to draw a cylinder given the radius, height, and number of sides
void tree(int sides, float r, float h)
{
    float angle = 360 / sides;
    // draw top shape
    fill(193, 154, 107);
    /*
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float z = sin( radians( i * angle ) ) * r;
        vertex( x, h, z );    
    }
    endShape(CLOSE);
    // draw bottom shape
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float z = sin( radians( i * angle ) ) * r;
        vertex( x, 0, z );    
    }
    endShape(CLOSE);
    */
    // draw body
    beginShape(TRIANGLE_STRIP);
    for (int i = 0; i < sides + 1; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float z = sin( radians( i * angle ) ) * r;
        vertex( x, h, z);
        vertex( x, 0, z);    
    }
    endShape(CLOSE);
    pushMatrix();
    translate(0, 3*h/4, 0);
    fill(0, 255, 0);
    box(h/2);
    popMatrix();
}

void longfence(){
  pushMatrix();
  rotateY(0.5*PI);
  shape(fence, 0, 0);
  popMatrix();  
  pushMatrix();
  translate(95.0,0,0);
  rotateY(0.5*PI);
  shape(fence, 0, 0);
  popMatrix();
  pushMatrix();
  translate(-95.0,0,0);
  rotateY(0.5*PI);
  shape(fence, 0, 0);
  popMatrix();
}

void keyPressed() {
  if (keyCode == ENTER) {
    camera(0, 100, 0, 0, 0, 0, 0, 0, 1);
    ortho();
  }
  else if (keyCode == SHIFT) {
    camera(0, 300, 450, 0, 0, 0, 0, -1, 0);
    perspective();
  }
  else if (keyCode == ALT) {
    init();
    println("New Round");
  }
}
