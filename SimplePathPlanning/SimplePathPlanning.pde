//CSCI 5611
//Simple Path Finding Code for checkin
//Yuxuan Huang

import java.lang.Math;
import java.util.PriorityQueue;
import java.util.Comparator;

//Description of Obstacle and Agent
PVector obsPos = new PVector(0, 0);
PVector agtPos;
PVector destPos = new PVector(9, 9);
float obsRadius = 2;
float agtRadius = 0.5;
float roomscale = 20;

//Description of PRM
int numofMS = 20; // number of milestones
float param = 0.5; // neighbor radius parameter
PRM prm;
ArrayList<Integer> path;
boolean destReached;

//Create Window
void setup() {
  size(600, 600, P3D);
  surface.setTitle("Simple Path Planning");
  camera(0, 0, 260, 0, 0, 0, 0, -1, 0);
  init();
}

void init() {
  agtPos = new PVector(-9, -9);
  prm = new PRM(numofMS);
  destReached = false;
  path = pathSearch();
  stepindex = 1;
}

float v = 10;
int stepindex;
void update(float dt){
  if (stepindex < path.size()){ // not reached destination
    PVector dir = PVector.sub(prm.mspos[path.get(stepindex)], agtPos).normalize();
    agtPos.add(PVector.mult(dir,v*dt));
    if (PVector.sub(prm.mspos[path.get(stepindex)], agtPos).mag()<0.05 ||
    (stepindex < path.size()-1 &&
    accessible(agtPos, prm.mspos[path.get(stepindex+1)],obsRadius+agtRadius)))
    stepindex += 1;
  }
}

//Draw the scene: one sphere per mass, one line connecting each pair
void draw() {
  // Static Scene
  background(255,255,255);
  ambientLight(128, 128, 128);
  directionalLight(128, 128, 128, -1, 0, -1);
  lightSpecular(0, 0, 0);
  update(.01);
  noFill();
  stroke(200);
  box(roomscale*10,100,roomscale*10); // 1 unit for 0.1 m
  noStroke();
  fill(200, 200, 200);
  cylinder(180, obsRadius*10, 100);
  // Agent
  fill(249, 236, 228);
  pushMatrix();
  translate(10*agtPos.x, -30, 10*agtPos.y);
  cylinder(180, agtRadius*10, 40);
  popMatrix();
  
  // PRM Visualization
  stroke(0, 255, 0);
  strokeWeight(4);
  for (int i=0; i<path.size()-1; i++){
    line(10*prm.mspos[path.get(i)].x, -50, 10*prm.mspos[path.get(i)].y, 
      10*prm.mspos[path.get(i+1)].x, -50, 10*prm.mspos[path.get(i+1)].y);
  }
  for (int i=0; i<numofMS+2; i++){
    stroke(255, 0, 0);
    strokeWeight(4);
    point(10*prm.mspos[i].x, -50, 10*prm.mspos[i].y);
    strokeWeight(1);
    for (int j=i+1; j<numofMS+2; j++){
      if (prm.adjacent[i][j] == true) line(10*prm.mspos[i].x, -50, 10*prm.mspos[i].y, 
      10*prm.mspos[j].x, -50, 10*prm.mspos[j].y);
    }
  }
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
    mspos = new PVector[num+2];
    //adjlist = new ArrayList<ArrayList<Integer>>(num);
    adjacent = new boolean[num+2][num+2];
    distance = new float[num+2][num+2];
    heuristic = new float[num+2];
    float r = obsRadius + agtRadius;
    
    // Generate random positions
    for (int i=1; i<num+1; i++){
      mspos[i] = new PVector(roomscale*((float)Math.random()-0.5), 
      roomscale*((float)Math.random()-0.5));
      // robustness check
      float boundary = 0.5*roomscale - agtRadius;
      if (mspos[i].x>boundary) mspos[i].x = boundary;
      if (mspos[i].y>boundary) mspos[i].y = boundary;
      if (mspos[i].x<(-1*boundary)) mspos[i].x = -1*boundary;
      if (mspos[i].y<(-1*boundary)) mspos[i].y = -1*boundary;
      PVector dir = PVector.sub(mspos[i], obsPos);
      if (dir.mag()<r){
        dir.normalize();
        mspos[i] = PVector.add(obsPos, PVector.mult(dir, 1.2*r));
      }
      
    }
    // Add start and destination to the graph
    mspos[0] = new PVector(agtPos.x,agtPos.y);
    mspos[num+1] = new PVector(destPos.x,destPos.y);  
    
    // Compute adjacency list and edge distance
    float dis;
    for (int i=1; i<num+1; i++){
      for (int j=1; j<num+1; j++){
        dis = PVector.sub(mspos[i], mspos[j]).mag();
        if (j == i) adjacent[i][j] = false;
        else if(adjacent[j][i] == true || 
        (dis<param*roomscale && accessible(mspos[i], mspos[j], r))) {
          adjacent[i][j] = true;
          distance[i][j] = dis;
        }
        else adjacent[i][j] = false;
      }
      heuristic[i] = PVector.sub(destPos, mspos[i]).mag();
      if (accessible(mspos[i], agtPos, r)) {
        dis = PVector.sub(mspos[i], agtPos).mag();
        adjacent[i][0] = true;
        adjacent[0][i] = true;
        distance[i][0] = dis;
        distance[0][i] = dis;
      }
      if (accessible(mspos[i], destPos, r)) {
        dis = heuristic[i];
        adjacent[i][num+1] = true;
        adjacent[num+1][i] = true;
        distance[i][num+1] = dis;
        distance[num+1][i] = dis;
      } 
    }
  
    adjacent[0][0] = false;
    adjacent[num+1][num+1] = false;
    if (accessible(agtPos, destPos, r)){
      adjacent[0][num+1] = true;
      adjacent[num+1][0] = true;
    }
    else{
      adjacent[0][num+1] = false;
      adjacent[num+1][0] = false;
    }
  }
}

boolean accessible(PVector pos1, PVector pos2, float r){
    PVector v1 = PVector.sub(obsPos, pos1);
    PVector v2 = PVector.sub(obsPos, pos2);
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
ArrayList<Integer> pathSearch(){
  ArrayList<Integer> result = new ArrayList<Integer>();
  int[] pstep = new int[numofMS+2];
  PriorityQueue<Step> fringe = new PriorityQueue<Step>();
  
  // first step
  for (int i = 1; i<numofMS+2; i++){
    if (prm.adjacent[0][i]) {
      fringe.add(new Step(i, prm.distance[0][i], prm.distance[0][i]+prm.heuristic[i]));
      pstep[i] = -1;
    }
  }
  while(true){
    Step currentStep = fringe.poll();
    if (currentStep.key == numofMS+1) break;
    for (int i = 1; i<numofMS+2; i++){
      if (prm.adjacent[currentStep.key][i]){ // if adjacent
        if (pstep[i] == 0) { // if i not reached before
          fringe.add(new Step(i, currentStep.cost + prm.distance[currentStep.key][i],
          currentStep.cost + prm.distance[currentStep.key][i]+prm.heuristic[i]));
          pstep[i] = currentStep.key;
        }
      }
    }  
  }
  int tmp = numofMS+1;
  while(tmp != -1){
    result.add(0, tmp);
    tmp = pstep[tmp];
  }
  result.add(0,0);
  return result;
}

//A function to draw a cylinder given the radius, height, and number of sides
void cylinder(int sides, float r, float h)
{
    float angle = 360 / sides;
    float halfHeight = h / 2;
    // draw top shape
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float z = sin( radians( i * angle ) ) * r;
        vertex( x, -halfHeight, z );    
    }
    endShape(CLOSE);
    // draw bottom shape
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float z = sin( radians( i * angle ) ) * r;
        vertex( x, halfHeight, z );    
    }
    endShape(CLOSE);
    // draw body
    beginShape(TRIANGLE_STRIP);
    for (int i = 0; i < sides + 1; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float z = sin( radians( i * angle ) ) * r;
        vertex( x, halfHeight, z);
        vertex( x, -halfHeight, z);    
    }
    endShape(CLOSE);
}

void keyPressed() {
  if (keyCode == ENTER) {
    camera(0, 100, 0, 0, 0, 0, 0, 0, 1);
    ortho();
  }
  else if (keyCode == SHIFT) {
    camera(0, 0, 260, 0, 0, 0, 0, -1, 0);
    perspective(); //<>//
  }
  else if (keyCode == ALT) init();
}
