// Rabbit and Dogs
// Dogs = psi, rabbit = zajic
// You can add new dogs by clicking mouse
// By Jaroslav Trnka, 2020
// https://github.com/JaroslavTrnka1
// jaroslav_trnka@centrum.cz

// Movement of dogs code taken from:
// Daniel Shiffman: Nature of Code
// https://natureofcode.com/

vehicleSystem psi;
Vehicle2 zajic;

void setup() {
  size(1000,1000);
  psi = new vehicleSystem();
  zajic = new Vehicle2();
  psi.addVehicle();
}

void draw() {
  background(255);
  zajic.escape(psi);
  zajic.update();
  zajic.display();
  psi.seek(zajic);
  psi.run();
}

void mousePressed() {
  psi.addVehicle();
}
  
class vehicleSystem {
  ArrayList<Vehicle> vehicles;
  
  vehicleSystem() {
    vehicles = new ArrayList();
  }
  
  void addVehicle() {
    vehicles.add(new Vehicle());
  }
  
  void seek(Vehicle2 p) {
    for (Vehicle v: vehicles) {
      v.seek(p.location);
    }
  }
  
  void run() {
    for (Vehicle v: vehicles) {
      v.update();
      v.display();
    }
  }
}

  
class Vehicle {

  PVector location;
  PVector velocity;
  PVector acceleration;
  // Additional variable for size
  float r;
  float maxforce;
  float maxspeed;
  float sensitivity;
  PVector previous;

  Vehicle() {
    acceleration = new PVector(0,0);
    velocity = new PVector(2,2);
    location = new PVector(mouseX, mouseY);
    r = 8.0;
    //[full] Arbitrary values for maxspeed and
    // force; try varying these!
    maxspeed = random(1,6);
    maxforce = random(0.1,0.3);
    sensitivity = 0;
    previous = new PVector(0,0);
    //[end]
  }

  // Our standard “Euler integration” motion model
  void update() {
    velocity.add(acceleration);
    velocity.limit(maxspeed);
    location.add(velocity);
    acceleration.mult(0);
    //println(acceleration);
  }

  // Newton’s second law; we could divide by mass if we wanted.
  void applyForce(PVector force) {
    acceleration.add(force);
    //println(acceleration);
  }

  // Our seek steering force algorithm
  void seek(PVector target) {
    PVector track = PVector.sub(target, previous);
    track.mult(sensitivity);
    PVector predicted = PVector.add(target, track);
    PVector desired = PVector.sub(predicted,location);
    desired.normalize();
    desired.mult(maxspeed);
    PVector steer = PVector.sub(desired,velocity);
    steer.limit(maxforce);
    applyForce(steer);
    previous = target;
  }

  void display() {
    // Vehicle is a triangle pointing in
    // the direction of velocity; since it is drawn
    // pointing up, we rotate it an additional 90 degrees.
    float theta = velocity.heading() + PI/2;
    fill(maxspeed * 40);
    stroke(0);
    pushMatrix();
    translate(location.x,location.y);
    rotate(theta);
    beginShape();
    vertex(0, -r*4);
    vertex(-r, -r*2);
    vertex(-r, r);
    vertex(r, r);
    vertex(r, -r*2);
    endShape(CLOSE);
    popMatrix();
  }
}

class Vehicle2 {

  PVector location;
  PVector velocity;
  PVector acceleration;
  // Additional variable for size
  float r;
  float maxforce;
  float maxspeed;
  float sensitivity;
  PVector [] previous = new PVector[20];
  
Vehicle2() {
    acceleration = new PVector(0,0);
    velocity = new PVector(0,0);
    location = new PVector(random(width), random (height));
    r = 6.0;
    //[full] Arbitrary values for maxspeed and
    // force; try varying these!
    sensitivity = 10;
    maxspeed = 8;
    maxforce = 0.4;
    for (int i = 0; i < 20; i++) {
      previous[i] = new PVector(0,0);
     }
    //[end]
  }

  // Our standard “Euler integration” motion model
  void update() {
    velocity.add(acceleration);
    velocity.limit(maxspeed);
    location.add(velocity);
    acceleration.mult(0);
  }

  // Newton’s second law; we could divide by mass if we wanted.
  void applyForce(PVector force) {
    acceleration.add(force);
  }

  // Our seek steering force algorithm
  void escape(vehicleSystem vs) {
    float safeAngle = 0;
    PVector danger = new PVector (0,0);
    int i = 0;
    float [] source = new float [vs.vehicles.size()];
    float [] distance = new float [vs.vehicles.size()];
    //println(previous[i]);
    PVector dangertarget = new PVector(0,0);
    //sensitivity = map(mouseX, 0, width, 0, 100);
    // smyčka na načtení předpokládaných pozic psů do vektoru úhlů (v radiánech) source
    for (Vehicle v: vs.vehicles) {
      dangertarget = PVector.sub(new PVector (v.location.x, v.location.y), location);
      //println(previous[i]);
      PVector track = PVector.sub(dangertarget, previous[i]);
      track.mult(sensitivity);
      PVector predicted = PVector.add(dangertarget, track);
      distance [i]= predicted.mag()/100;
      //println(distance);
      predicted.normalize();
      source [i] = predicted.heading();
      //println(source[i]);
      predicted.mult(100/(distance[i]));
      danger.add(predicted);
      previous[i] = dangertarget;
      i++;
    }
    float y, r, pr, fr;
    float dangerInAngle;
    float highestsafety = 10000;
    for (float j = 0; j < (100 * TWO_PI); j++) { //smyčka pro úhly 0 až TWO_PI, která prověří jejich bezpečnost
      dangerInAngle = 0;
      for (int k = 0; k < vs.vehicles.size(); k++) {  //smyčka pro daný úhel v radiánech, která prověří v tomto úhlu nebezpečí jednotlivých psů a sečte to
        r = exp(-(map(abs(source[k] - j/100), 0, PI/distance[k], 5, -6)));
        //println(r);
        pr = exp(-(map(abs(source[k] - j/100 + TWO_PI), 0, PI/distance[k], 5, -6)));
        fr = exp(-(map(abs(source[k] - j/100 - TWO_PI), 0, PI/distance[k], 5, -6)));
        y = 1/(1 + r) + 1/(1 + pr) + 1/(1+ fr);
        dangerInAngle += (y/distance[k]);
        //println(dangerInAngle);
      }
      float [] distanceOfCorners = {(width - location.x)/100, (height - location.y)/100, location.x/100, location.y/100};
      for (int m = 0; m < 4; m++) { //nebezpečí okrajů - source úhel budou osy, vzdálenost se vypočte z polohy
        r = exp(-(map(abs(m * PI/2 - j/100), 0, PI/distanceOfCorners[m], 5, -6)));
        pr = exp(-(map(abs(m * PI/2 - j/100 + TWO_PI), 0, PI/distanceOfCorners[m], 5, -6)));
        fr = exp(-(map(abs(m * PI/2 - j/100 - TWO_PI), 0, PI/distanceOfCorners[m], 5, -6)));
        y = 1/(1 + r) + 1/(1 + pr) + 1/(1+ fr);
        dangerInAngle += (0.001*y/distanceOfCorners[m]);
      }
      //println(j);
      //if(j ==600) {println(highestsafety);}
      if (dangerInAngle < highestsafety) {
        highestsafety = dangerInAngle;
        safeAngle = j/100; //rozsah 0 - TWO_PI * 100
        //println(j);
      }
    }
    PVector desired = PVector.fromAngle(safeAngle);
    //println(safeAngle);
    //PVector desired = danger.mult(-1);
    corners(desired);
    desired.normalize();
    desired.mult(maxspeed);
    PVector steer = PVector.sub(desired,velocity);
    steer.limit(maxforce);
    applyForce(steer);
  }
  
    void corners(PVector h) {
    int closeness = 50;
    if (location.x < closeness) {
      h.add(new PVector(sq(closeness - location.x), 0));
    }
    if (location.x > width-closeness) {
      h.add(new PVector(-sq(location.x - (width - closeness)), 0));
    }
    if (location.y < closeness) {
      h.add(new PVector(0, sq(closeness - location.y)));
    }
    if (location.y > height-closeness) {
      h.add(new PVector(0, -sq(location.y - (height - closeness))));
    }
    }

  void display() {
    // Vehicle is a triangle pointing in
    // the direction of velocity; since it is drawn
    // pointing up, we rotate it an additional 90 degrees.
    float theta = velocity.heading() + PI/2;
    fill(255);
    stroke(0);
    pushMatrix();
    translate(location.x,location.y);
    rotate(theta);
    beginShape();
    vertex(0, -r*4);
    vertex(-r, -r*2);
    vertex(-r, r);
    vertex(r, r);
    vertex(r, -r*2+2);
    vertex(r+5, -r*2-1);
    vertex(r+5, -r*2);
    vertex(r, -r*2);
    endShape(CLOSE);
    popMatrix();
  }
}
