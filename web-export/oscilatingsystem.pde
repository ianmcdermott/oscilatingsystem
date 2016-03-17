
import ddf.minim.*;
import ddf.minim.ugens.*;

Minim minim;
AudioOutput out;

Oscil      wave;
Frequency  currentFreq;



boolean debug = false;
int numPoints = 25;
PImage sprite;

boolean dPath = false;
Path path;
float noise = 0;
float noiseX= 0;
float noiseY= 110000;
boolean noiseOn = false;

VehicleSystem vs;

float offset = 0;

void setup() {


  ////////MINIM SETUP ////////
  minim = new Minim(this);
  out   = minim.getLineOut();

  currentFreq = Frequency.ofHertz( 432 );
  wave = new Oscil( currentFreq, 0.6f, Waves.SINE );

  wave.patch( out );

  // size(1500, 1200, P2D);
  fullScreen(1);
  smooth();
  // Call a function to generate new Path object
  //newPath();
  path = new Path();
  for (int i = 0; i <numPoints; i++) {
    randomPath();
  }
  // We are now making random vehicles and storing them in an ArrayList
  vs = new VehicleSystem();
  background(0);
  frameRate(60);
  sprite = loadImage("sprite.png");
}

void draw() {
  fill(0, 50);
  //background(255);
  float oscillate = map(sin(offset), -1, 1, -9, 4);
  println(oscillate);

  rect(0, 0, width, height);
  /* if (noiseOn = true) {
   noiseX = map(noise(noise), -1, 1, -2, 2);
   noiseY = map(noise(noise), -1, 1, -2, 2);
   path.updatePoints(noiseX, noiseY);
   } else { */
  noiseX = map(sin(noise), -1, 1, -1, 1);
  noiseY = map(cos(noise), -1, 1, -1, 1);
  offset+= 100;
  path.updatePoints(noiseX, noiseY);
  //}
  // Display the path

  if (dPath) path.display();


  // Instructions
  fill(0);
  textAlign(CENTER);
  text("Hit 'd' to toggle debugging lines.\nClick the mouse to generate new vehicles.", width/2, height-20);

  noise+= .1;

  vs.run(oscillate);
  
  ////// MINIM DRAW ///////
    



}

void newPath() {
  // A path is a series of connected points
  // A more sophisticated path might be a curve
  path = new Path();
  float offset = 30;
  path.addPoint(offset, offset);
  path.addPoint(width-offset, offset);
  path.addPoint(width-offset, height-offset);
  path.addPoint(width/2, height-offset*3);
  path.addPoint(offset, height-offset);
}

void randomPath() {
  path.addPoint(random(50, width-50), random(50, height-50));
}



void keyPressed() {
  if (key == 'd') debug = !debug;
  if (key == 'p') dPath = !dPath;
  if (key == 'n') noiseOn = !noiseOn;
}
/*
void mousePressed() {
 vs.mouseVehicle(mouseX, mouseY);
 }*/

class Path {

  // A Path is an arraylist of points (PVector objects)
  ArrayList<PVector> points;
  // A path has a radius, i.e how far is it ok for the boid to wander off
  float radius;

  Path() {
    // Arbitrary radius of 20
    radius = 50;
    points = new ArrayList<PVector>();
  }

  // Add a point to the path
  void addPoint(float x, float y) {
    PVector point = new PVector(x, y);
    points.add(point);
  }

  // Draw the path
  void display() {
    strokeJoin(ROUND);
    
    // Draw thick line for radius
    stroke(175);
    strokeWeight(radius*2);
    noFill();
    beginShape();
    for (PVector v : points) {
      vertex(v.x, v.y);
    }
    endShape(CLOSE);
    // Draw thin line for center of path
    stroke(0);
    strokeWeight(1);
    noFill();
    beginShape();
    for (PVector v : points) {
      vertex(v.x, v.y);
    }
    endShape(CLOSE);
  }
  
  void updatePoints(float bounceX, float bounceY){
     PVector bounce = new PVector(bounceX, bounceY);
     for(int i = 0; i < points.size(); i++){
       points.get(i).add(bounce);
       if(i % 2 == 0) bounce.mult(-1);
          //    if(i % 4 == 0) bounce.mult(1.2);

     }
  }
}

class Vehicle {

  // All the usual stuff
  PVector location;
  PVector velocity;
  PVector acceleration;
  float r;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed
  int c;
  
  float maxRadius, minRadius;
  
  float freqArray[] = { 432.00, 544.29, 647.27, 864.00, 1088.57, 1294.54};

  // Constructor initialize all values
  Vehicle( PVector l, float ms, float mf, float rad, int col) {
    location = l.get();
    r = rad;
    c = col;
    maxspeed = ms;
    maxforce = mf;
    acceleration = new PVector(0, 0);
    velocity = new PVector(random(-ms, ms), random(-ms, ms));

    maxRadius = r;
    minRadius = r/1.1;
    
  }

  // A function to deal with path following and separation
  void applyBehaviors(ArrayList vehicles, Path path) {
    // Follow path force
    PVector f = follow(path);
    // Separate from other boids force
    PVector s = separate(vehicles);
    // Arbitrary weighting
    f.mult(3);
    s.mult(1);
    // Accumulate in acceleration
    applyForce(f);
    applyForce(s);
  }

  void applyBehaviors(ArrayList vehicles, VehicleArray prevVehicles) {
    // Follow path force
    PVector f = follow(prevVehicles);
    // Separate from other boids force
    PVector s = separate(vehicles);
    // Arbitrary weighting
    f.mult(3);
    s.mult(1);
    // Accumulate in acceleration
    applyForce(f);
    applyForce(s);
  }

  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    acceleration.add(force);
  }



  // Main "run" function
  public void run(float o) {
    update(o);
    borders();
    render();
  }


  // This function implements Craig Reynolds' path following algorithm
  // http://www.red3d.com/cwr/steer/PathFollow.html
  PVector follow(Path p) {

    // Predict location 25 (arbitrary choice) frames ahead
    PVector predict = velocity.get();
    predict.normalize();
    predict.mult(25);
    PVector predictLoc = PVector.add(location, predict);

    // Now we must find the normal to the path from the predicted location
    // We look at the normal for each line segment and pick out the closest one
    PVector normal = null;
    PVector target = null;
    float worldRecord = 1000000;  // Start with a very high worldRecord distance that can easily be beaten

    // Loop through all points of the path
    for (int i = 0; i < p.points.size(); i++) {

      // Look at a line segment
      PVector a = p.points.get(i);
      PVector b = p.points.get((i+1)%p.points.size()); // Note Path has to wraparound

      // Get the normal point to that line
      PVector normalPoint = getNormalPoint(predictLoc, a, b);

      // Check if normal is on line segment
      PVector dir = PVector.sub(b, a);
      // If it's not within the line segment, consider the normal to just be the end of the line segment (point b)
      //if (da + db > line.mag()+1) {
      if (normalPoint.x < min(a.x, b.x) || normalPoint.x > max(a.x, b.x) || normalPoint.y < min(a.y, b.y) || normalPoint.y > max(a.y, b.y)) {
        normalPoint = b.get();
        // If we're at the end we really want the next line segment for looking ahead
        a = p.points.get((i+1)%p.points.size());
        b = p.points.get((i+2)%p.points.size());  // Path wraps around
        dir = PVector.sub(b, a);
      }

      // How far away are we from the path?
      float d = PVector.dist(predictLoc, normalPoint);
      // Did we beat the worldRecord and find the closest line segment?
      if (d < worldRecord) {
        worldRecord = d;
        normal = normalPoint;

        // Look at the direction of the line segment so we can seek a little bit ahead of the normal
        dir.normalize();
        // This is an oversimplification
        // Should be based on distance to path & velocity
        dir.mult(25);
        target = normal.get();
        target.add(dir);
      }
    }

    // Draw the debugging stuff
    if (debug) {
      // Draw predicted future location
      stroke(0);
      fill(0);
      line(location.x, location.y, predictLoc.x, predictLoc.y);
      ellipse(predictLoc.x, predictLoc.y, 4, 4);

      // Draw normal location
      stroke(0);
      fill(0);
      ellipse(normal.x, normal.y, 4, 4);
      // Draw actual target (red if steering towards it)
      line(predictLoc.x, predictLoc.y, target.x, target.y);
      if (worldRecord > p.radius) fill(255, 0, 0);
      noStroke();
      ellipse(target.x, target.y, 8, 8);
    }

    // Only if the distance is greater than the path's radius do we bother to steer
    if (worldRecord > p.radius*2) {
      return seek(target);
    } else {
      return new PVector(0, 0);
    }
  }


  PVector follow(VehicleArray v) {

    // Predict location 25 (arbitrary choice) frames ahead
    PVector predict = velocity.get();
    predict.normalize();
    predict.mult(25);
    PVector predictLoc = PVector.add(location, predict);

    // Now we must find the normal to the path from the predicted location
    // We look at the normal for each line segment and pick out the closest one
    PVector normal = null;
    PVector target = null;
    float worldRecord = 1000000;  // Start with a very high worldRecord distance that can easily be beaten

    // Loop through all points of the path
    for (int i = 0; i < v.vehicles.size(); i++) {

      // Look at a line segment
      PVector a = v.vehicles.get(i).location;
      PVector b = v.vehicles.get((i+1)%v.vehicles.size()).location; // Note Path has to wraparound

      // Get the normal point to that line
      PVector normalPoint = getNormalPoint(predictLoc, a, b);

      // Check if normal is on line segment
      PVector dir = PVector.sub(b, a);
      // If it's not within the line segment, consider the normal to just be the end of the line segment (point b)
      //if (da + db > line.mag()+1) {
      if (normalPoint.x < min(a.x, b.x) || normalPoint.x > max(a.x, b.x) || normalPoint.y < min(a.y, b.y) || normalPoint.y > max(a.y, b.y)) {
        normalPoint = b.get();
        // If we're at the end we really want the next line segment for looking ahead
        a = v.vehicles.get((i+1)%v.vehicles.size()).location;
        b = v.vehicles.get((i+2)%v.vehicles.size()).location;  // Path wraps around
        dir = PVector.sub(b, a);
      }

      // How far away are we from the path?
      float d = PVector.dist(predictLoc, normalPoint);
      // Did we beat the worldRecord and find the closest line segment?
      if (d < worldRecord) {
        worldRecord = d;
        normal = normalPoint;

        // Look at the direction of the line segment so we can seek a little bit ahead of the normal
        dir.normalize();
        // This is an oversimplification
        // Should be based on distance to path & velocity
        dir.mult(25);
        target = normal.get();
        target.add(dir);
      }
    }

    // Draw the debugging stuff
    if (debug) {
      // Draw predicted future location
      stroke(0);
      fill(0);
      line(location.x, location.y, predictLoc.x, predictLoc.y);
      ellipse(predictLoc.x, predictLoc.y, 4, 4);

      // Draw normal location
      stroke(0);
      fill(0);
      ellipse(normal.x, normal.y, 4, 4);
      // Draw actual target (red if steering towards it)
      line(predictLoc.x, predictLoc.y, target.x, target.y);
      if (worldRecord > v.radius) fill(255, 0, 0);
      noStroke();
      ellipse(target.x, target.y, 8, 8);
    }

    // Only if the distance is greater than the path's radius do we bother to steer
    if (worldRecord > v.radius*2) {
      return seek(target);
    } else {
      return new PVector(0, 0);
    }
  }


  // A function to get the normal point from a point (p) to a line segment (a-b)
  // This function could be optimized to make fewer new Vector objects
  PVector getNormalPoint(PVector p, PVector a, PVector b) {
    // Vector from a to p
    PVector ap = PVector.sub(p, a);
    // Vector from a to b
    PVector ab = PVector.sub(b, a);
    ab.normalize(); // Normalize the line
    // Project vector "diff" onto line by using the dot product
    ab.mult(ap.dot(ab));
    PVector normalPoint = PVector.add(a, ab);
    return normalPoint;
  }

  // Separation
  // Method checks for nearby boids and steers away
  PVector separate (ArrayList boids) {
    float desiredseparation = r*2;
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (int i = 0; i < boids.size(); i++) {
      Vehicle other = (Vehicle) boids.get(i);
      float d = PVector.dist(location, other.location);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(location, other.location);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }


  // Method to update location
  void update(float osc) {
    // Update velocity


    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    PVector velOsc = velocity.mult(osc);
    velOsc.div(maxspeed);
    location.add(velocity);
    acceleration.add(velOsc);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
    
    r = map(velocity.mag(), 0, 10, minRadius, maxRadius);

    //float note = map(velocity, 0, maxspeed, 54.00, 108.00);
    int listItem = (int) map(velocity.mag(), 0, 10, 0, freqArray.length);
   
    float note = freqArray[listItem];

    currentFreq = Frequency.ofHertz( note ); 

    wave.setFrequency( currentFreq );
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, location);  // A vector pointing from the location to the target

    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    // Steering = Desired minus Velocationity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force

    return steer;
  }


  void render() {
    // Simpler boid is just a circle
    fill(c, 0, 0 );
    //stroke(0,100);
    noStroke();
    pushMatrix();
    translate(location.x, location.y);
    //    ellipse(0, 0, 100/r, 100/r);
    ellipse(0, 0, r, r);
    // scale(r);
    // image(sprite, 0, 0, r, r);
    popMatrix();
  }

  // Wraparound
  void borders() {
    if (location.x < -r) location.x = width+r;
    //if (location.y < -r) location.y = height+r;
    if (location.x > width+r) location.x = -r;
    //if (location.y > height+r) location.y = -r;
  }
}
class VehicleArray {
  ArrayList<Vehicle> vehicles;
  int arrayNum = 1;
  float radius;
  int colorInt;
  int vehicleNum = 2;
  float osc ;

  VehicleArray(float ms, float mf, int col, int vn, float rad) {
    vehicles = new ArrayList<Vehicle>();
    for (int i = 0; i < vn; i++) {
      newVehicle(random(width), random(height),  ms, mf, col, rad);
    }
  }

  void run(Path p, float o) {
    for (Vehicle v : vehicles) {
      // Path following and separation are worked on in this function
      v.applyBehaviors(vehicles, p);
      // Call the generic run method (update, borders, display, etc.)
      v.run(o);
    }
  }

  void run(VehicleArray v, float o) {
    for (Vehicle vehic : vehicles) {
      // Path following and separation are worked on in this function
      vehic.applyBehaviors(vehicles, v);
      // Call the generic run method (update, borders, display, etc.)
      vehic.run(o);
    }
  }


  void newVehicle(float x, float y, float ms, float mf,  int col, float rad) {
    float maxspeed = ms;
    float maxforce = mf;
    //radius to force 
    radius = rad;
    colorInt = col;
    vehicles.add(new Vehicle(new PVector(x, y), maxspeed, maxforce, radius, colorInt));
  }

  void mouseVehicle(float x, float y) {
    float maxspeed =  random(2, 4);
    float maxforce = 0.3;
    radius = 12;
    colorInt = 255;
    vehicles.add(new Vehicle(new PVector(x, y), maxspeed, maxforce, radius, colorInt));
  }

  void displayPath() {
    strokeJoin(ROUND);

    // Draw thick line for radius

    stroke(colorInt, 0, 0, 50);
    strokeWeight(radius*2);
    noFill();
    beginShape();
    for (Vehicle v : vehicles) {
      vertex(v.location.x, v.location.y);
    }
    endShape(CLOSE);
    // Draw thin line for center of path
    stroke(colorInt, 0, 0, 120);
    strokeWeight(1);
    noFill();
    beginShape();
    for (Vehicle v : vehicles) {
      vertex(v.location.x, v.location.y);
    }
    endShape(CLOSE);
  }
}
class VehicleSystem {
  int numGen = 30;
  VehicleArray[]va = new VehicleArray[numGen];
  float maxForce = 5;
  float maxSpeed;
  int c = 255;
  int vehicleNum = 10 ;
  float radius = 40;

  VehicleSystem() {
    maxSpeed = .2;
    for (int i = 0; i < va.length; i++) {

      va[i] = new VehicleArray(maxSpeed, maxForce, c, vehicleNum, radius);
      maxSpeed*=1.1;
      maxForce/=.1;
      c-=255/numGen;
      vehicleNum*=1.125;
      radius/=1.1;
    }
  }

  //create a new gen
  void addGeneration() {
    //  va.add();
  }

  void run(float o) {
    for (int i = 0; i < va.length; i++) {
      if (i == 0) va[i].run(path, o);
      else va[i].run(va[i-1], o);
    }
    if (dPath) {
      // Draw predicted future location
      for (int i = 0; i < numGen-1; i++) {
        //   va[i].displayPath();
      }
    }
  }
}

