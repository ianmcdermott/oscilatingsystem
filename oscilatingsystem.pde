
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