/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Code for graphical interface
 * 
 * J. Evan Smith, Ben Y. Brown, modified from work by Rogerio Bego
 * Last revised: 11 October 2020
 *
 * https://github.com/brownby/uScope/
 */

import ddf.minim.*;          // used to connect to device over USB audio
import processing.serial.*;  // used to connect to device over virtual COM port
import controlP5.*;
import java.util.*;

boolean nInt = true;         // n is an integer (round) or decimal !nInt 
boolean fmt = true;          // fmt = true = "format", !fmt = false = "no format"
boolean stream = false;      // for startStop
boolean connected = false;   // for CDC
boolean selected = false;    // for port selection
boolean OoR = false;

byte numCh = 2;
byte scaleLinear = 0;   
byte scaleLog = 1;     
byte changeMove = 2;       // value changed by "MouseDragged"
byte changeRelease = 3;    // value changed by "MouseReleased"

int samples = 4000;        // host-side buffer size
int sample_rate = 176400;  // unused for now, but to avoid "magic" numbers in setting up sampling / "show samples"
int vTrigger = 0;        // *flag -> why initialized here?
int marg1, marg2;          // to adjust the position of objects
int fntSize = 10;

float DIV = 45.0;          // division unit size

String version="beta";
String portName;

color rgb[]={color(255, 255, 0), color(0, 204, 255)};  // for 2 channels: yellow (CH0) and blue (CH1)
PFont font;

Minim minim;       // parent to AudioInput
AudioInput in;     // isochronous USB connection to device
Serial myDevice;   // CDC USB connection to device

ControlP5 cp5;     // used for scrollable list, COM port selection

Channel channel[] = new Channel[numCh];
Group group[]     = new Group[numCh+1]; // used to change V/div and ms/div simultaneously on all channels using SHIFT key

Display   display;

Button    connect;
Button    startStop;
Button    resetAxes;
Button    resetCursors;

CheckBox  showSamples; 
CheckBox  calcFreq;      // detect frequency via interpeak spacing
CheckBox  nTrigger;

Panel     pnlSamples;    // panel for sampling controls -> not displayed (!)
Dial      dt;            // delta t (time of each reading), fixed in setup()
Dial      q;             // number of readings, fixed in setup()

Panel     pnlWave;       // panel for the waveform generator
CheckBox  wave;          // toggle
Dial      fWave;         // frequency of waveform 
Dial      aWave;         // amplitude of waveform 
Dial      oWave;         // DC offset of waveform
CheckBox  sineWave;      // type
CheckBox  pulseWave;     // type
CheckBox  squareWave;    // type
CheckBox  sawtoothWave;  // type

void setup() {
  
  size(1040, 635);  // *flag -> how does this display on 1080p monitor?
  frameRate(15);    // sub-cinematic to reduce CPU utilization
  
  display = new Display(30+10, 70, 17*DIV, 12*DIV);  // 17 horizontal and 12 vertical divisions
  
  marg1 = display.x+display.w+10; 
  marg2 = marg1+200;
  
  minim = new Minim(this);
  in = minim.getLineIn(Minim.MONO, samples, 176400, 16);
  
  in.disableMonitoring();  // *flag
  in.mute();

  for (byte k=0; k<numCh+1; k++){ group[k] = new Group(); }  // must be completed before channels
  for (byte k=0; k<numCh; k++){ channel[k] = new Channel(k, rgb[k], marg1+15, display.y+2+k*125, 185, 110); }
  
  for (byte k=0; k<numCh; k++){ 
  
    channel[k].vertScale.saveV();  // for reset axes
    channel[k].horiScale.saveV(); 
  
  }
  
  connect          = new Button("connect",marg1-102,15,93,40,rgb[0],rgb[0],color(0));
  startStop        = new Button("start / stop",marg1+15,15,185,40,color(0,255,0),color(255,0,0),color(0));
  resetAxes        = new Button("axes",marg1+70,channel[1].y+channel[1].h+17,45,20);
  resetCursors     = new Button("cursors",resetAxes.x+resetAxes.w,channel[1].y+channel[1].h+17,60,20);
  
  nTrigger         = new CheckBox("normal trigger", marg1+25, channel[1].y+channel[1].h+55, 15);
  showSamples      = new CheckBox("show samples", nTrigger.x, nTrigger.y+nTrigger.h+5, 15);
  calcFreq         = new CheckBox("detect frequency", nTrigger.x, showSamples.y+showSamples.h+5, 15);
  
  pnlWave          = new Panel("Waveform Generator", color(168,52,235), marg1+15, display.y+display.h-170, 185, 170);   //display.x+785
  wave             = new CheckBox("output status", showSamples.x, pnlWave.y+25, 15);
  fWave            = new Dial(scaleLinear, changeMove, !nInt, fmt, "", "Hz", 1e3f, 300, 20e3f, pnlWave.x+10, pnlWave.y+53, pnlWave.w-20, 20);
  aWave            = new Dial(scaleLinear, changeMove, !nInt, fmt, "", "V", 3.3f, 100e-3f, 3.3f, pnlWave.x+10, fWave.y+fWave.h+3, pnlWave.w-20, 20);
  oWave            = new Dial(scaleLinear, changeMove, !nInt, fmt, "", "V", 1.65f, 50e-3f, 3.25f, pnlWave.x+10, aWave.y+aWave.h+3, pnlWave.w-20, 20); 
  sineWave         = new CheckBox("sine", pnlWave.x+10, oWave.y+oWave.h+10, 15);  
  pulseWave        = new CheckBox("pulse", pnlWave.x+90, oWave.y+oWave.h+10, 15);  
  squareWave       = new CheckBox("square", sineWave.x, sineWave.y+20, 15);    
  sawtoothWave     = new CheckBox("sawtooth", pulseWave.x, pulseWave.y+20, 15);    
  
  pnlSamples       = new Panel("sampling", color(0,100, 255), display.x+785, display.y+display.h-85, 200, 85);
  dt               = new Dial(scaleLog, changeRelease, nInt, fmt, "dt", "s", 5.667e-6f, 10e-6f, 2f, pnlSamples.x+5, pnlSamples.y+20, 100, 20);
  q                = new Dial(scaleLinear, changeRelease, nInt, !fmt, "q", "", samples, 100, 3000, dt.x+dt.w+5, dt.y, 60, 20);
  
  wave.clicked = true;
  sineWave.clicked = true;
  startStop.clicked = true;
  
  cp5 = new ControlP5(this);
  font = createFont("Verdana", fntSize);

  String[] ports = Serial.list();
  List p = Arrays.asList(ports);
 
  cp5.addScrollableList("SerialPorts")
     .setPosition(connect.x-248, 17)
     .setSize(247, 100)
     .setCaptionLabel("Select Serial Port")
     .setBarHeight(37)
     .setItemHeight(18)
     .setFont(font)
     .setColorBackground(color(200))
     .setColorActive(color(255))
     .setColorForeground(rgb[0])
     .setColorValueLabel(color(0))
     .setColorCaptionLabel(color(0))
     .addItems(p);  
     
}


void draw() {

  background(110); 
  fill(244, 244, 244); 
  
  stroke(color(0)); strokeWeight(1.8); 
  rect(connect.x-250,15,249,40); 
  
  textSize(24); fill(255); textAlign(LEFT, CENTER);
  text("μScope "+version, display.x, 30);
  
  if (OoR){
    
    textSize(13); fill(rgb[0]); textAlign(LEFT, CENTER);
    text("the output waveform is out of range!", display.x + 165, 35);  
    
  }
  
  textSize(15); fill(255); textAlign(RIGHT, CENTER);  
  text("RESET",resetAxes.x-10,resetAxes.y+resetAxes.h/2);
  
  display.display();
  
  connect.display();
  startStop.display();
  nTrigger.display();
  resetAxes.display();
  resetCursors.display();
  showSamples.display();
  calcFreq.display();
  
  pnlWave.display();
  wave.display();
  fWave.display();
  aWave.display();
  oWave.display();
  sineWave.display();
  pulseWave.display();
  squareWave.display();
  sawtoothWave.display();
  
  for (byte k=0; k<numCh; k++) { channel[k].display(); }
  
  if (connected){
    handleIncoming(); 
  }  
}

void SerialPorts(int n) {
  
  portName = cp5.get(ScrollableList.class, "SerialPorts").getItem(n).get("name").toString();
  selected = true;

}

void mouseClicked() {
  
  for (int k=0; k<numCh; k++) {
    if (channel[k].mouseClicked()){ 
      if (channel[k].chN.clicked){
        println("switch channel "+str(k)+" on");
      } 
      else {
        println("switch channel "+str(k)+" off");
      }
    }
  }
 
 
  if (startStop.mouseClicked()) {
    
      stream = !stream; 
      nTrigger.clicked = false; 
      //channel[0].enable_latch = false;
      //channel[1].enable_latch = false;
      if (stream == false){ println("stop"); }
      else { println("start"); }
      
  }
  
  if (resetAxes.mouseClicked()) {
    
    for (int k=0; k<numCh;k++) {
      
     channel[k].p0 = display.y+5*DIV*(k+1); // reset zero voltage position for all channels
     channel[k].chN.clicked = true;         // turn on all channels

     channel[k].horiScale.restore();
     channel[k].vertScale.restore();        // return to inital scale
     
     channel[k].displayClicked = false;     // remove cursors

    }
    println("reset axes");
    resetAxes.clicked = false;
  }
  
  if (resetCursors.mouseClicked()) {
    
     for (int k=0; k<numCh;k++) {
       
        channel[k].displayClicked = false; 
     
     }
     println("reset cursors");
     resetCursors.clicked = false;
  }
  
  if (connect.mouseClicked()){
    if(selected){
      if(connected){ myDevice.stop(); }
      myDevice = new Serial(this, portName, 115200);
      connected = true;
      startStop.clicked = false;
      stream = true;
    }
    connect.clicked = false;
  }

  showSamples.mouseClicked();
  nTrigger.mouseClicked();
//  calcFreq.mouseClicked(); // *flag -> on hold until second beta

  if (connected){
    
    if (wave.mouseClicked()){
    
      if (wave.clicked == true){ 
        myDevice.write("x");
        println("turning output on"); 
      }
      else{ 
        myDevice.write("i");
        println("turning output off"); 
      }
    }
  
    if (sineWave.mouseClicked()){
    
      sineWave.clicked = true;
      pulseWave.clicked = false;
      squareWave.clicked = false;
      sawtoothWave.clicked = false;
      
      myDevice.write("0");
      
      println("sine wave selected");
         
    }
     
    if (pulseWave.mouseClicked()){
    
      sineWave.clicked = false;
      pulseWave.clicked = true;
      squareWave.clicked = false;
      sawtoothWave.clicked = false;
      
      myDevice.write("1");
         
      println("pulse wave selected");   
         
    }
    
    if (squareWave.mouseClicked()){
    
      sineWave.clicked = false;
      pulseWave.clicked = false;
      squareWave.clicked = true;
      sawtoothWave.clicked = false;
      
      myDevice.write("2");
      
      println("square wave selected");
         
    }
    
    if (sawtoothWave.mouseClicked()){
    
      sineWave.clicked = false;
      pulseWave.clicked = false;
      squareWave.clicked = false;
      sawtoothWave.clicked = true;
      
      myDevice.write("3");
      
      println("sawtooth wave selected");
         
    }

    fWave.mouseClicked(); 
    aWave.mouseClicked();
    oWave.mouseClicked();
  
  }
}

void mousePressed() {
  
  for (int k=0; k<numCh; k++) { channel[k].mousePressed(); }
  
  connect.mousePressed();
  resetAxes.mousePressed();
  resetCursors.mousePressed();
  
  if (connected){
  
    fWave.mousePressed();
    aWave.mousePressed();
    oWave.mousePressed();
  
  }
}

void mouseReleased() {

  for (int k=0; k<numCh; k++) { channel[k].mouseReleased(); }
  
  connect.mouseReleased();
  resetAxes.mouseReleased();
  resetCursors.mouseReleased();

  if (connected){

  fWave.mouseReleased();
  aWave.mouseReleased();
  oWave.mouseReleased();
  
  }
}

void mouseMoved() {
  
  for (int k=0; k<numCh; k++) { channel[k].mouseMoveu(); } 
  
  if (connected){
  
  fWave.mouseMoveu();
  aWave.mouseMoveu();
  oWave.mouseMoveu();
  
  }
}

void mouseDragged() {

  for (int k=0; k<numCh; k++) { channel[k].mouseDragged(); }

  if (connected){
    
    fWave.mouseDragged();
    aWave.mouseDragged();
    oWave.mouseDragged();

  }
  
  if(fWave.mouseDragged()){ 
    
      if (sawtoothWave.clicked == true && fWave.v.v > 6000){
        
        OoR = true;
        
      }
      else{
       
        myDevice.write("f"+nf(round(fWave.v.v))); 
        OoR = false;
        
      }
    
  }
  
  if(aWave.mouseDragged()){ 
  
    if (((oWave.v.v + aWave.v.v/2)>3.3) || ((oWave.v.v - aWave.v.v/2)<0.0)){
    
      OoR = true;
    
    }
    else{ 
     
      myDevice.write("a"+nf(round(aWave.v.v*1E3))); 
      OoR = false;
      
    }
  }
   
  if(oWave.mouseDragged()){ 
    
    if (((oWave.v.v + aWave.v.v/2)>3.3) || ((oWave.v.v - aWave.v.v/2)<0.0)){
      
      OoR = true;
    
    }
    else{ 
    
      myDevice.write("o"+nf(round(oWave.v.v*1E3))); 
      OoR = false;
    
    }
  }
}

void adjustFt() {
  
  float ftNew=dt.v.getV()*q.v.getV()/10.0;
  for (int k=0; k<numCh; k++) { channel[k].horiScale.setV(ftNew); }
  
}
  
void handleIncoming() {
  
  if (stream == true){
  
    for(int i = 0; i < in.bufferSize(); i++) { channel[0].buffer[i]= int(in.left.get(i)*5400); } // empirical 'calibration' to match Waveforms amplitude, offset
    for(int i = 0; i < in.bufferSize(); i++) { channel[1].buffer[i]= int(in.left.get(i)*5400); } 
    
    channel[0].updated=true;
    channel[1].updated=true;
    
  }
}
