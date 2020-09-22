/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Code for graphical interface
 * 
 * J. Evan Smith, Ben Y. Brown, modified from work by Rogerio Bego
 * Last revised: 8 September 2020
 *
 * =========== OUTLINE ===========
 *
 * ☑ import libraries
 * ☑ variable initalization
 * ☑ object instantiation
 * ☑ setup() 
 * ☑ draw()
 * ☑ mouseClicked() 
 * ☑ mousePressed()
 * ☑ mouseReleased()
 * ☑ mouseMoved()
 * ☑ mouseDragged()
 * ☑ adjustFt()
 * ☑ handleIncoming() --> buffer
 * 
 * =========== CLASSES ===========
 *
 * ☑ Button
 * ☑ Channel
 * ☑ CheckBox
 * ☑ Dial
 * ☑ Display 
 * ☑ FmtNum
 * ☑ Group
 * ☑ Panel
 *
 * =========== LEGEND ===========
 *
 * ☐ = in development
 * ☑ = clean & 'complete'
 *
 * https://github.com/brownby/uScope/tree/usb-dev/processing/interface
 */

// *** import libraries *** //

import ddf.minim.*;           // used to connect to device over USB audio
import processing.serial.*;   // used to connect to device over virtual COM port

// *** variable initialization *** //

String version="beta";

boolean nInt = true;             // n is an integer (round) or decimal !nInt 
boolean fmt = true;              // fmt = true = "format", !fmt = false = "no format"
boolean stream = true;           // for startStop
boolean dtError = false;         // check for sampling time error
boolean waitforTrigger = false;   

byte numCh = 2;
byte scaleLinear = 0;   
byte scaleLog = 1;     
byte changeMove = 2;      // value changed by "MouseDragged"
byte changeRelease = 3;   // value changed by "MouseReleased"

int samples = 4000;
int sample_rate = 107100; // unused for now, but to avoid "magic" numbers in setting up sampling / "show samples"
int vTrigger = 308;         // value of trigger 0-1024 (0-5V), if 10 bit ADC 
int marg1, marg2;         // to adjust the position of objects

float DIV = 45.0;         // division unit size

color rgb[]={color(255, 255, 0), color(0, 204, 255)};  // for 2 channels: yellow (CH0) and blue (CH1)

// *** object instantiation *** //

Minim minim;
AudioInput in; // USB connection to device

Serial myDevice;

Channel channel[] = new Channel[numCh];
Group group[]     = new Group[numCh+1]; // used to change V/div and ms/div simultaneously on all channels using SHIFT key

Display display;

Button    startStop;
Button    resetAxes;
Button    resetCursors;  // measure vs size? *flag

CheckBox  showSamples; 
CheckBox  calcFreq;    // detect frequency
CheckBox  slowRoll;

// ---- sampling controls ---- //

Panel     pnlSamples;          // panel for sampling controls
Button    oneSample;           // request a sample
Button    severalSamples;      // request several samples
Button    streamContinuous;    // enters reading every dt
Dial      dt;                  // delta t (time of each reading)
Dial      q;                   // number of readings
FmtNum    tTotal;              // total sampling time dt*q
FmtNum    tTotalReal, dtReal;  // check if the real sample time is the same as desired

// ---- waveform --- //

Panel     pnlWave;       // panel for the waveform generator
CheckBox  wave;          // f and t are dependent: f = 1/t, t = 1/f
Dial      fWave;         // frequency of waveform 
Dial      aWave;         // amplitude of waveform 
Dial      oWave;         // DC offset of waveform
CheckBox  sineWave;      // type
CheckBox  pulseWave;     // type
CheckBox  squareWave;    // type
CheckBox  sawtoothWave;  // type

// *** setup function *** //

void setup() {
  
  size(1040, 635); 
  frameRate(15);

  display = new Display(30+10, 60, 17*DIV, 12*DIV);  // 17 horizontal and 12 vertical divisions
  
  marg1 = display.x+display.w+10; 
  marg2 = marg1+200;
  
  minim = new Minim(this);
  in = minim.getLineIn(Minim.MONO, samples, 176400, 16);
  in.disableMonitoring();
  in.mute();
  
  printArray(Serial.list());
  myDevice = new Serial(this, "COM11", 115200);

  for (byte k=0; k<numCh+1; k++){ group[k] = new Group(); }  // must be completed before channels
  for (byte k=0; k<numCh; k++){ channel[k] = new Channel(k, rgb[k], marg1+15, display.y+12+k*125, 185, 110); }
  
  for (byte k=0; k<numCh; k++){ 
  
    channel[k].vertScale.saveV();
    channel[k].horiScale.saveV();
    
  }
  
  startStop        = new Button("start / stop",marg1+15,15,185,40,color(255,0,0),color(0));
  resetAxes        = new Button("axes",marg1+70,channel[1].y+channel[1].h+15,45,20);
  resetCursors     = new Button("cursors",resetAxes.x+resetAxes.w+2,channel[1].y+channel[1].h+15,60,20);
  
  slowRoll         = new CheckBox("slow roll", marg1+25, channel[1].y+channel[1].h+50, 15);
  showSamples      = new CheckBox("show samples", slowRoll.x, slowRoll.y+slowRoll.h+5, 15);
  calcFreq         = new CheckBox("detect frequency", slowRoll.x, showSamples.y+showSamples.h+5, 15);
  
  pnlWave          = new Panel("Waveform Generator", color(168,52,235), marg1+15, display.y+display.h-170, 185, 170);   //display.x+785
  wave             = new CheckBox("output status", showSamples.x, pnlWave.y+25, 15);
  fWave            = new Dial(scaleLinear, changeMove, !nInt, fmt, "", "Hz", 1e3f, 200, 20e3f, pnlWave.x+10, pnlWave.y+53, pnlWave.w-20, 20);
  aWave            = new Dial(scaleLinear, changeMove, !nInt, fmt, "", "V", 3.3f, 100e-3f, 3.3f, pnlWave.x+10, fWave.y+fWave.h+3, pnlWave.w-20, 20);
  oWave            = new Dial(scaleLinear, changeMove, !nInt, fmt, "", "V", 1.65f, 300e-3f, 3f, pnlWave.x+10, aWave.y+aWave.h+3, pnlWave.w-20, 20); 
  sineWave         = new CheckBox("sine", pnlWave.x+10, oWave.y+oWave.h+10, 15);  
  pulseWave        = new CheckBox("pulse", pnlWave.x+90, oWave.y+oWave.h+10, 15);  
  squareWave       = new CheckBox("square", sineWave.x, sineWave.y+20, 15);    
  sawtoothWave     = new CheckBox("sawtooth", pulseWave.x, pulseWave.y+20, 15);    

  wave.clicked = true;
  sineWave.clicked = true;

// ---- sampling controls ---- //

  pnlSamples       = new Panel("sampling", color(0,100, 255), display.x+785, display.y+display.h-85, 200, 85);
  dt               = new Dial(scaleLog, changeRelease, nInt, fmt, "dt", "s", 5.667e-6f, 10e-6f, 2f, pnlSamples.x+5, pnlSamples.y+20, 100, 20);
  dtReal           = new FmtNum(0,nInt,fmt);
  q                = new Dial(scaleLinear, changeRelease, nInt, !fmt, "q", "", samples, 100, 3000, dt.x+dt.w+5, dt.y, 60, 20);
  tTotal           = new FmtNum(dt.v.getV()*q.v.getV(), !nInt);
  tTotalReal       = new FmtNum(0,!nInt);
 
}

void draw() {

  background(110); 
  fill(244, 244, 244); 
  
  display.display();
  
  textSize(24); fill(255); textAlign(LEFT, CENTER);
  text("μScope "+version, display.x, 30);
  
  textSize(15); textAlign(LEFT, CENTER);
  text("open source instrumentation for Arduino", display.x+465, 30);

  textSize(15); textAlign(RIGHT, CENTER);  
  text("RESET",resetAxes.x-10,resetAxes.y+resetAxes.h/2);

  startStop.display();
  slowRoll.display();
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
  
  handleIncoming();
  
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
 
  if (startStop.mouseClicked()) {
    
    stream = !stream; 
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
  
  showSamples.mouseClicked();
  calcFreq.mouseClicked();

  fWave.mouseClicked(); 
  aWave.mouseClicked();
  oWave.mouseClicked();
  
}

void mousePressed() {
  
  for (int k=0; k<numCh; k++) { channel[k].mousePressed(); }
  
  fWave.mousePressed();
  aWave.mousePressed();
  oWave.mousePressed();
  
  resetAxes.mousePressed();
  resetCursors.mousePressed();
  
}

void mouseReleased() {

  for (int k=0; k<numCh; k++) { channel[k].mouseReleased(); }
  
  resetAxes.mouseReleased();
  resetCursors.mouseReleased();

  fWave.mouseReleased();
  aWave.mouseReleased();
  oWave.mouseReleased();
  
}

void mouseMoved() {
  
  for (int k=0; k<numCh; k++) { channel[k].mouseMoveu(); } 
  
  fWave.mouseMoveu();
  aWave.mouseMoveu();
  oWave.mouseMoveu();

}

void mouseDragged() {

  for (int k=0; k<numCh; k++) { channel[k].mouseDragged(); }

  fWave.mouseDragged();
  aWave.mouseDragged();
  oWave.mouseDragged();

  if(fWave.mouseDragged()){ myDevice.write("f"+nf(round(fWave.v.v))); }
  if(aWave.mouseDragged()){ myDevice.write("a"+nf(round(aWave.v.v*1E3))); }
  if(oWave.mouseDragged()){ myDevice.write("o"+nf(round(oWave.v.v*1E3))); }

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
