/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Code for graphical interface
 * 
 * J. Evan Smith, Ben Y. Brown, modified from work by Rogerio Bego
 * Last revised: 24 August 2020
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

import ddf.minim.*;  // used to connect to device over USB audio


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

int sample_rate = 150000; // unused for now, but to avoid "magic" numbers in setting up sampling / "show samples"
int vTrigger = 0;         // value of trigger 0-1024 (0-5V), if 10 bit ADC 
int marg1, marg2;         // to adjust the position of objects

float DIV = 45.0;         // division unit size

color rgb[]={color(255, 255, 0), color(0, 204, 255)};  // for 2 channels: yellow (CH0) and blue (CH1)


// *** object instantiation *** //

Minim minim;
AudioInput in; // USB connection to device
AudioOutput out;

Channel channel[] = new Channel[numCh];
Group group[]     = new Group[numCh+1]; // used to change V/div and ms/div simultaneously on all channels using SHIFT key

Display display;

Button    startStop;
Button    resetAxes;
Button    resetCursors;  // measure vs size? *flag

CheckBox  showSamples; 
CheckBox  calcFreq;    // detect frequency

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
CheckBox  sineWave;      // type
CheckBox  pulseWave;     // type
CheckBox  squareWave;    // type
CheckBox  sawtoothWave;  // type


// *** setup function *** //

void setup() {
  
  size(1040, 635); 
  frameRate(30);

  display = new Display(30+10, 60, 17*DIV, 12*DIV);  // 17 horizontal and 12 vertical divisions
  
  marg1 = display.x+display.w+10; 
  marg2 = marg1+200;
  
  minim = new Minim(this);
  in = minim.getLineIn(Minim.MONO, 1000, 44100, 8);
  out = minim.getLineOut(Minim.MONO, 1000, 44100, 8);
  in.disableMonitoring();

  for (byte k=0; k<numCh+1; k++){ group[k] = new Group(); }  // must be completed before channels
  for (byte k=0; k<numCh; k++){ channel[k] = new Channel(k, rgb[k], marg1+15, display.y+25+k*130, 185, 110); }
  
  for (byte k=0; k<numCh; k++){ 
  
    channel[k].vertScale.saveV();
    channel[k].horiScale.saveV();
    
  }
  
  startStop        = new Button("start / stop",marg1+15,15,185,40,color(255,0,0),color(0));
  resetAxes        = new Button("axes",marg1+70,channel[1].y+channel[1].h+30,45,20);
  resetCursors     = new Button("cursors",resetAxes.x+resetAxes.w+2,channel[1].y+channel[1].h+30,60,20);
  
  showSamples      = new CheckBox("show samples", marg1+25, channel[1].y+channel[1].h+70, 15);
  calcFreq         = new CheckBox("detect frequency", showSamples.x, showSamples.y+showSamples.h+5, 15);
  
  pnlWave          = new Panel("Waveform Generator", color(168,52,235), marg1+15, display.y+display.h-150, 185, 150);   //display.x+785
  wave             = new CheckBox("output status", showSamples.x, pnlWave.y+25, 15);
  fWave            = new Dial(scaleLinear, changeMove, !nInt, fmt, "", "Hz", 1e3f, 1e-3f, 10e3f, pnlWave.x+10, pnlWave.y+53, pnlWave.w-20, 20);
  aWave            = new Dial(scaleLinear, changeMove, !nInt, fmt, "", "V", 1f, 10e-3f, 3f, pnlWave.x+10, fWave.y+fWave.h+3, pnlWave.w-20, 20);
  sineWave         = new CheckBox("sine", pnlWave.x+10, aWave.y+aWave.h+10, 15);  
  pulseWave        = new CheckBox("pulse", pnlWave.x+90, aWave.y+aWave.h+10, 15);  
  squareWave       = new CheckBox("square", sineWave.x, sineWave.y+20, 15);    
  sawtoothWave     = new CheckBox("sawtooth", pulseWave.x, pulseWave.y+20, 15);    

  wave.clicked = true;
  sineWave.clicked = true;

  
// ---- sampling controls ---- //

  pnlSamples       = new Panel("sampling", color(0,100, 255), display.x+785, display.y+display.h-85, 200, 85);
  dt               = new Dial(scaleLog, changeRelease, nInt, fmt, "dt", "s", 6.67e-6f, 10e-6f, 2f, pnlSamples.x+5, pnlSamples.y+20, 100, 20);
  dtReal           = new FmtNum(0,nInt,fmt);
  q                = new Dial(scaleLinear, changeRelease, nInt, !fmt, "q", "", 1000-1, 1, 100, dt.x+dt.w+5, dt.y, 60, 20);
  tTotal           = new FmtNum(dt.v.getV()*q.v.getV(), !nInt);
  tTotalReal       = new FmtNum(0,!nInt);
 
}


// *** draw function *** //

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
  resetAxes.display();
  resetCursors.display();
  showSamples.display();
  calcFreq.display();
  
  pnlWave.display();
  wave.display();
  fWave.display();
  aWave.display();
  sineWave.display();
  pulseWave.display();
  squareWave.display();
  sawtoothWave.display();
  
  for (byte k=0; k<numCh; k++) { channel[k].display(); }
  
  handleIncoming();
  
}


// *** mouseClicked function *** //

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
      in.unmute();
      out.unmute();
      //in.setVolume(1.0);
      println(in.isMuted());
      println(in.getVolume());
      println("turning output on"); 
    }
    else{ 
      in.mute();
      out.mute();
      //in.setVolume(0.0);
      println(in.isMuted());
      println(in.getVolume());
      println("turning output off"); 
    }
  }
  
  if (sineWave.mouseClicked()){
  
    sineWave.clicked = true;
    pulseWave.clicked = false;
    squareWave.clicked = false;
    sawtoothWave.clicked = false;
    
    println("sine wave selected");
       
  }
   
  if (pulseWave.mouseClicked()){
  
    sineWave.clicked = false;
    pulseWave.clicked = true;
    squareWave.clicked = false;
    sawtoothWave.clicked = false;
       
    println("pulse wave selected");   
       
  }
  
  if (squareWave.mouseClicked()){
  
    sineWave.clicked = false;
    pulseWave.clicked = false;
    squareWave.clicked = true;
    sawtoothWave.clicked = false;
    
    println("square wave selected");
       
  }
  
    if (sawtoothWave.mouseClicked()){
  
    sineWave.clicked = false;
    pulseWave.clicked = false;
    squareWave.clicked = false;
    sawtoothWave.clicked = true;
    
    println("sawtooth wave selected");
       
  }
 
  if (startStop.mouseClicked()) {
    
    stream = !stream; 
    if (stream == false){ println("stop"); }
    else { println("start"); }
    
  }
  
  if (resetAxes.mouseClicked()) {
    
    for (int k=0; k<numCh;k++) {
      
     channel[k].p0 = display.y+4*DIV*(k+1); // reset zero voltage position for all channels
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

  if (dt.mouseClicked()) { adjustFt(); } // if dt changed, then adjustFt()
  if (q.mouseClicked())  { adjustFt(); } // if q changed, then adjustFt()
  
}


// *** mousePressed function *** //

void mousePressed() {
  
  for (int k=0; k<numCh; k++) { channel[k].mousePressed(); }
  
  q.mousePressed();
  dt.mousePressed();
  
  resetAxes.mousePressed();
  resetCursors.mousePressed();
  
}


// *** mouseReleased function *** //

void mouseReleased() {

  for (int k=0; k<numCh; k++) { channel[k].mouseReleased(); }
  
  resetAxes.mouseReleased();
  resetCursors.mouseReleased();

  if (dt.mouseReleased()) { adjustFt(); }  // if dt changed, then adjustFt()
  if (q.mouseReleased())  { adjustFt(); }  // if q changed, then adjustFt()
  
}


// *** mouseReleased function *** //

void mouseMoved() {
  
  for (int k=0; k<numCh; k++) { channel[k].mouseMoveu(); } 
  
  dt.mouseMoveu();
  q.mouseMoveu();

}


// *** mouseDragged function *** //

void mouseDragged() {

  for (int k=0; k<numCh; k++) { channel[k].mouseDragged(); }

  dt.mouseDragged();
  q.mouseDragged();

}


// *** adjustFt function *** //

void adjustFt() {
  
  float ftNew=dt.v.getV()*q.v.getV()/10.0;
  for (int k=0; k<numCh; k++) { channel[k].horiScale.setV(ftNew); }
  
}


// *** handleIncoming function *** //
  
void handleIncoming() {
  
  if (stream == true){
  
    for(int i = 0; i < in.bufferSize()-1; i++) { channel[0].buffer[i]= int(in.left.get(i)*300)+40; } // empirical 'calibration' to match Waveforms amplitude, offset
    for(int i = 0; i < in.bufferSize()-1; i++) { channel[1].buffer[i]= int(in.left.get(i)*300)+40; } 
    
    channel[0].updated=true;
    channel[1].updated=true;
    
  }
}
