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
 * ☑ handleIncoming() --> buffer vs. stream?
 * 
 * =========== CLASSES ===========
 *
 * ☑ Button
 * ☐ Channel
 * ☑ CheckBox
 * ☐ Dial
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
boolean dtError = false;         // check for sampling time error
boolean waitforTrigger = false;   

byte numCh = 2;
byte scaleLinear = 0;   
byte scaleLog = 1;     
byte changeMove = 2;     // value changed by "MouseDragged"
byte changeRelease = 3;  // value changed by "MouseReleased"

int vTrigger = 0;  // value of trigger 0-1024 (0-5V), if 10 bit ADC 
int marg1, marg2;  // to adjust the position of objects

float DIV = 45.0;     // division unit size

color rgb[]={color(255, 255, 0), color(0, 204, 255)};  // for 2 channels: yellow (CH0) and blue (CH1)


// *** object instantiation *** //

Minim minim;
AudioInput in; // USB connection to device

Channel channel[] = new Channel[numCh];
Group group[] = new Group[numCh+1]; // used to change V/div and ms/div simultaneously on all channels using SHIFT key

Display display;

Button    startStop;
Button    resetAxes;
Button    resetMedir;  // measure vs size? *flag

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

Panel     pnlWave;   // panel for the waveform generator
CheckBox  wave;      // f and t are dependent: f = 1/t, t = 1/f
Dial      fWave;     // frequency of waveform (0.125Hz-10kHz) 
Dial      tWave;     // period of waveform (100us-8s)
Dial      dutyWave;  // duty cycle (0-100%)


// *** setup function *** //

void setup() {
  
  size(1040, 635); 
  frameRate(30);

  display = new Display(30+10, 60, 17*DIV, 12*DIV);  // 17 horizontal and 12 vertical divisions
  
  marg1 = display.x+display.w+10; 
  marg2 = marg1+200;
  
  minim = new Minim(this);
  in = minim.getLineIn(Minim.MONO, 1000, 44100, 8);
  in.disableMonitoring();

  for (byte k=0; k<numCh+1; k++){ group[k] = new Group(); }  // must be completed before channels
  for (byte k=0; k<numCh; k++){ channel[k] = new Channel(k, rgb[k], marg1+20, display.y+90+k*130, 185, 100); }
  
  startStop        = new Button("start / stop",marg1+20,channel[0].y-70,185,40);
  resetAxes        = new Button("axes",marg1+70,channel[1].y+channel[1].h+30,45,20);
  resetMedir       = new Button("size",resetAxes.x+resetAxes.w+2,channel[1].y+channel[1].h+30,45,20);
  
  showSamples      = new CheckBox("show samples", marg1+20, channel[1].y+channel[1].h+70, 15);
  calcFreq         = new CheckBox("detect frequency", showSamples.x, showSamples.y+showSamples.h+5, 15);
  
// ---- sampling controls ---- //

  pnlSamples       = new Panel("sampling", display.x+785, display.y+display.h-85, 200, 85);
  dt               = new Dial(scaleLog, changeRelease, nInt, fmt, "dt", "s", 24e-6f, 10e-6f, 2f, pnlSamples.x+5, pnlSamples.y+20, 100, 20);
  dtReal           = new FmtNum(0,nInt,fmt);
  q                = new Dial(scaleLinear, changeRelease, nInt, !fmt, "q", "", 1000-1, 1, 100, dt.x+dt.w+5, dt.y, 60, 20);
  tTotal           = new FmtNum(dt.v.getV()*q.v.getV(), !nInt);
  tTotalReal       = new FmtNum(0,!nInt);
  oneSample        = new Button("one", dt.x, dt.y+dt.h+5, 50, 20);
  severalSamples   = new Button("many", oneSample.x+oneSample.w+5, oneSample.y, oneSample.w, oneSample.h);
  streamContinuous = new Button("contin", severalSamples.x+severalSamples.w+5, severalSamples.y, severalSamples.w, severalSamples.h);
 
}


// *** draw function *** //

void draw() {

  background(110); 
  fill(244, 244, 244); 
  
  display.display();
  
  textSize(24); textAlign(LEFT, TOP);
  text("μScope "+version, display.x, 20);

  textSize(15); textAlign(RIGHT, CENTER);  
  text("RESET",resetAxes.x-10,resetAxes.y+resetAxes.h/2);

  startStop.display();
  resetAxes.display();
  resetMedir.display();
  showSamples.display();
  calcFreq.display();
  
  for (byte k=0; k<numCh; k++) { channel[k].display(); }
  
  handleIncoming();

}


// *** mouseClicked function *** //

void mouseClicked() {
 

  if (resetAxes.mouseClicked()) {
    
    for (int k=0; k<numCh;k++) {
      
     channel[k].p0 = display.y+3*DIV*(k+1);  // reset zero voltage position for all channels
     
     // TODO: add reset for horizontal scaling
    
    }
    resetAxes.clicked = false;
  }
  
  if (resetMedir.mouseClicked()) {
    
     for (int k=0; k<numCh;k++) {
       
        channel[k].displayClicked = false; 
     
     }
     resetMedir.clicked = false;
  }
  
  showSamples.mouseClicked();
  calcFreq.mouseClicked();

  if (dt.mouseClicked()) { adjustFt(); } // if dt changed, then adjustFt()
  if (q.mouseClicked())  { adjustFt(); } // if q changed, then adjustFt()

  if (oneSample.mouseClicked()) { 
    
    severalSamples.clicked=false;
    streamContinuous.clicked=false;
    oneSample.clicked=false;
    
    // check if there is a trigger triggered so that waitfor the trigger
    // will be flashing to indicate that it is waiting for the trigger
    
    int k2 = -1;
    
    for (int k=0; k<numCh;k++) {
      
      if (channel[k].trigger.clicked) {
         k2=k;
         break; 
      }
    }
    
    println("k2=",k2);
    
    if (k2>=0 && k2<=3) {
      
       pnlSamples.blink = true;
       channel[k2].trigger.blink = true;
       waitforTrigger = true;
       
    }
    else {
      
       pnlSamples.blink = false;
       waitforTrigger = false;
       
    }
  }
  
  if (severalSamples.mouseClicked()) {
    
    oneSample.clicked = false;
    streamContinuous.clicked = false;
  
  }
  
  if (streamContinuous.mouseClicked()) {
  
    oneSample.clicked = false;
    severalSamples.clicked = false;
  
  }
}


// *** mousePressed function *** //

void mousePressed() {
  
  for (int k=0; k<numCh; k++) { channel[k].mousePressed(); }
  
  q.mousePressed();
  dt.mousePressed();
  
  resetAxes.mousePressed();
  resetMedir.mousePressed();
 
  oneSample.mousePressed();
  severalSamples.mousePressed();
  streamContinuous.mousePressed();
  
}


// *** mouseReleased function *** //

void mouseReleased() {

  for (int k=0; k<numCh; k++) { channel[k].mouseReleased(); }
  
  resetAxes.mouseReleased();
  resetMedir.mouseReleased();

  oneSample.mouseReleased();
  severalSamples.mouseReleased();
  streamContinuous.mouseReleased();

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
  
  for(int i = 0; i < in.bufferSize()-1; i++) { channel[0].buffer[i]= int(in.left.get(i)*300)+40; } // empirical 'calibration' to match Waveforms amplitude, offset
  for(int i = 0; i < in.bufferSize()-1; i++) { channel[1].buffer[i]= int(in.left.get(i)*300)+40; } 
  
  channel[0].updated=true;
  channel[1].updated=true;
  
  if (waitforTrigger) {
    
    waitforTrigger = false;
    for (int k=0; k<numCh; k++){ channel[k].trigger.blink=false; }
    
  }
    
  // move data and store at the end?
  
  //for (int k=1; k<q.v.v; k++) {
  //  channel[0].v[k-1]=channel[0].v[k];
  //} 
  //channel[0].v[int(q.v.v-1)] = int(in.left.get(1)*300)+40;
  
}
