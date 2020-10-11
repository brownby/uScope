import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import ddf.minim.*; 
import processing.serial.*; 
import controlP5.*; 
import java.util.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class uInterface extends PApplet {

/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Code for graphical interface
 * 
 * J. Evan Smith, Ben Y. Brown, modified from work by Rogerio Bego
 * Last revised: 23 September 2020
 *
 * https://github.com/brownby/uScope/
 */

          // used to connect to device over USB audio
  // used to connect to device over virtual COM port



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

float DIV = 45.0f;          // division unit size

String version="beta";
String portName;

int rgb[]={color(255, 255, 0), color(0, 204, 255)};  // for 2 channels: yellow (CH0) and blue (CH1)
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

public void setup() {
  
    // *flag -> how does this display on 1080p monitor?
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


public void draw() {

  background(110); 
  fill(244, 244, 244); 
  
  stroke(color(0)); strokeWeight(1.8f); 
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

public void SerialPorts(int n) {
  
  portName = cp5.get(ScrollableList.class, "SerialPorts").getItem(n).get("name").toString();
  selected = true;

}

public void mouseClicked() {
  
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

public void mousePressed() {
  
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

public void mouseReleased() {

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

public void mouseMoved() {
  
  for (int k=0; k<numCh; k++) { channel[k].mouseMoveu(); } 
  
  if (connected){
  
  fWave.mouseMoveu();
  aWave.mouseMoveu();
  oWave.mouseMoveu();
  
  }
}

public void mouseDragged() {

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
  
    if (((oWave.v.v + aWave.v.v/2)>3.3f) || ((oWave.v.v - aWave.v.v/2)<0.0f)){
    
      OoR = true;
    
    }
    else{ 
     
      myDevice.write("a"+nf(round(aWave.v.v*1e3f))); 
      OoR = false;
      
    }
  }
   
  if(oWave.mouseDragged()){ 
    
    if (((oWave.v.v + aWave.v.v/2)>3.3f) || ((oWave.v.v - aWave.v.v/2)<0.0f)){
      
      OoR = true;
    
    }
    else{ 
    
      myDevice.write("o"+nf(round(oWave.v.v*1e3f))); 
      OoR = false;
    
    }
  }
}

public void adjustFt() {
  
  float ftNew=dt.v.getV()*q.v.getV()/10.0f;
  for (int k=0; k<numCh; k++) { channel[k].horiScale.setV(ftNew); }
  
}
  
public void handleIncoming() {
  
  if (stream == true){
  
    for(int i = 0; i < in.bufferSize(); i++) { channel[0].buffer[i]= PApplet.parseInt(in.left.get(i)*5400); } // empirical 'calibration' to match Waveforms amplitude, offset
    for(int i = 0; i < in.bufferSize(); i++) { channel[1].buffer[i]= PApplet.parseInt(in.left.get(i)*5400); } 
    
    channel[0].updated=true;
    channel[1].updated=true;
    
  }
}
class Button{
  
   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   String txt;

   boolean mPressed=false;
   boolean clicked=false;
   
   int rgb_active  = color(0,255,0);
   int rgb_outline = color(0);
   int rgb_pressed = color(0,255,0);
   
   Button(String txt_,int x_,int y_, int w_, int h_) {  //constructor
   
      txt=txt_; x=x_; y=y_; w=w_; h=h_;
   
   } 
   
   Button(String txt_,int x_,int y_, int w_, int h_, int rgb_pressed_, int rgb_active_, int rgb_outline_) {
     
      txt=txt_; x=x_; y=y_; w=w_; h=h_;
      rgb_active = rgb_active_;
      rgb_outline = rgb_outline_;
      rgb_pressed = rgb_pressed_;
      
   }
   
   public void display() {
     
     if (mPressed){ fill(rgb_pressed); } 
     else if (clicked){ fill(rgb_active); } 
     else{ fill(200); }
     
     stroke(rgb_outline); strokeWeight(2); 
     rect(x,y,w,h); 
     
     if (rgb_active==color(0,0,255) && clicked){ fill(255); } else{ fill(0); } // use white for text on blue background color
     textAlign(CENTER,CENTER); textSize(14); 
     text(txt,x+w/2,y+h/2);

   }
   
   public void mousePressed() {
     
     if (mouseButton==LEFT) {
       if (mouseX>x && mouseX<x+w && mouseY>y && mouseY<y+h){
          mPressed=true;
       }
     } 
   }
   
   public void mouseReleased() {
     
      if (mPressed) mPressed=false;
   
   }
   
   public boolean mouseClicked() { // returns if clicked or not
     
     boolean status=false;
     
     if (mouseButton==LEFT){
       if (mouseX>x && mouseX<x+w && mouseY>y && mouseY<y+h){
          clicked=!clicked;
          status=true;
          mPressed=false;
       }
     }
     
     return status;
     
   }
}
class Channel {
  
  // *** variable initialization *** //
  
  byte n;
  
  boolean updated = false;
  boolean holdP0=false;         // indicates mouse got p0, zero voltage position
  boolean holdTrigger=false;    // indicates mouse got trigger
  boolean enable_latch = false;
  boolean displayClicked=false;  
  
  int nRGB;
  
  int x;  // position
  int y;  // position 
  int w;  // width
  int h;  // height
  
  int qPeaks;
  int qMax = 4000; 
  int latch_index = 0;
 
  int v[]      = new int[qMax];
  int dif[]    = new int[qMax];  // v[t+1]-v[t]
  int buffer[] = new int[qMax];
  int peaks[]  = new int[qMax];

  float p0;               // zero voltage position
  float p0Trigger;        // trigger position
  float dP0Trigger;       // difference between p0 and p0Trigger
  float mouseOffSet;      // for moving objects, like p0
  float xi, yi, dx, dy;   // rectangle for measuring time and voltage on the display
  float fa=5.0f/(1023.0f);  // ADC conversion from / to voltage
  float horiCalibrate = 1.7f;
  
  FmtNum fCalc = new FmtNum(0,!nInt,fmt);
  FmtNum tCalc = new FmtNum(0,!nInt,fmt); // frequency and period calcuated from the peaks
 

 // *** object instantiation *** //
  
  Button chN;
 
  Dial vertScale;  // factor of scale for voltage (vertical)
  Dial horiScale;  // factor of scale for time (horizontal)
 
  CheckBox trigger;  // trigger
  CheckBox smooth;   // smooth data with curveVertex(), vertex()
  CheckBox cursors;  // cursors time and voltage

  Channel(byte n_, int nRGB_, int x_, int y_, int w_, int h_) { // constructor
  
     n = n_; nRGB = nRGB_; x = x_; y = y_; w = w_; h = h_;

     chN     = new Button("CH-"+str(n),x,y,w/2,15,color(0,255,0),nRGB,nRGB);
     trigger = new CheckBox("trigger",x+w/2+10,y+3,14);
     
     chN.clicked = true;
     
     vertScale = new Dial(scaleLog,changeMove,!nInt,fmt,"","v/div",1f,100e-3f,10f,x+10,y+23,w-20,20,1);//21
     horiScale = new Dial(scaleLog,changeMove,!nInt,fmt,"","s/div",1e-3f,50e-6f,2e-3f,x+10,vertScale.y+vertScale.h+3,w-20,20,2);
     
     p0 = display.y+5*DIV*(n+1);
     p0Trigger = p0;
     
     cursors = new CheckBox("cursors",horiScale.x,horiScale.y+horiScale.h+5,15);
     smooth  = new CheckBox("smooth",horiScale.x+horiScale.w/2,horiScale.y+horiScale.h+5,15);
     
  }
  
  public void display() {
     
     if (updated){
       arrayCopy(buffer,v);  // buffer --> v
       updated=false;
     }
     
     vertScale.groupUpdate();
     horiScale.groupUpdate();
     
     strokeWeight(2); stroke(nRGB); noFill();
     rect(x,y,w,h);
     chN.display();
     
     if (chN.clicked){

       trigger.display();
       vertScale.display();
       horiScale.display();
       cursors.display();
       smooth.display();
       
       strokeWeight(1); stroke(nRGB,150);
       line(display.x-10*n,p0,display.x+display.w,p0); // line for zero voltage position
       fill(nRGB); noStroke();
       triangle(display.x-10*n,p0,display.x-10-10*n,p0-10,display.x-10-10*n,p0+10); // associated triangle
      

       if (trigger.clicked){
         
         if (!holdTrigger) { p0Trigger = fy(vTrigger); }

         strokeWeight(2); stroke(nRGB,100);
         line(display.x-10*n,p0Trigger,display.x+display.w,p0Trigger);
         fill(nRGB); noStroke();
         triangle(display.x+display.w,p0Trigger,display.x+display.w+10,p0Trigger-10,display.x+display.w+10,p0Trigger+10);
        
      }
      
      displayXt();      
      displayRect();
    } 
  }
  
  
  public void p0MousePressed() {
    
    int pInitial;
    int pFinal;
    
    if (chN.clicked){
      
      pInitial = display.x-10-10*n;
      pFinal = display.x-10*n;
      
      if (mouseX>pInitial && mouseX<pFinal && mouseY>(p0-10) && mouseY<(p0+10)) {
        
        mouseOffSet=mouseY-p0;
        dP0Trigger=p0Trigger-p0;
        holdP0=true;
        
      } 
      else {
        
        pInitial = display.x+display.w;
        pFinal   = display.x+display.w+10;
        
        if (mouseX>pInitial && mouseX<pFinal && mouseY>(p0Trigger-10) && mouseY<(p0Trigger+10)) {
           
           mouseOffSet=mouseY-p0Trigger;
           holdTrigger=true;
           
        }
      }
    }
  }
  
  
  public void p0MouseDragged() {
    
    if (holdP0) {
     
      p0 = constrain(mouseY,display.y,display.y+display.h)-mouseOffSet; 
      p0Trigger = p0 + dP0Trigger;
      
      if (keyPressed && key==CODED && keyCode==SHIFT) {
        
        int k2 = 2;
        if (p0<=display.y+display.h/2) {
          for (int k=0;k<4;k++) {
            if (k != n) {
                 
              channel[k].p0=constrain(display.y+(p0-display.y)*k2,display.y,display.y+display.h);
              k2++;
               
            }
          }
        } 
        else {
          for (int k=0; k<4; k++){
            if (k!=n){
                
                channel[k].p0=constrain(display.y+display.h-(display.y+display.h-p0)*k2,display.y,display.y+display.h);
                k2++;
                
            }
          }
        }
      }
    } 
    else if (holdTrigger) {
     
      p0Trigger=constrain(mouseY,fy(1024),p0)-mouseOffSet; 
      println("holdTrigger=true  p0Trigger="+str(p0Trigger));
     
    }
  }


  public void displayXt(){ 
    
    float px, py;
    stroke(nRGB); strokeWeight(2); noFill();
    
    if (trigger.clicked) { edgeDetector(); }
    else { enable_latch = false; }
    
    strokeWeight(2); stroke(nRGB);
    
    if (enable_latch == true){
      
      channel[n].trigger.blink = true;
      
      beginShape();
      for (int k=latch_index; k<q.v.v; k++) {

        px = 115 + fx(k)-fx(latch_index);
        py = fy(v[k]);
        
        if (px>display.x+display.w || px<display.x){ break; }
        if (py<display.y){ continue; }
       
        if (smooth.clicked){ curveVertex(px,py); }
        else { vertex(px,py); }
        
        if (showSamples.clicked){ stroke(255); strokeWeight(4); point(px,py); strokeWeight(2); stroke(nRGB); }
      }
      endShape();
      if (nTrigger.clicked){
        startStop.clicked = true;
        stream = false;
      }
    }
    else if (enable_latch == false){
      
      channel[n].trigger.blink = false;
      
      beginShape();
      for (int k=0; k<q.v.v; k++) {

        px = fx(k);
        py = fy(v[k]);
        
        if (px>display.x+display.w || px<display.x){ break; }
        if (py<display.y){ continue; }
        
        if (smooth.clicked){ curveVertex(px,py); }
        else { vertex(px,py); }
        
        if (showSamples.clicked){ stroke(255); strokeWeight(4); point(px,py); strokeWeight(2); stroke(nRGB); }
      } 
      endShape();
    }
    
    if (calcFreq.clicked) { smoothDifferential(); }
    
    latch_index = 0;
  
  }
 
   
  public float fx(int x){ return display.x + horiCalibrate*DIV*dt.v.v/horiScale.v.v*x; } // dt.v.v is sample time = 9.33E-6, horiScale.v.v = time/div
  public float fy(int y){ return p0 - y*fa/vertScale.v.v*DIV; }  
 
 
  public void edgeDetector(){
    
    byte hysteresis = 3;
    int margin = 1;// (int)map(horiScale.v.v,horiScale.vMin,horiScale.vMax,4.0,2.0);
    
    int threshold_high = vTrigger + hysteresis;
    int threshold_low = vTrigger - hysteresis;
    
    for (int k=margin+1; k<q.v.v-margin-1; k++){
      if (v[k-margin]<threshold_low && v[k+margin]>threshold_high){
        
        if (fx(k)>display.x+display.w || fx(k)<display.x){ break; }
        
        //strokeWeight(5); stroke(0,255,0); // to display all edges detected
        //point(fx(k),p0Trigger);  
        
        if (latch_index == 0){ 
          
          if (fx(k)>display.x+display.w || fx(k)<display.x){ break; }
          
          latch_index = k;
          
          //strokeWeight(5); stroke(0,255,0); // display latch edge (moving)
          //point(fx(latch_index),p0Trigger);
          
          //strokeWeight(2); stroke(0,255,0);  
          //triangle(fx(latch_index),display.h+65,fx(latch_index)-10,display.h+75,fx(latch_index)+10,display.h+75);
        
          strokeWeight(5); stroke(0,255,0); // display fixed latch edge
          point(display.x+75,p0Trigger);
          
          strokeWeight(2); stroke(0,255,0);  
          triangle(display.x+75,display.h+65,display.x+65,display.h+75,display.x+85,display.h+75);
          
          strokeWeight(1); stroke(0,255,0); 
          line(display.x+75,display.y,display.x+75,display.y+display.h);
        
        }
      }
      if (latch_index != 0){ enable_latch = true; }
      else { enable_latch = false; }
    }
  }
  
  public void smoothDifferential() {

    int vMax  =  0, pMax  = -1;
    int vMin  =  0, pMin  = -1;  
    int vMin1 =  0, vMin2 =  0;
    int pMin1 = -1, pMin2 = -1; // -1 indicates not found 

    qPeaks=0;
    
    for (int k=1; k<q.v.v; k++){ // find the maximum dif value vMax >= pMax (point k)
      dif[k-1]=v[k]-v[k-1];
      if (dif[k-1]>vMax) {
        
        vMax=dif[k-1];
        pMax=k-1;
        
      }
      if (dif[k-1]<vMin){
        
        vMin=dif[k-1];
        pMin=k-1;
        
      }
    }

   
    vMin=(int)(2.0f/3.0f*(float)vMin);  // search for peaks between between vMin and 2/3*vMin-vNoise
    qPeaks=0;
    
    if (vMin<0) {
      for (int k=0; k<q.v.v-1; k++){ 
        if (dif[k]<=vMin){
           
          qPeaks++;
          peaks[qPeaks-1]=k;
           
        }
      }
      if (qPeaks>=2){
        if (peaks[1]-peaks[0]>1){
           
          pMin1=peaks[0]; vMin1=dif[pMin1];
          pMin2=peaks[1]; vMin2=dif[pMin2];
         
        }
      }
    }
    
    if (pMin1>=0 && pMin2>=0){ }        // must be square wave
    else{                               // must be sinusoidal (smooth)
      for (int k=0; k<q.v.v-2; k++) {   // get two change points from zero
        if (dif[k]>0 && dif[k+1]<=0) {  // found + to - peak?
          if (pMin1<0) {
            
            vMin1=dif[k+1];
            pMin1=k+1;

          } 
          else if (pMin2<0) {
            
            vMin2=dif[k+1];
            pMin2=k+1;
            break;
            
          }
        }
      }
    }
 
    tCalc.setV(0);
    fCalc.setV(0);

    if (pMin1>=0 && pMin2>=0) {

        strokeWeight(5); stroke(255,0,255);  
        point(fx(pMin1),p0);
        point(fx(pMin2),p0);
        
        strokeWeight(1);
       
        tCalc.setV(abs(pMin2 - pMin1)*dt.v.v);
        fCalc.setV(1/tCalc.v);
      
        textAlign(LEFT); fill(0);
        text(fCalc.printV() + "Hz (" + tCalc.printV() + "s)", cursors.x, cursors.y + 29); 
     }  
  }

  
  // *** Control of measuring rectangle *** //
    
  public void displayRect() {     // show the selection rectangle and time, voltage levels
    if (displayClicked) {
        
      fill(nRGB,50); stroke(nRGB,255); strokeWeight(1);
      dashed(xi,yi,xi+dx,yi+dy,3);
         
      fill(255);

      float vTemp=(abs(dx)/(DIV*horiCalibrate))*horiScale.v.v*1000.0f;
      String vh=nf(vTemp,0,2)+" ms";
      String fh=nf(1000/vTemp,0,1)+ " Hz";
      String vv=nf(abs(dy)/(DIV)*vertScale.v.v,0,2)+" V";
      
      if (xi+dx > xi){ textAlign(LEFT); text(vh+" "+fh+" "+ vv,xi,yi-15); }
      else { textAlign(RIGHT); text(vh+" "+fh+" "+ vv,xi,yi-15); }
         
    }       
  }
     
  public void dashed(float xi, float yi, float xf, float yf, float step) {
       
    float temp;
    boolean faz=true;
    
    if (xi>xf){ temp=xf; xf=xi; xi=temp; } 
    if (yi>yf){ temp=yf; yf=yi; yi=temp; }
    
    for (float x=xi; x<xf; x+=step) {
      if (faz) {
         
        line(x,yi,x+step,yi);
        line(x,yf,x+step,yf);
          
      } 
      faz=!faz;
    }
    for (float y=yi; y<yf; y+=step) {
      if (faz) {
         
        line(xi,y,xi,y+step);
        line(xf,y,xf,y+step);
          
      } 
      faz=!faz;
    }
  }

  public void displayMousePressed() {

    if (cursors.clicked) { // set to search for which channel color is closest to mouse
      if (mouseX>display.x && mouseX<display.x+display.w && mouseY>display.y && mouseY<display.y+display.h){
        
        displayClicked = true;
        xi = mouseX;
        yi = mouseY;    
        dx=0; dy=0;   
        
      }
    }
  }
     
  public void displayMouseDragged() {
    
    if (cursors.clicked){
      if (displayClicked){  
        if (mouseX>display.x && mouseX<display.x+display.w && mouseY>display.y && mouseY<display.y+display.h){
          
            dx = mouseX - xi;
            dy = mouseY - yi;
        
           }
         }
       }
     }
  
  public void displayMouseRelease() {

    if (cursors.clicked){
      if (displayClicked) {
        if (abs(dx)<10 && abs(dy)<10){
          
               displayClicked=false;

        }
      } 
    }
  }

   
  public boolean mouseClicked() {
    
    boolean ret=false;
    ret=chN.mouseClicked(); 

    if (trigger.mouseClicked()) {
      if (trigger.clicked){
        for (int k=0;k<numCh;k++){
          
          channel[k].trigger.clicked = false;
          vTrigger = 0;
          
        }
        trigger.clicked=true;
        println("switching trigger for channel "+str(n)+" on");
      }
      else{ 
          vTrigger = 0;
          enable_latch = false; 
      }  
    }
    
    vertScale.mouseClicked();
    horiScale.mouseClicked();
    
    if (cursors.mouseClicked()) {
      if (cursors.clicked) {
        for (int k=0;k<numCh;k++) { channel[k].cursors.clicked=false; }
        cursors.clicked=true;
      }
    }
    smooth.mouseClicked();
    return ret;
  }
  
  public void mousePressed() {
    
    vertScale.mousePressed();
    horiScale.mousePressed();
    
    p0MousePressed(); 
    displayMousePressed();
    
  }
  
  public void mouseDragged() {
    
    vertScale.mouseDragged();
    horiScale.mouseDragged();
    
    p0MouseDragged();
    displayMouseDragged();
    
  }
  
  public void mouseReleased() {
    
    vertScale.mouseReleased();
    horiScale.mouseReleased();
    
    if (holdP0){ holdP0 = false; }
    if (holdTrigger){
      
      vTrigger=constrain(PApplet.parseInt((p0-p0Trigger)/(fa/vertScale.v.v*DIV)),0,1024);
      println("tv"+str(vTrigger)+".");
      holdTrigger=false;
      
    }
    displayMouseRelease();
  }
  
  public void mouseMoveu() { vertScale.mouseMoveu(); horiScale.mouseMoveu(); }
}
class CheckBox {
   
   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   int tSize;  // textSize
   
   boolean clicked = false;
   boolean blink = false;
   
   String txt, txt2="";
   
   CheckBox(String txt_, int x_, int y_, int tSize_) {  //constructor
   
     txt=txt_; x=x_; y=y_; tSize=tSize_;
     textSize(tSize);
     h = tSize;
     w = (int)textWidth(txt)+h+5; 
     
   }
   
   public void display() {
     
      if (blink) { fill(map(millis()%750,0,1000,0,125)); }
      else { fill(0); }
      
      textAlign(LEFT,CENTER); 
      textSize(14); 
      text(txt, x+h+5, y+h/2-2);
      
      if (clicked) { fill(0,200,0); }
      else { noFill(); }
      
      stroke(0); strokeWeight(1); rect(x+2,y+2,h-2,h-4); 
      fill(0);
        
      if (clicked && txt2.length()>0){
         text(txt2,x+5,y+1.75f*h);
      }
   }
   
  public boolean mouseClicked(){
     boolean r=false;
     if (mouseX>x && mouseX<x+w & mouseY>y && mouseY<y+h){
        clicked=!clicked;
        r=true;
     }
     return r;
  } 
}
class Dial {
   
   boolean clicked = false;
   boolean showTriangles = false;
   boolean showIncrements = false;
   boolean nInt = false;
   
   byte scale = scaleLinear;
   byte change = changeRelease;  // change v when MouseDrag or MouseRelease
   
   int x;            // position
   int y;            // position 
   int w;            // width
   int h;            // height
   int g;            // used to change V/div and ms/div simultaneously on all channels using SHIFT key
   int cx;
   int mouseOffSet;
   
   float vOld;
   float vMin, vMax;
   
   String unity = "";
   String txt;

   FmtNum v, vTemp;
  
   
   Dial(byte scale_, byte change_, boolean nInt_, boolean fmt_, String txt_,String unity_, float v_, float vMin_, float vMax_, int x_, int y_, int w_, int h_) {  //constructor
   
       scale=scale_; change=change_; txt=txt_;
       unity=unity_;
       vMin=vMin_; vMax=vMax_;
       x=x_; y=y_; w=w_; h=h_;
       
       v=new FmtNum(v_,nInt_,fmt_);
       updateCx(); 
       vTemp=new FmtNum(v.v,nInt_,fmt_);
       
       g = 0;
       
   } 
   
   Dial(byte scale_, byte change_, boolean nInt_, boolean fmt_, String txt_,String unity_, float v_, float vMin_, float vMax_, int x_, int y_, int w_, int h_, int g_) {
     
       scale=scale_; change=change_; txt=txt_;
       unity=unity_;
       vMin=vMin_; vMax=vMax_;
       x=x_; y=y_; w=w_; h=h_;
       
       v=new FmtNum(v_,nInt_,fmt_);
       updateCx(); 
       vTemp=new FmtNum(v.v,nInt_,fmt_);
      
       g = g_;
       group[g].qtd++;

   } 

   public void saveV(){ vOld = v.v; }
   
   public void restore(){ setV(vOld); }
   
   public void setV(float v_){
     
       v.setV(v_); 
       updateCx();
       
   }
   
   public void updateCx(){ cx = v2x(v.v); }
   
   public void display(){ display(color(0)); }
   
   public void display(int rgb) {
     
     stroke(rgb);
     strokeWeight(1); fill(200); rect(x,y,w,h);                // outline rectangle
     noStroke();  fill(244,244,244); rect(x+1,y+1,cx-x-2,h-2); // now fill slider to the value
      
     if (showIncrements){
       
       fill(0); stroke(0); textSize(10);
       text("-100",x,y+5); 
       text("-10",x+w/6,y+5);
       text("-1",x+2*w/6,y+5);
       text("+1",x+3*w/6,y+5);
       text("+10",x+4*w/6,y+5);
       text("+100",x+5*w/6,y+5);
       
     }
     
     if (showTriangles){
       
       fill(250,250,0); stroke(0);
       triangle(cx,y+3*h/4,cx-5,y+h,cx+5,y+h); 
       triangle(cx,y+h/4,cx-5,y,cx+5,y); 
        
     }

     stroke(0);
      
     for (int k = 0; k<5; k++){ 
        
       float vx = x + (k + 1)*0.17f*w;
       if(k == 2){ line(vx,y,vx,y+0.2f*h); } // add grid lines between 6 units, so 5 lines
       else { line(vx,y,vx,y+0.1f*h); } 
       
     }

     fill(0); strokeWeight(2); textSize(12); textAlign(CENTER,CENTER); 
     String t=txt+" ";
      
     if (clicked) {
        
       if (change==changeRelease){ t+=vTemp.printV(); } 
       else { t+=v.printV(); }
       
     } 
     else { t+=v.printV(); }
      
     text(t+unity,x+w/2,y+h/2-2);
      
   }
   

   public int v2x(float v_) {
     
     if (scale==scaleLinear) { return (int)map(v_,vMin,vMax,x,x+w); } 
     else { return (int)map(log(v_)/log(10),log(vMin)/log(10), log(vMax)/log(10),x,x+w); }
     
   }
   
   public float x2v(int cx_) {
     
     if (scale==scaleLinear) { return map(cx_,x,x+w,vMin,vMax); } 
     else{ return pow(10,map(cx_,x,x+w,log(vMin)/log(10),log(vMax)/log(10))); }  
   
   }
   
   
   public void groupUpdate() {
     
     if (g>0) {
       if (group[g].count>0) {
          
         group[g].count--;
         setV(group[g].v);
         if (group[g].count<=0) { group[g].v=0; }
           
       }
     }
   }
   
   
   public boolean mouseClicked(){
     
     boolean changed = false;
     float v2 = 0;
     
     if (mouseX>x && mouseX<x+w && mouseY>y && mouseY<y+h){
       
       changed=true;
       int p=(int)map(mouseX,x,x+w,1,7);

       switch (p) {
         case 1: 
           v2=v.addN(-100);
           break;
         case 2: 
           v2=v.addN(-10);
           break;
         case 3: 
           v2=v.addN(-1);
           break;
         case 4: 
           v2=v.addN(+1);
           break;
         case 5: 
           v2=v.addN(+10);
           break; 
         case 6:
           v2=v.addN(+100);
           break;
       }
       
       if (v2<vMin){ v.setV(vMin); } 
       else if (v2>vMax){ v.setV(vMax); }
       else { v.setV(v2); }
       
       updateCx();
       ifShiftChangeGroup();       
     } 
     return changed;
   }


   public void mouseMoveu() {
     
     if (mouseY>y && mouseY<y+h) {
        
       if (mouseX>cx-10 && mouseX<cx+10){ showTriangles = true; } 
       else { showTriangles = false; }
        
       if (mouseX>x && mouseX<x+w && keyPressed && keyCode==CONTROL){
          
         println("showIncrements=" + showIncrements);
         showIncrements=true; 
        
       } 
       else { showIncrements=false; } 
     }  
     else { showTriangles=false; }   
   }
   
   
   public void mousePressed() {
     
     if (mouseButton==LEFT){
      if (mouseY>y && mouseY<y + h) {
        if (mouseX>cx - 10 && mouseX<cx + 10){
          
          clicked=true; 
          vTemp.setV(v.v);
          mouseOffSet=mouseX-cx;

        }
      }
     }
   }
   
   
   public boolean mouseDragged() { 

      boolean send = false; // legacy --> cmd to uC, TODO: delete
      if (clicked){
        
         cx=constrain(mouseX-mouseOffSet,x,x+w);
         if (change==changeMove) { // change immediately while moving mouse
           
           vTemp.setV(x2v(cx)); 
           v.setV(vTemp.v); 
           send = true;   
           ifShiftChangeGroup(); 
         
         }
         else{ vTemp.setV(x2v(cx)); }
      }
     return send; 
   }
   
   
   public boolean mouseReleased() { 
     
     boolean send = false; // legacy --> cmd to uC, TODO: delete
     if (clicked) { 
       clicked=false;
       if (change == changeRelease) {
         if (mouseY>y-10 && mouseY<y+h+10) { 
            
           v.setV(vTemp.v); // change when releasing the mouse
           send=true; 
           ifShiftChangeGroup(); 
               
         } 
         else{ cx=v2x(v.v); }
       }
     } 
     return send;
   }
   
   
   public void ifShiftChangeGroup() {
     
     if (keyPressed && key==CODED && keyCode==SHIFT) {
       
        group[g].v=v.v;
        group[g].count=group[g].qtd; // number of controls that synchronize
     
     }
   }
}
class Display { 

   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   Display(float xi, float yi, float wi, float hi) { // constructor
      
      x=PApplet.parseInt(xi); y=PApplet.parseInt(yi); w=PApplet.parseInt(wi); h=PApplet.parseInt(hi);
   
   } 
   
   public void display() {
     
      fill(40); stroke(0,0,255); strokeWeight(1);
      rect(x,y,w,h); 
      stroke(100);
      
      for (float row=y; row<y+h; row+=h/12) { 
        line(x,row,x+w,row);
      }
      
      for (float col=x; col<x+w; col+=w/10) {
        line(col,y,col,y+h);
      } 
   }
}
class FmtNum{
  
   float v;  // value as float
   float n;  // numeric part of formatted expression
   char  u;  // unit part of formatted expression
   
   int i; // index for units
   char unid[] = {'f','p','n','u','m',' ','k','M','G','T','P'};  // pico(-12), nano(-9), micro(-6), milli(-3), (0), kilo(3), mega(6), giga(9), tera(12)
   
   boolean nInt     = false;  // true --> round n to integer value
   boolean format   = true;   // formatted as nu = numeric + unit

   FmtNum(float v_,boolean nInt_,boolean fmt_) {  //constructor
   
     v = v_;
     nInt = nInt_;
     v2nu(v);
     format = fmt_;
   
   }
   
   FmtNum(float v_,boolean nInt_) {
     
     v = v_;
     nInt = nInt_;
     v2nu(v);
     format = true;
   
   } 
   
   public String printV() {
     
     if (nInt) {               // is integer?
       if (format) {           // is formatted?
         return nf(n,0,0)+u;  
       } 
       else {                  // needs to be formatted
         return str(PApplet.parseInt(v));
       }
     } 
     else{                     // is decimal
       if (format){            // is formatted?
         return str(n)+u;
       } else {                // needs to be formatted
         return str(v); 
       }
     }
     
   }
   
   public void setV(float v_) {
     
     v=v_;
     v2nu(v);  
   
   }
   
   public float getV() {
     
     if (nInt){ return PApplet.parseInt(n)*pow(10,(i-5)*3); } 
     else{ return v; }
     
   }

   public void setNInt(){
     
       n=round(n);
       nu2v();
   }
   
   public float addN(float k) { // add / substract n (if u = ' ' go to decimal place

      float n2=PApplet.parseInt(n);
      int i2=i;    
      
      if (n2+k>0) {
        n2+=k;
      } 
      else {
        if (i2>0) {
           i2--;
           n2=1000+k; 
        }
      }
      
      return n2*pow(10,(i2-5)*3);
      
   } 
   
   public void v2nu(float v_) { // value to formatted numeric + unit?
   
    i = constrain(PApplet.parseInt((log(v_)/log(10)+15)/3),0,unid.length-1); // calculate the index of the exponent of the number (v_) in base 10
    
    if (nInt) {
      n = round(v_/pow(10,(i-5)*3));
    } 
    else {
      n = round((v_/pow(10,(i-5)*3))*10.0f)/10.0f;
    }
    
    u = unid[i];
    
   }
   
   public void nu2v() { // formatted numeric + unit to value?
   
      v = n*pow(10,(i-5)*3);
      v2nu(v); 
      
   }
}
class Group {
  
  float v     = 0;
  int   qtd   = 0;
  int   count = 0;
  
}
class Panel {
  
   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   int RGB;
   
   String  txt   = "";
   String  txt2  = "";
   Boolean blink = false;
   
   Panel(String txt_, int RGB_, int x_, int y_, int w_, int h_) {  // constructor
   
     txt = txt_;
     RGB = RGB_;
     x = x_; 
     y = y_; 
     w = w_; 
     h = h_;
   
   }
   
   public void display(){
      
      strokeWeight(2); stroke(RGB); noFill();
      rect(x,y,w,h);
      
      fill(RGB);
      rect(x,y,w-w/5,15);
      
      if (blink){
        fill(map(millis()%1000,0,1000,0,255));
      }
      else {
        fill(0);
      } 
      
      textAlign(LEFT); text(txt+" "+txt2,x+5,y+textAscent());
      
   } 
}
  public void settings() {  size(1040, 635); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "uInterface" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
