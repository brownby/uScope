class Channel {
  
  // *** variable initialization *** //
  
  byte n;
  
  boolean updated = false;
  boolean holdP0=false;         // indicates mouse got p0, zero voltage position
  boolean holdTrigger=false;    // indicates mouse got trigger
  boolean enable_latch = false;
  boolean displayClicked=false;  
  
  color nRGB;
  
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
  float fa=5.0/(1023.0);  // ADC conversion from / to voltage
  float horiCalibrate = 1.7;
  
  FmtNum fCalc = new FmtNum(0,!nInt,fmt);
  FmtNum tCalc = new FmtNum(0,!nInt,fmt); // frequency and period calcuated from the peaks
 

 // *** object instantiation *** //
  
  Button chN;
 
  Dial vertScale;  // factor of scale for voltage (vertical)
  Dial horiScale;  // factor of scale for time (horizontal)
 
  CheckBox trigger;  // trigger
  CheckBox smooth;   // smooth data with curveVertex(), vertex()
  CheckBox cursors;  // cursors time and voltage

  Channel(byte n_, color nRGB_, int x_, int y_, int w_, int h_) { // constructor
  
     n = n_; nRGB = nRGB_; x = x_; y = y_; w = w_; h = h_;

     chN     = new Button("CH-"+str(n),x,y,w/2,15,nRGB,nRGB);
     trigger = new CheckBox("trigger",x+w/2+10,y+3,14);
     
     chN.clicked = true;
     
     vertScale = new Dial(scaleLog,changeMove,!nInt,fmt,"","v/div",1f,100e-3f,10f,x+10,y+23,w-20,20,1);//21
     horiScale = new Dial(scaleLog,changeMove,!nInt,fmt,"","s/div",1e-3f,50e-6f,2e-3f,x+10,vertScale.y+vertScale.h+3,w-20,20,2);
     
     p0 = display.y+5*DIV*(n+1);
     p0Trigger = p0;
     
     cursors = new CheckBox("cursors",horiScale.x,horiScale.y+horiScale.h+5,15);
     smooth  = new CheckBox("smooth",horiScale.x+horiScale.w/2,horiScale.y+horiScale.h+5,15);
     
  }
  
  void display() {
     
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
  
  
  void p0MousePressed() {
    
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
  
  
  void p0MouseDragged() {
    
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


  void displayXt(){ 
    
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
 
   
  float fx(int x){ return display.x + horiCalibrate*DIV*dt.v.v/horiScale.v.v*x; } // dt.v.v is sample time = 9.33E-6, horiScale.v.v = time/div
  float fy(int y){ return p0 - y*fa/vertScale.v.v*DIV; }  
 
 
  void edgeDetector(){
    
    byte hysteresis = 5;
    int margin = (int)map(horiScale.v.v,horiScale.vMin,horiScale.vMax,4.0,2.0);
    
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
  
  void smoothDifferential() {

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

   
    vMin=(int)(2.0/3.0*(float)vMin);  // search for peaks between between vMin and 2/3*vMin-vNoise
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
    
  void displayRect() {     // show the selection rectangle and time, voltage levels
    if (displayClicked) {
        
      fill(nRGB,50); stroke(nRGB,255); strokeWeight(1);
      dashed(xi,yi,xi+dx,yi+dy,3);
         
      fill(255);
      
      //horiCalibrate*DIV*dt.v.v/horiScale.v.v*x;
         
      float vTemp=(abs(dx)/(DIV*horiCalibrate))*horiScale.v.v*1000.0;
      String vh=nf(vTemp,0,2)+" ms";
      String fh=nf(1000/vTemp,0,1)+ " Hz";
      String vv=nf(abs(dy)/(DIV)*vertScale.v.v,0,2)+" V";
      
      textAlign(RIGHT); text(vh+" "+fh,xi+dx-10,yi+dy/2);
      textAlign(LEFT); text(vv,xi+dx,yi+dy/2);
         
    }       
  }
     
  void dashed(float xi, float yi, float xf, float yf, float step) {
       
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

  void displayMousePressed() {

    if (cursors.clicked) { // set to search for which channel color is closest to mouse
      if (mouseX>display.x && mouseX<display.x+display.w && mouseY>display.y && mouseY<display.y+display.h){
        
        displayClicked = true;
        xi = mouseX;
        yi = mouseY;    
        dx=0; dy=0;   
        
      }
    }
  }
     
  void displayMouseDragged() {
    
    if (cursors.clicked){
      if (displayClicked){  
        if (mouseX>display.x && mouseX<display.x+display.w && mouseY>display.y && mouseY<display.y+display.h){
          
            dx = mouseX - xi;
            dy = mouseY - yi;
        
           }
         }
       }
     }
  
  void displayMouseRelease() {

    if (cursors.clicked){
      if (displayClicked) {
        if (abs(dx)<10 && abs(dy)<10){
          
               displayClicked=false;

        }
      } 
    }
  }

   
  boolean mouseClicked() {
    
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
  
  void mousePressed() {
    
    vertScale.mousePressed();
    horiScale.mousePressed();
    
    p0MousePressed(); 
    displayMousePressed();
    
  }
  
  void mouseDragged() {
    
    vertScale.mouseDragged();
    horiScale.mouseDragged();
    
    p0MouseDragged();
    displayMouseDragged();
    
  }
  
  void mouseReleased() {
    
    vertScale.mouseReleased();
    horiScale.mouseReleased();
    
    if (holdP0){ holdP0 = false; }
    if (holdTrigger){
      
      vTrigger=constrain(int((p0-p0Trigger)/(fa/vertScale.v.v*DIV)),0,1024);
      println("tv"+str(vTrigger)+".");
      holdTrigger=false;
      
    }
    displayMouseRelease();
  }
  
  void mouseMoveu() { vertScale.mouseMoveu(); horiScale.mouseMoveu(); }
}
