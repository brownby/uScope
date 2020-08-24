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

   void saveV(){ vOld = v.v; }
   
   void restore(){ setV(vOld); }
   
   void setV(float v_){
     
       v.setV(v_); 
       updateCx();
       
   }
   
   void updateCx(){ cx = v2x(v.v); }
   
   void display(){ display(color(0)); }
   
   void display(color rgb) {
     
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
        
       float vx = x + (k + 1)*0.17*w;
       if(k == 2){ line(vx,y,vx,y+0.2*h); } // add grid lines between 6 units, so 5 lines
       else { line(vx,y,vx,y+0.1*h); } 
       
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
   

   int v2x(float v_) {
     
     if (scale==scaleLinear) { return (int)map(v_,vMin,vMax,x,x+w); } 
     else { return (int)map(log(v_)/log(10),log(vMin)/log(10), log(vMax)/log(10),x,x+w); }
     
   }
   
   float x2v(int cx_) {
     
     if (scale==scaleLinear) { return map(cx_,x,x+w,vMin,vMax); } 
     else{ return pow(10,map(cx_,x,x+w,log(vMin)/log(10),log(vMax)/log(10))); }  
   
   }
   
   
   void groupUpdate() {
     
     if (g>0) {
       if (group[g].count>0) {
          
         group[g].count--;
         setV(group[g].v);
         if (group[g].count<=0) { group[g].v=0; }
           
       }
     }
   }
   
   
   boolean mouseClicked(){
     
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


   void mouseMoveu() {
     
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
   
   
   void mousePressed() {
     
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
   
   
   boolean mouseDragged() { 

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
   
   
   boolean mouseReleased() { 
     
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
   
   
   void ifShiftChangeGroup() {
     
     if (keyPressed && key==CODED && keyCode==SHIFT) {
       
        group[g].v=v.v;
        group[g].count=group[g].qtd; // number of controls that synchronize
     
     }
   }
}
