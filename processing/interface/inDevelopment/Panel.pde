class Panel {
  
   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   String  txt   = "";
   String  txt2  = "";
   Boolean blink = false;
   
   Panel(String txt_, int x_, int y_, int w_, int h_) {  // constructor
     txt=txt_;
     x=x_; y=y_; w=w_; h=h_;
   }
   
   void display(){
      
      strokeWeight(1); 
      fill(200); 
      stroke(0);
      rect(x,y,w,h);
      
      if (blink){
        fill(map(millis()%1000,0,1000,0,255));
      }
      else {
        fill(0);
      } 
      
      textAlign(LEFT); text(txt+" "+txt2,x+5,y+textAscent());
      
   } 
}
