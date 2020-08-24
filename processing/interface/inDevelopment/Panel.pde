class Panel {
  
   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   color RGB;
   
   String  txt   = "";
   String  txt2  = "";
   Boolean blink = false;
   
   Panel(String txt_, color RGB_, int x_, int y_, int w_, int h_) {  // constructor
   
     txt = txt_;
     RGB = RGB_;
     x = x_; 
     y = y_; 
     w = w_; 
     h = h_;
   
   }
   
   void display(){
      
      strokeWeight(2); stroke(RGB); noFill();
      rect(x,y,w,h);
      
      fill(RGB);
      rect(x,y,w/2+w/4,15);
      
      if (blink){
        fill(map(millis()%1000,0,1000,0,255));
      }
      else {
        fill(0);
      } 
      
      textAlign(LEFT); text(txt+" "+txt2,x+5,y+textAscent());
      
   } 
}
