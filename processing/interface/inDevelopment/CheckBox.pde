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
   void display() {
     
      if (blink) { fill(map(millis()%1000,0,1000,0,125)); }
      else { fill(0); }
      
      textAlign(LEFT,CENTER); 
      textSize(14); 
      text(txt, x+h+5, y+h/2-2);
      
      if (clicked) { fill(0,200,0); }
      else { noFill(); }
      
      stroke(0); strokeWeight(1); rect(x+2,y+2,h-2,h-4); 
      fill(0);
        
      if (clicked && txt2.length()>0){
         text(txt2,x+5,y+1.75*h);
      }
   }
   
  boolean mouseClicked(){
     boolean r=false;
     if (mouseX>x && mouseX<x+w & mouseY>y && mouseY<y+h){
        clicked=!clicked;
        r=true;
     }
     return r;
  } 
}
