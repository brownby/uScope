class Button{
  
   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   String txt;
   boolean mPressed=false;
   boolean clicked=false;
   color rgb_active  = color(0,255,0);
   color rgb_outline = color(0);
   
   
   Button(String txt_,int x_,int y_, int w_, int h_) {  //constructor
   
      txt=txt_; x=x_; y=y_; w=w_; h=h_;
   
   } 
   
   Button(String txt_,int x_,int y_, int w_, int h_,color rgb_active_, color rgb_outline_) {
     
      txt=txt_; x=x_; y=y_; w=w_; h=h_;
      rgb_active = rgb_active_;
      rgb_outline = rgb_outline_;
      
   }
   
   void display() {
     
     if (mPressed){ fill(0,100,0); } 
     else if (clicked){ fill(rgb_active); } 
     else{ fill(200); } 
     
     stroke(rgb_outline); strokeWeight(2); 
     rect(x,y,w,h); 
     
     if (rgb_active==color(0,0,255) && clicked){ fill(255); } else{ fill(0); } // use white for text on blue background color
     textAlign(CENTER,CENTER); textSize(14); 
     text(txt,x+w/2,y+h/2);

   }
   
   void mousePressed() {
     
     if (mouseButton==LEFT) {
       if (mouseX>x && mouseX<x+w && mouseY>y && mouseY<y+h){
          mPressed=true;
       }
     } 
   }
   
   void mouseReleased() {
     
      if (mPressed) mPressed=false;
   
   }
   
   boolean mouseClicked() { // returns if clicked or not
     
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
