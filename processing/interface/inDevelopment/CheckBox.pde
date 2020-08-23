class CheckBox {
   
   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   int tSize;  // textSize
   //color cor; // cor do fundo
   //color corBack; // cor do fundo do texto
   
   boolean clicked = false;
   boolean blink = false;
   
   String tex, tex2="";
   //constructor
   CheckBox(String tex_, int x_, int y_, int tSize_){
     tex=tex_; x=x_; y=y_; tSize=tSize_; //h=h_; cor=cor_; corBack=corBack_;
     textSize(tSize);
     h=tSize;
     w=(int)textWidth(tex)+h+5; 
     //println("w=",w);
   }
   void display(){
      //noFill(); stroke(255); strokeWeight(1);  (x,y,w,h);
      if (blink){
        fill(map(millis()%1000,0,1000,0,125));
      }else{
        fill(0);
      }
      textAlign(LEFT,CENTER); 
      textSize(14); text(tex,x+h+5,y+h/2-2);
      if (clicked) {
         fill(0,200,0); 
      } else {
         noFill();
      }
      stroke(0);strokeWeight(1); rect(x+2,y+2,h-2,h-4); //ellipse(x+h/2,y+h/2,0.6*h,0.6*h); //rect(x+w-h,y,h,h);
      fill(0);
        //println("clicked=",clicked," tex2=",tex2);
        
      if (clicked && tex2.length()>0){
         text(tex2,x+5,y+1.75*h);
        
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
