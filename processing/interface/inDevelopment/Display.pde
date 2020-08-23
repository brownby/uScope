class Display { 

   int x;  // position
   int y;  // position 
   int w;  // width
   int h;  // height
   
   Display(float xi, float yi, float wi, float hi) { // constructor
      
      x=int(xi); y=int(yi); w=int(wi); h=int(hi);
   
   } 
   
   void display() {
     
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
