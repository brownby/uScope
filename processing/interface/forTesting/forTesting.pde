import ddf.minim.*;

Minim minim;
AudioInput in;

void setup() {
  
  surface.setResizable(true);
  surface.setSize(1000,150);
  
  minim = new Minim(this);
  in = minim.getLineIn(Minim.MONO, 1000, 44100, 8);

}

void draw() {
  
  background(0);
  stroke(255);
  
  for(int i = 0; i < in.bufferSize()-1; i++) { 
    
    line( i, 75 + in.left.get(i)*75, i+1, 75 + in.left.get(i+1)*75 );
  
  }
}
