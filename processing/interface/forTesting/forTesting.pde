import ddf.minim.*;

Minim minim;
AudioInput in;

void setup() {
  
  surface.setResizable(true);
  surface.setSize(1000,250);
  minim = new Minim(this);

  in = minim.getLineIn(Minim.MONO,1023,44100, 8);

}

void draw() {
  
  background(0);
  stroke(255);
  
  for(int i = 0; i < in.bufferSize() - 1; i++) {
    
    line( i, 50 + in.left.get(i)*50, i+1, 50 + in.left.get(i+1)*50 );
    line( i, 150 + in.right.get(i)*50, i+1, 150 + in.right.get(i+1)*50 );
  
  }
}
