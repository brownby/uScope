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
     
      // faz retangulo
      //if (clicked) {stroke(100,0,0);} else {stroke(0);}
      stroke(rgb);
      strokeWeight(1);   fill(200);  rect(x,y,w,h); 
      // faz o valor v
      noStroke();  fill(244,244,244); rect(x+1,y+1,cx-x-2,h-2);
      
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
        // faz o triangulo do cursor
         fill(250,250,0); stroke(0);
        triangle(cx,y+3*h/4,cx-5,y+h,cx+5,y+h); //rect(cx-10,y,20,h);
        triangle(cx,y+h/4,cx-5,y,cx+5,y); //rect(cx-10,y,20,h);
      }

      // imprimir as linhas para delimitar as 6 areas de + -
      stroke(0);
      for (int k=0;k<5;k++){ 
        float vx=x+(k+1)*0.17*w;
        if (k==2){line(vx,y,vx,y+0.2*h);} 
          else {line(vx,y,vx,y+0.1*h);} 
       
      }

      
      //faz o texto
      fill(0); strokeWeight(2); textSize(12);  textAlign(CENTER,CENTER); 

      String t=txt+" ";
      if (clicked) {
        if (change==changeRelease){
          t+=vTemp.printV();
        } else {
          t+=v.printV();
        }
      } else {
        t+=v.printV();
      }
      text(t+unity,x+w/2,y+h/2-2);
  
  
   }
   
  
   
   int v2x(float v_){
      if (scale==scaleLinear) {
        return (int)map(v_,vMin,vMax,x,x+w);
      } else {
        return (int)map(log(v_)/log(10),log(vMin)/log(10), log(vMax)/log(10),x,x+w); 
      }
   }
   
   float x2v(int cx_){
     if (scale==scaleLinear){
       return map(cx_,x,x+w,vMin,vMax);
     } else{
       return pow(10,map(cx_,x,x+w,log(vMin)/log(10),log(vMax)/log(10)));
     }  
   }
   
   
   void groupUpdate(){
     if (g>0){
        if (group[g].conta>0){
           group[g].conta--;
           setV(group[g].v);
           if (group[g].conta<=0) group[g].v=0;
        }
     }
       
   }
   
   
   boolean mouseClicked(){ // Soma/Subtrai 1,10 ou 100 do valor => true se alterou o valor
     boolean alterou=false;
     float v2=0;
     if (mouseX>x && mouseX<x+w && mouseY>y && mouseY<y+h){
       alterou=true;
       int p=(int)map(mouseX,x,x+w,1,7);
       //int p=round(map(mouseX,x,x+w,1,5));
       //println("p=",p);
       //println("fmtV=",fmtV(v));
       switch (p) {
          case 1: // subtrair 100
            v2=v.addN(-100);
            break;
          case 2: // subtrair 10 em 10
             //println("-10");
             v2=v.addN(-10);
           break;
          case 3: // subtrair de 1 em 1
            //println("-1");
            v2=v.addN(-1);
           break;
          case 4: // somar de 1 em 1
            //println("+1");
            v2=v.addN(+1);
           break;
          case 5: // somar de 10 em 10
            //println("+10");
            v2=v.addN(+10);
           break; 
          case 6: //somar
            v2=v.addN(+100);
            break;
       }
       if (v2<vMin) {
          v.setV(vMin); 
       } else if (v2>vMax){
          v.setV(vMax); 
       } else {
          v.setV(v2); 
       }
       updateCx();
       ifShiftAlterarGrupo();       
     } 
     return alterou;
   }

   void mouseMoveu(){
      if (mouseY>y && mouseY<y+h) {
        if (mouseX>cx-10 && mouseX<cx+10){
         // println("mouseMoveu Dial");
          showTriangles=true;
        } else {
          showTriangles=false;
        }
        if (mouseX>x && mouseX<x+w && keyPressed && keyCode==CONTROL){
          println("showIncrements=" + showIncrements);
           showIncrements=true; 
        } else {
           showIncrements=false;
        }
      }  else {
        showTriangles=false;
      }   
   }
   
   void mousePressed(){
     if (mouseButton==LEFT){
      if (mouseY>y && mouseY<y+h) {
        if (mouseX>cx-10 && mouseX<cx+10){
          //println("mousePressionado"); 
          clicked=true; 
           vTemp.setV(v.v);
           mouseOffSet=mouseX-cx;
           //println("cx=",cx);
        }
      }
     }
   }
   
   boolean mouseDragged(){ // retorna true se é para enviar o comando para Garagino
      //println("Dial.mouseDragged");
      boolean enviar=false;
      if (clicked){
         cx=constrain(mouseX-mouseOffSet,x,x+w);
         if (change==changeMove){ // é para change Imediatamente enquanto Mover o Mouse
            vTemp.setV(x2v(cx)); // converte o x para v
              v.setV(vTemp.v); 
              enviar=true;   // enviar o comando de change para o Garagino!
              ifShiftAlterarGrupo(); // se tiver SHIFT então change Grupo
         }else{
            vTemp.setV(x2v(cx));
         }
      }
     return enviar; 
   }
   
   boolean mouseReleased(){ // retorna true se é para enviar o comando para o Garagino
     boolean enviar=false;
      if (clicked) {
        clicked=false;
        if (change==changeRelease){
           if (mouseY>y-10 && mouseY<y+h+10) { // && mouseX>x-15 && mouseX<x+w+15){
               v.setV(vTemp.v); // é para change quando Soltar o Mouse
               enviar=true;  // enviar comando de change para o Garagino!
               ifShiftAlterarGrupo(); // se tiver SHIFT então change Grupo
            } else{
               cx=v2x(v.v);
           }
        }
    } 
    return enviar;
   }
   
   void ifShiftAlterarGrupo(){
     if (keyPressed && key==CODED && keyCode==SHIFT){
        group[g].v=v.v;
        group[g].conta=group[g].qtd; //quantidade de controles que irão sincronizar o valor
     }
   }
   
}
