/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Code for graphical interface
 * 
 * J. Evan Smith, Ben Y. Brown, modified from work by Rogerio Bego
 * Last revised: 23 August 2020
 *
 * =========== OUTLINE ===========
 *
 * ☑ import libraries
 * ☑ variable initalization
 * ☑ object instantiation
 * ☑ setup 
 * ☑ draw
 * ☐ mouseClicked 
 * ☐ mousePressed
 * ☐ mouseReleased
 * ☐ mouseMoved
 * ☐ mouseDragged
 * ☐ adjustFt
 * 
 * =========== CLASSES ===========
 *
 * ☐ Button
 * ☐ Channel
 * ☐ CheckBox
 * ☐ Dial
 * ☑ Display 
 * ☐ FmtNum
 * ☑ Group
 * ☑ Panel
 *
 * https://github.com/brownby/uScope/tree/usb-dev/processing/interface
 */


// *** import libraries *** //

import ddf.minim.*;  // used to connect to device over USB audio


// *** variable initialization *** //

String version="alpha";

boolean nInt = true;  // n is an integer (round) or decimal !nInt 
boolean fmt = true;   // fmt = true = "format", !fmt = false = "no format"

byte scaleLinear = 0;   
byte scaleLog = 1;     
boolean dtError = false; // check for sampling time error

byte changeMove = 2;     // value changed by "MouseDragged"
byte changeRelease = 3;  // value changed by "MouseReleased"

boolean waitforTrigger = false;   
int vTrigger = 0;                // value of trigger 0-1024 (0-5V), if 10 bit ADC 

byte numCh = 2;
color rgb[]={color(255, 255, 0), color(0, 0, 255)};  // for 2 channels: yellow (CH0) and blue (CH1)

float Q=45.0;     // division unit size
int marg1, marg2; // to adjust the position of objects

// *** object instantiation *** //

Minim minim;
AudioInput in; // USB connection to device

Channel channel[] = new Channel[numCh];
Group group[] = new Group[numCh+1]; // used to change V/div and ms/div simultaneously on all channels using SHIFT key

Display display;

Button    startStop;
Button    resetAxes;
Button    resetMedir;  // measure vs size? *flag

CheckBox  showSamples; 
CheckBox  calcFreq;    // detect frequency

Panel     pnlSamples;          // panel for sampling controls
Button    oneSample;           // request a sample
Button    severalSamples;      // request several samples
Button    streamContinuous;    // enters reading every dt
Dial      dt;                  // delta t (time of each reading)
Dial      q;                   // number of readings
FmtNum    tTotal;              // total sampling time dt*q
FmtNum    tTotalReal, dtReal;  // check if the real sample time is the same as desired

Panel     pnlWave;   // panel for the waveform generator
CheckBox  wave;      // f and t are dependent: f = 1/t, t = 1/f
Dial      fWave;     // frequency of waveform (0.125Hz-10kHz) 
Dial      tWave;     // period of waveform (100us-8s)
Dial      dutyWave;  // duty cycle (0-100%)

void setup() {
  
  size(1040, 635); 
  frameRate(30);

  display = new Display(30+10, 60, 17*Q, 12*Q);  // 17 horizontal and 12 vertical divisions
  
  marg1 = display.x+display.w+10; 
  marg2 = marg1+200;
  
  minim = new Minim(this);
  in = minim.getLineIn(Minim.MONO, 1000, 44100, 8);
  in.disableMonitoring();

  for (byte k=0; k<numCh+1; k++){ // must be completed before channels
    group[k] = new Group(); 
  }
   
  for (byte k=0; k<numCh; k++) {
    channel[k] = new Channel(k, rgb[k], marg1+20, display.y+90+k*130, 185, 100); //h115
  }
  
  startStop        = new Button("start / stop",marg1+20,channel[0].y-70,185,40);
  resetAxes        = new Button("axes",marg1+70,channel[1].y+channel[1].h+30,45,20);
  resetMedir       = new Button("size",resetAxes.x+resetAxes.w+2,channel[1].y+channel[1].h+30,45,20);
  
  showSamples      = new CheckBox("show samples", marg1+20, channel[1].y+channel[1].h+70, 15);
  calcFreq         = new CheckBox("detect frequency", showSamples.x, showSamples.y+showSamples.h+5, 15);

  pnlSamples       = new Panel("sampling", display.x+785, display.y+display.h-85, 200, 85);
  dt               = new Dial(scaleLog, changeRelease, nInt, fmt, "dt", "s", 24e-6f, 10e-6f, 2f, pnlSamples.x+5, pnlSamples.y+20, 100, 20);
  dtReal           = new FmtNum(0,nInt,fmt);
  q                = new Dial(scaleLinear, changeRelease, nInt, !fmt, "q", "", 1000-1, 1, 100, dt.x+dt.w+5, dt.y, 60, 20);
  tTotal           = new FmtNum(dt.v.getV()*q.v.getV(), !nInt);
  tTotalReal       = new FmtNum(0,!nInt);
  oneSample        = new Button("one", dt.x, dt.y+dt.h+5, 50, 20);
  severalSamples   = new Button("many", oneSample.x+oneSample.w+5, oneSample.y, oneSample.w, oneSample.h);
  streamContinuous = new Button("contin", severalSamples.x+severalSamples.w+5, severalSamples.y, severalSamples.w, severalSamples.h);
 
}


void draw() {

  background(110); 
  fill(244, 244, 244); 
  
  display.display();
  
  textAlign(LEFT, TOP);
  textSize(24); 
  text("μScope "+version, display.x, 20);

  textAlign(RIGHT, CENTER); 
  textSize(15);  
  text("RESET",resetAxes.x-10,resetAxes.y+resetAxes.h/2);
  
  for(int i = 0; i < in.bufferSize()-1; i++) { 
    
    channel[0].buffer[i]= int(in.left.get(i)*300)+40;
  
  }
  channel[0].updated=true;

  for (byte k=0; k<numCh; k++) {
    channel[k].display();
  }

  startStop.display();
  resetAxes.display();
  resetMedir.display();
  showSamples.display();
  calcFreq.display();

  tTotal.setV(dt.v.getV()*q.v.getV());
  pnlSamples.tex2="("+tTotal.printV()+"s)";
  
  pnlSamples.display();
  dt.display();
  q.display();  
  oneSample.display();
  severalSamples.display();
  streamContinuous.display();
  
  textAlign(LEFT);
  if (dtError){ fill(255,0,0); } else { fill(0,20,0); }  
  String tex="Real: dt"+dtReal.printV()+"s";
  if (streamContinuous.clicked==false){
     tex+="  total"+tTotalReal.printV()+"s"; 
  }
  text(tex,pnlSamples.x+5,pnlSamples.y+pnlSamples.h-2);
  fill(0);

}


void mouseClicked() {
 

  if (resetAxes.mouseClicked()){
    for (int k=0; k<numCh;k++){
     channel[k].p0=display.y+3*Q*(k+1);//posição da tensão zero
    }
    resetAxes.clicked=false;
  }
  
  if (resetMedir.mouseClicked()){
     for (int k=0; k<numCh;k++){
        channel[k].telaClicou=false; 
     }
     resetMedir.clicked=false;
  }
  
  showSamples.mouseClicked();
  calcFreq.mouseClicked();
  //showDif.mouseClicked();

  //se clicou em dt ou q então send comando para garagino e ajustar display
  if (dt.mouseClicked()) { // se true alterou dt, então ajustarFt() (escala de t na display)

    adjustFt();
  }
  if (q.mouseClicked()) { // se true alterou q, então ajustarFt()

    adjustFt();
  }


  if (oneSample.mouseClicked()) { // receber apenas Uma Amostra
    severalSamples.clicked=false;
    streamContinuous.clicked=false;

    oneSample.clicked=false;
    // verificar se tem algum trigger acionado para que ele fique waitfor o disparo
    // vai ficar piscando para indicar que está aguardando o disparo.
    int k2=-1;
    for (int k=0; k<numCh;k++){
      if (channel[k].trigger.clicked) {
         k2=k;
         break; 
      }
    }
    println("k2=",k2);
    
    if (k2>=0 && k2<=3){
       pnlSamples.blink=true;
       channel[k2].trigger.blink=true;
       waitforTrigger=true;
    } else {
       pnlSamples.blink=false;
       waitforTrigger=false;
     }
  }
  if (severalSamples.mouseClicked()) {
    oneSample.clicked=false;
    streamContinuous.clicked=false;
  }
  if (streamContinuous.mouseClicked()) {
    oneSample.clicked=false;
    severalSamples.clicked=false;
  }

}

void mousePressed() {
  //d.mousePressionou(); 
  for (int k=0; k<numCh; k++) {
    channel[k].mousePressionou();
  }
  dt.mousePressionou();
  q.mousePressionou();
  //ruido.mousePressionou();

  // só para aparecer o verde do pressionado
  oneSample.mousePressionou();
  severalSamples.mousePressionou();
  streamContinuous.mousePressionou();

 // fWave.mousePressionou();
 // tWave.mousePressionou();
 // dutyWave.mousePressionou();
  
  resetAxes.mousePressionou();
  resetMedir.mousePressionou();

}

void mouseReleased() {
  // d.mouseSoltou();
  for (int k=0; k<numCh; k++) {
    channel[k].mouseSoltou();
  }


  resetAxes.mouseSoltou();
  resetMedir.mouseSoltou();
  // só para aparecer o verde do pressionado
  oneSample.mouseSoltou();
  severalSamples.mouseSoltou();
  streamContinuous.mouseSoltou();


  //se Releaser o mouse no dt ou q, então send os dados para o Garagino
  if (dt.mouseSoltou()) {

    adjustFt();
  }
  if (q.mouseSoltou()) {

    // acertar as escalas ft de cada channel
    adjustFt();
  }
}

void mouseMoved() {
  
  for (int k=0; k<numCh; k++) {
    channel[k].mouseMoveu();
  } 
  dt.mouseMoveu();
  q.mouseMoveu();

}

void mouseDragged() {
  //d.mouseArrastou(); 
  for (int k=0; k<numCh; k++) {
    channel[k].mouseArrastou();
  }
  dt.mouseArrastou();
  q.mouseArrastou();

  
}

void adjustFt() {
  float ftNew=dt.v.getV()*q.v.getV()/10.0;
  //println("ftNew=",ftNew," dt=",dt.v.getV()," q=",q.v.getV());
  for (int k=0; k<numCh; k++) {
    channel[k].ft.setV(ftNew);
  }
}
  
void dummy() {
  //if (p.available()>0) {}
  
  String cmd="", val="";
  String tex="";//p.readStringUntil(10);
  
  //print(">>>> ",tex);
  if (tex.charAt(0)=='>') { //comando: >cmd=v1(tab)v2(tab)v3(tab)
    int i=tex.indexOf("=");
    if (i>=0) { // encontrou wave "=" (igual)  obs: i=-1 => não encontrou o wave '='
      cmd=tex.substring(1, i); // pegar o comando obs: substring(inclusive,exclusive)
      val=tex.substring(i+1); // pegar o valor
      //println("cmd=",cmd," val=",val);
      if (cmd.equals("f")) { // entra fluxo de dados - deslocar dados e armazenar no final
        String tex2[]=splitTokens(val); //val = "0(t)dtReal(t)ch0(t)ch1(t)ch2"
        //int vc[]=int(splitTokens(val));
        
        //move the data down to include the new data at the end
        for (int j=0; j<4; j++) {
          for (int k=1; k<q.v.v; k++) {
            channel[j].v[k-1]=channel[j].v[k];
          }
        }
        
        channel[0].v[int(q.v.v-1)]=int(tex2[2]);
        channel[1].v[int(q.v.v-1)]=int(tex2[3]);
        channel[2].v[int(q.v.v-1)]=int(tex2[4]);
        channel[3].v[int(q.v.v-1)]=int(tex2[5]);
        
        dtReal.setV(float(tex2[1]));
        if (dtReal.v-dt.v.v>1.1*dt.v.v){ dtError=true;} else {dtError=false;}
        println("cmd=",cmd," val=",val," dtReal=",dtReal.printV());
      } else if (cmd.equals("v")) { // entrada de Varias Amostra
        int v[]=int(splitTokens(val));
        //println("v.length=",v.length);
        int kk=v[0]; // indice da matriz
        
        channel[0].buffer[v[0]]=v[1];
        channel[1].buffer[v[0]]=v[2];
        channel[2].buffer[v[0]]=v[3];
        channel[3].buffer[v[0]]=v[4];
        
      } else if (cmd.equals("q")) { // quantidade de variaveis
        //q.val=float(val);
      } else if (cmd.equals("dt")) { // tamanho do dt (ms)
        //dt.val=float(val);
      } else if (cmd.equals("tTotalReal")) { // tempo total da amostra
        //println("updated");
        tTotalReal.setV(float(val));
        //text(tTotalReal,pnlSamples.x+2,pnlSamples.y+pnlSamples.h);
        println("cmd=",cmd," val=",val," tTotalReal=",tTotalReal.printV());
        channel[0].updated=true;  // terminou de entrar os dados então
        channel[1].updated=true;  //  carregar do buffer
        channel[2].updated=true;
        channel[3].updated=true;
        if (waitforTrigger){
           waitforTrigger=false;
           pnlSamples.blink=false;
           for (int k=0; k<4;k++){
             channel[k].trigger.blink=false;
           }
           
        }
      } else if (cmd.equals("dtReal")){
        dtReal.setV(float(val));
        if (dtReal.n>dt.v.n+10){ dtError=true;} else {dtError=false;}
        //text(dtReal,pnlSamples.x+2,pnlSamples.y+pnlSamples.h-12);
        println("cmd=",cmd," val=",val," dtReal=",dtReal.printV());
        
      } else if (cmd.equals("r") || cmd.equals("c") || cmd.equals("rc")) { // valor do resistor
        String tex2[]=splitTokens(val, "\t\r");
        
      } else if (cmd.charAt(0)=='?') {  // carregando as configurações do Garagino (ao conectar) 
        cmd=cmd.substring(2); // eliminar 2 caracteres iniciais "? comando"
        val=val.substring(0,val.length()-2); // eliminar 2 caracteres finais:  \n\r(13,10)(^M^J) (retorno de linha)        
        println("cmd=",cmd," val=",val);
        if (cmd.equals("q")){ // val=100
          q.v.v=float(val);
        } else if (cmd.equals("dt")){
          char unid=val.charAt(val.length()-2);
          val=val.substring(0,val.length()-2);
          println("unid=",unid," val=",val);
          if (unid=='u'){
            val=val+"e-6";            
          }else{
            val=val+"e-3";
          }
          println("val=",val);
          dt.setV(float(val));
          adjustFt();
          
        }else if (cmd.equals("channelTrigger")){ // val= 0,1,2,x
           for (int k=0;k<4;k++){channel[k].trigger.clicked=false;}
           if (!val.equals("x")){
              channel[int(val)].trigger.clicked=true;   
           }
        } else if (cmd.equals("uma")){ // val= 0 ou 1
          //oneSample.clicked=boolean(int(val));
        }else if (cmd.equals("varias")){ // val= 0 ou 1
          severalSamples.clicked=boolean(int(val));
        }else if (cmd.equals("fluxo")){ // val= 0 ou 1
          streamContinuous.clicked=boolean(int(val));
        }else if (cmd.equals("pwmOn")){ // val=0 ou 1 (false/true) 
          wave.clicked=boolean(int(val));
        }else if (cmd.equals("pwmP")){ // cmd="pwmP", val=" 100000us"
          val=val.substring(0,val.length()-2)+"e-6"; // remover "us" e colocar "e-6" (microsegundos)
          tWave.setV(float(val));
          fWave.setV(1/tWave.v.v);
          //println("pwmP=",float(val));
        }else if (cmd.equals("pwmPon")){  // cmd="pwmPon", val="25%"
          val=val.substring(0,val.length()-1);
          dutyWave.setV(float(val));
          println("pwmPon=",float(val));
        }
      }
    }
  }
}
