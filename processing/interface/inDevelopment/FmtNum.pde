class FmtNum{
  
   float v;  // value as float
   float n;  // numeric part of formatted expression
   char  u;  // unit part of formatted expression
   
   int i; // index for units
   char unid[]={'f','p','n','u','m',' ','k','M','G','T','P'};  // pico(-12), nano(-9), micro(-6), milli(-3), (0), kilo(3), mega(6), giga(9), tera(12)
   
   boolean nInt     = false;  // true --> round n to integer value
   boolean format   = true;   // formatted as nu = numeric + unit

   FmtNum(float v_,boolean nInt_,boolean fmt_) {  //constructor
   
     v=v_;
     nInt=nInt_;
     v2nu(v);
     format=fmt_;
   
   }
   
   FmtNum(float v_,boolean nInt_) {
     
     v=v_;
     nInt=nInt_;
     v2nu(v);
     format=true;
   
   } 
   
   String printV() {
     
     if (nInt) {               // is integer?
       if (format) {           // is formatted?
         return nf(n,0,0)+u;  
       } 
       else {                  // needs to be formatted
         return str(int(v));
       }
     } 
     else{                     // is decimal
       if (format){            // is formatted?
         return str(n)+u;
       } else {                // needs to be formatted
         return str(v); 
       }
     }
     
   }
   
   void setV(float v_) {
     
     v=v_;
     v2nu(v);  
   
   }
   
   float getV() {
     
     if (nInt){ return int(n)*pow(10,(i-5)*3); } 
     else{ return v; }
     
   }

   void setNInt(){
     
       n=round(n);
       nu2v();
   }
   
   float addN(float k) { // add / substract n (if u = ' ' go to decimal place

      float n2=int(n);
      int i2=i;    
      
      if (n2+k>0) {
        n2+=k;
      } 
      else {
        if (i2>0) {
           i2--;
           n2=1000+k; 
        }
      }
      
      return n2*pow(10,(i2-5)*3);
      
   } 
   
   void v2nu(float v_) { // value to formatted numeric + unit?
   
    i = constrain(int((log(v_)/log(10)+15)/3),0,unid.length-1); // calculate the index of the exponent of the number (v_) in base 10
    
    if (nInt) {
      n = round(v_/pow(10,(i-5)*3));
    } 
    else {
      n = round((v_/pow(10,(i-5)*3))*10.0)/10.0;
    }
    
    u = unid[i];
    
   }
   
   void nu2v() { // formatted numeric + unit to value?
   
      v = n*pow(10,(i-5)*3);
      v2nu(v); 
      
   }
}
