#ifndef CSUSZOABLAK_H_INCLUDED
#define CSUSZOABLAK_H_INCLUDED

double ertekek[100];
double maxertek;
double minertek;

int maxid;
int minid;

void nullaz()
{
  for(int i=0;i<100; i++)
  {
    ertekek[i]=0;
  }
}

void kiertekel()
{
  maxertek=ertekek[0];
  //maxid=0;
  //minid=0;
  minertek=ertekek[0];
  for(int i=0; i<100; i++)
  {
    if(ertekek[i]>maxertek)
    {
      maxertek=ertekek[i];
      //maxid=i;
    }
    if(ertekek[i]<minertek)
    {
      minertek=ertekek[i];
      //minid=i;
    }
    #ifdef nullertekvizsgalat
    
    #endif
  }
}

void measurement_performMeasure()
{
  if(first)
  {
    first=false;
    ido=millis(); //megnézem hogy milyen időpillanatban indult a mérés
  }
  double ertek = analogRead(A_PIN);
  ertekek[counter]=ertek;
  kiertekel();
  counter++;
  
  //Serial.println(maxI);
  if(digitalRead(7)==HIGH && !motvaDigitMeres && lastButtonTime<fullTime-1000000)
    {
      offset=ertek;
      motvaDigitMeres=true;
      lastButtonTime=fullTime;
      Serial.println("reset");
    }
    else
    {
      motvaDigitMeres=false;
    }
  if(counter==100)
  {
    counter=0;
    izecount++;
    double maxvoltage = (maxertek - offset) / 1024 * 5;
      double maxi = (maxvoltage) / 100;
      double maxI = maxi * 1000;
      double minvoltage = (minertek - offset) / 1024 * 5;
      double mini = (minvoltage) / 100;
      double minI = mini * 1000;
      Serial.print("maxI:");
      Serial.println(maxI);
      Serial.print("minI:");
      Serial.println(minI);
      performUpload("esp32currentMeasurement/max", maxI);
      performUpload("esp32currentMeasurement/min", minI);
  }

  if(izecount==10)
  {
    izecount=0;
     Serial.println(millis());//megnézem meddig tartott a mérés
  }
  /*if(millis()>(ido+1000UL) &&! votma)
    {
      votma=true;
      Serial.println(izecount);
    }*/
}

#endif // CSUSZOABLAK_H_INCLUDED
