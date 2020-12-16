#ifndef CSUSZOABLAKIDVEL_H_INCLUDED
#define CSUSZOABLAKIDVEL_H_INCLUDED

double ertekek[100];
double maxertek;
double minertek;

int maxid=0;
int minid=0;

void nullaz()
{
  for(int i=0;i<100; i++)
  {
    ertekek[i]=0;
  }
}

void kiertekel(double ujertek)
{
  ertekek[counter]=ujertek;
  if(minid==counter || maxid==counter) //ha kicsúszott a min vagy a max újra kell vizsgálnunk a tömböt
  {
    maxertek=ertekek[0];
    maxid=0;
    minid=0;
    minertek=ertekek[0];
    for(int i=0; i<100; i++)
    {
      if(ertekek[i]>maxertek)
      {
        maxertek=ertekek[i];
        maxid=i;
      }
      if(ertekek[i]<minertek)
      {
        minertek=ertekek[i];
        minid=i;
      }
    }
  }
  else{ //egyébként megnézzük hogy az újonnan bejött érték nagyobb-e mint az eddigi max, vagy kisebb-e mint az eddigi min
    if(ertekek[minid]>ujertek)
    {
      minid=counter;
    }
    if(ertekek[maxid]<ujertek)
    {
      maxid=counter;
    }
  }
}

void measurement_performMeasure()
{
  if(first)
  {
    first=false;
    ido=millis(); //megnézem hogy milyen időpillanatban indult a mérés
  }
  double ertek=(double)egyMeres();
  kiertekel(ertek);
  counter++;
  /*double maxvoltage = (maxertek - offset) / 1024 * 5;
  double maxi = (maxvoltage) / 100;
  double maxI = maxi * 1000;*/
  //Serial.println(maxI);
  /*if(digitalRead(6)==HIGH && !motvaDigitMeres && lastButtonTime<fullTime-1000000)
    {
      offset=ertek;
      motvaDigitMeres=true;
      lastButtonTime=fullTime;
      Serial.println("reset");
    }
    else
    {
      motvaDigitMeres=false;
    }*/
  if(counter==100)
  {
    counter=0;
    izecount++;
      double maxvoltage = (maxertek - offset) / 4096 * 3.3f;
      double maxi = (maxvoltage) / 100;
      double maxI = maxi * 1000;
      double minvoltage = (minertek - offset) / 4096 * 3.3f;
      double mini = (minvoltage) / 100;
      double minI = mini * 1000;
      //Serial.print("maxI:");
      Serial.println(maxI);
      /*Serial.print("minI:");
      Serial.println(minI);*/
      performUpload("esp32currentMeasurement/max", maxI);
      performUpload("esp32currentMeasurement/min", minI);
    
  }

  if(izecount==10)
  {
    izecount=0;
     //Serial.println(millis());//megnézem meddig tartott a mérés
  }
  /*if(millis()>(ido+1000UL) &&! votma)
    {
      votma=true;
      Serial.println(izecount);
    }*/
}

#endif // CSUSZOABLAKIDVEL_H_INCLUDED
