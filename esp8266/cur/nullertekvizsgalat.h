#ifndef NULLERTEKVIZSGALAT_H_INCLUDED
#define NULLERTEKVIZSGALAT_H_INCLUDED

  
  int elozoid=0;
  int elozoelottiid=0;
  double ertekek[100];
  double maxertek;
  double minertek;
  int elozoertek;
  
  int maxid=0;
  int minid=0;
  int valtocounter=0;

  double aramszamol(double a)
  {
    double voltage = (double)(a - offset) / 1024 * 5;
    double i = (voltage) / 100;
    return i * 1000;
  }
  
  void nullaz()
  {
    for(int i=0;i<100; i++)
    {
      ertekek[i]=0;
    }
  }

  bool signum(double x)
  {
    if(x>0)
    {
      return true;
    }
    else if(x<0)
    {
      return false;
    }
    else
    {
      return false;
    }
  }

  /*void kiertekel(double ujertek)
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
  }*/

  void measurement_performMeasure()
{
  if(first)
  {
    first=false;
    ido=millis(); //megnézem hogy milyen időpillanatban indult a mérés
  }
  double ertek = analogRead(A_PIN);

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

  double voltage = (ertek - offset) / 1024 * 5;
  double i = (voltage) / 100;
  ertekek[counter] = i * 1000;
  //Serial.println(aramszamol(ertek));

  if(signum(aramszamol(ertek))!=signum(aramszamol(elozoertek)) && counter!=0)
  {
    valtocounter++;
    if(signum(ertekek[elozoid]))
    {
      maxertek=ertekek[0];
      for(int i=0; i<counter; i++)
      {
        if(ertekek[i]>maxertek)
        {
          maxertek=ertekek[i];
        }
      }
      Serial.print("max: ");
      Serial.println(maxertek);
      performUpload("esp32currentMeasurement/max", maxertek);
      
    }
    else
    {
      minertek=ertekek[0];
      for(int i=0; i<counter; i++)
      {
        if(ertekek[i]<minertek)
        {
          minertek=ertekek[i];
        }
      }
      Serial.print("min: ");
      Serial.println(minertek);
      performUpload("esp32currentMeasurement/min", minertek);
    }
    counter=0;
  }
  elozoertek=ertek;
  elozoelottiid=elozoid;
  elozoid=counter;
  counter++;
  if(counter==100)
  {
      /*maxertek=ertekek[0];
      minertek=ertekek[0];
      
      for(int i=0; i<counter; i++)
      {
        if(ertekek[i]<minertek)
        {
          minertek=ertekek[i];
        }
        if(ertekek[i]>maxertek)
        {
          maxertek=ertekek[i];
        }
      }
      Serial.print("max: ");
      Serial.println(maxertek);
      Serial.print("min: ");
      Serial.println(minertek);*/
      counter=0;
      //offset=ertek;
  }
  izecount++;
  if(izecount==1000)
  {
    izecount=0;
     /*Serial.println(millis());//megnézem meddig tartott a mérés
     Serial.print("valtasok:");
     Serial.println(valtocounter);*/
  }
  /*if(millis()>(ido+1000UL) &&! votma)
    {
      votma=true;
      Serial.println(izecount);
    }*/
}

#endif // NULLERTEKVIZSGALAT_H_INCLUDED
