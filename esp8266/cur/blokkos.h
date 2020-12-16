#ifndef BLOKKOS_H_INCLUDED
#define BLOKKOS_H_INCLUDED

void measurement_performMeasure()
{
  if(first)
  {
    first=false;
    ido=millis(); //megnézem hogy milyen időpillanatban indult a mérés
  }
  double ertek = analogRead(A_PIN);
  if (counter == 0)
  {
    minimum = ertek;
    maximum = ertek;
  }
  /*if (Serial.availableForWrite()<8)
  {
    Serial.println("USBoverload");
  }*/
  else {
    
  }

  if (maximum < ertek)
  {
    maximum = ertek;
  }
  if (minimum > ertek)
  {
    minimum = ertek;
  }
  

  counter++;
  if(counter == 100)
  {
    //Serial.print("maximum: ");
      /*Serial.println(maximum);
      //Serial.print("minimum: ");
      Serial.println(minimum);*/
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
    else{
      double maxvoltage = (maximum - offset) / 1024 * 5;
      double maxi = (maxvoltage) / 100;
      double maxI = maxi * 1000;
      double minvoltage = (minimum - offset) / 1024 * 5;
      double mini = (minvoltage) / 100;
      double minI = mini * 1000;
      Serial.print("maxI:");
      Serial.println(maxI);
      Serial.print("minI:");
      Serial.println(minI);

      performUpload("esp32currentMeasurement/max", maxI);
      performUpload("esp32currentMeasurement/min", minI);
    }
    
    counter = 0;
  }
  izecount++;

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

#endif // BLOKKOS_H_INCLUDED
