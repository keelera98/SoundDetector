#include "arduinoFFT.h"
#include <LiquidCrystal.h>

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
//samples can be changed to a higher value but it might run out of memory
const uint16_t samples = 32; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  Serial.println("Ready");
  lcd.begin(16, 2);
}

void loop()
{
  /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  lcd.clear();
  /*Print the results of the sampling according to time */
  /*Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME);*/
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  /*Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME);*/
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  /*Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);*/
  /*Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);*/
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //lcd.println("Computed magnitudes:");
  //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  lcd.print(x); //Print out what frequency is the most dominant.
    //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  int low = 0;
  int high = (samples >> 1);
  double largeNumber = binarySearch(vReal, low, high);
  //prints values to LCD, the setCursor is what moves the dB levels to the second line
  lcd.print(" Hz");
  lcd.setCursor(0, 1);
  lcd.print(largeNumber);
  lcd.print(" dB");
  lcd.setCursor(0, 0);
  lcd.display();
  //while(1); /* Run Once */
  delay(2000); /* Repeat after delay */
}

//Our binary search method that finds the highest magnitude and returns it
//uses a pointer to the main array that holds the samples
double binarySearch(double *vData, int low, int high){
  if(low == high){
    return vData[low];
  }
  if((high == low + 1) && vData[low] >= vData[high]){
     return vData[low];
  }

  if((high == low + 1) && vData[low] < vData[high]){
    return vData[high];
  }

  int mid = (low + high)/2;

  if(vData[mid] > vData[mid + 1] && vData[mid] > vData[mid - 1]){
    return vData[mid];
  }

  if(vData[mid] > vData[mid + 1] && vData[mid] < vData[mid - 1]){
    return binarySearch(vData, low, mid - 1);
  }else{
    return binarySearch(vData, mid + 1, high);
  }
}

/*No longer used by the program, but the method orgininally printed out a list of the frequency 
samples as well as the magnitude associated with that*/
void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    //lcd.print(abscissa);
    //if(scaleType==SCL_FREQUENCY)
      //lcd.print("Hz");
    //lcd.print(" ");
    lcd.print(vData[i]);
    lcd.print(" ");
  }
  //lcd.println();
}
