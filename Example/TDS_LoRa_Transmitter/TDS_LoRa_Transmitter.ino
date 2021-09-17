#include <OneWire.h>
#include <DallasTemperature.h>
#include <RadioLib.h>


#define one_wire_bus A1

#define TdsSensorPin A0

#define LED 5
bool LEDState = false;

#define SensorPower A2

OneWire oneWire(one_wire_bus);
DallasTemperature sensors(&oneWire);

#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

//328p
#define DIO0 2
#define DIO1 6
//#define DIO2 7
//#define DIO5 8

#define LORA_RST 4
#define LORA_CS 10

#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 13

#define FREQUENCY 434.0
#define BANDWIDTH 125.0
#define SPREADING_FACTOR 9
#define CODING_RATE 7
#define OUTPUT_POWER 10
#define PREAMBLE_LEN 8
#define GAIN 0

SX1278 radio = new Module(LORA_CS, DIO0, LORA_RST, DIO1, SPI, SPISettings());



void setup()
{
    Serial.begin(115200);
    pinMode(TdsSensorPin,INPUT);
    pinMode(SensorPower, OUTPUT);
    pinMode(LED,OUTPUT);

    delay(1000);

    digitalWrite(SensorPower,HIGH);
    delay(1000);

    sensors.begin();

    SPI.begin();    
    //int state = radio.begin();
    int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE, SX127X_SYNC_WORD, OUTPUT_POWER, PREAMBLE_LEN, GAIN);
    if (state == ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
            ;
    }

    delay(1000);
    Serial.println("Test Begin:");
}

static unsigned long analogSampleTimepoint = millis();
static unsigned long printTimepoint = millis();
static unsigned long LEDTimepoint = millis(); 
   
void loop()
{
   
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   

   if(millis()-printTimepoint > 800U)
   {
      sensors.requestTemperatures();
      temperature = sensors.getTempCByIndex(0);
      
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      /*Serial.print("voltage:");
      Serial.print(averageVoltage,2);
      Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
      Serial.print("T:");  
      Serial.println(temperature);*/
      
   }
   if(millis()-LEDTimepoint > 1500U)
   {
      digitalWrite(LED, LEDState);
      LEDState = !LEDState;
      LEDTimepoint = millis();

      int state_t = radio.transmit("TDS Value:" + (String)tdsValue + "ppm  " + (String)temperature +"C");
      if (state_t == ERR_NONE)
      {
            // the packet was successfully transmitted
            Serial.println(F(" success!"));
            Serial.println("TDS Value:" + (String)tdsValue + "ppm  " + (String)temperature +"C");
    
            // print measured data rate
            Serial.print(F("[SX1278] Datarate:\t"));
            Serial.print(radio.getDataRate());
            Serial.println(F(" bps"));
       }
   }
   
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
    bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
    for (i = 0; i < iFilterLen - j - 1; i++) 
          {
      if (bTab[i] > bTab[i + 1]) 
            {
    bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
    bTab[i + 1] = bTemp;
       }
    }
      }
      if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
      else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
