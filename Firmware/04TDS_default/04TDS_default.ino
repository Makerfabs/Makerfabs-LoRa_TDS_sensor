#include <RadioLib.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#define NODENAME "LORA_POWER_1"
String node_id = String("ID") + "040001";

//Set sleep time, when value is 1 almost sleep 20s,when value is 450, almost 1 hour.
#define SLEEP_CYCLE 450

//Lora set
//Set Lora frequency
#define FREQUENCY 434.0
//#define FREQUENCY 868.0
//#define FREQUENCY 915.0

#define BANDWIDTH 125.0
#define SPREADING_FACTOR 9
#define CODING_RATE 7
#define OUTPUT_POWER 10
#define PREAMBLE_LEN 8
#define GAIN 0

//328p
#define DIO0 2
#define DIO1 6

#define LORA_RST 4
#define LORA_CS 10

#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 13

//pin set
#define VOLTAGE_PIN A3
#define SENSOR_POWER_PIN A2


#define DEBUG_OUT_ENABLE 1

SX1278 radio = new Module(LORA_CS, DIO0, LORA_RST, DIO1);

bool readSensorStatus = false;
int sensorValue = 0; // variable to store the value coming from the sensor
int batValue = 0;    // the voltage of battery
int count = 0;
int ADC_O_1;           // ADC Output First 8 bits
int ADC_O_2;           // ADC Output Next 2 bits
int16_t packetnum = 0; // packet counter, we increment per xmission

#define one_wire_bus A1

OneWire oneWire(one_wire_bus);
DallasTemperature sensors(&oneWire);

#define TdsSensorPin A0
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;


static unsigned long analogSampleTimepoint = millis();
static unsigned long printTimepoint = millis();
static unsigned long LEDTimepoint = millis(); 


void Lora_init()
{
    int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE, SX127X_SYNC_WORD, OUTPUT_POWER, PREAMBLE_LEN, GAIN);
    if (state == ERR_NONE)
    {
#if DEBUG_OUT_ENABLE
        Serial.println(F("success!"));
#endif
    }
    else
    {
#if DEBUG_OUT_ENABLE
        Serial.print(F("failed, code "));
        Serial.println(state);
#endif
        // while (true)
        //     ;
    }
}
void setup()
{
#if DEBUG_OUT_ENABLE
    Serial.begin(115200);
    Serial.println("Soil start.");
#endif
    delay(100);

    // set up Timer 1

    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH);
    delay(100);

    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, HIGH); //Sensor power on
    delay(100);

    Lora_init();

    
    for(int i = 0; i<3;i++)
    {
      do_some_work();
      delay(1000);
    }
     
//setup over
#if DEBUG_OUT_ENABLE
    Serial.println("[Set]Sleep Mode Set");
    
#endif
    low_power_set();

}

void loop()
{
    wdt_disable();

    if (count > SLEEP_CYCLE) //(7+1) x 8S  450
    {
#if DEBUG_OUT_ENABLE
        //code start
        Serial.println("Code start>>");
#endif

        do_some_work();
        all_pins_low();

#if DEBUG_OUT_ENABLE
        //code end
        Serial.println("Code end<<");
#endif
        //count init
        count = 0;
    }

    low_power_set();
}

ISR(WDT_vect)
{
#if DEBUG_OUT_ENABLE
    Serial.print("[Watch dog]");
    Serial.println(count);
#endif
    delay(100);
    count++;
    //wdt_reset();
    wdt_disable(); // disable watchdog
}

//Set low power mode and into sleep
void low_power_set()
{
    all_pins_low();
    delay(10);
    // disable ADC
    ADCSRA = 0;

    sleep_enable();
    watchdog_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    delay(10);
    noInterrupts();
    sleep_enable();

    // turn off brown-out enable in software
    MCUCR = bit(BODS) | bit(BODSE);
    MCUCR = bit(BODS);
    interrupts();

    sleep_cpu();
    sleep_disable();
}

//Enable watch dog
void watchdog_init()
{
    // clear various "reset" flags
    MCUSR = 0;
    // allow changes, disable reset
    WDTCSR = bit(WDCE) | bit(WDE);
    WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0); // set WDIE, and 8 seconds delay
    wdt_reset();                                // pat the dog
}

void do_some_work()
{

    digitalWrite(SENSOR_POWER_PIN, HIGH); // Sensor/RF95 power on
    digitalWrite(LORA_RST, HIGH);
    delay(5);
    
    Lora_init();
    delay(50);
    
    ADMUX = _BV(REFS0) ;

    ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);

    for (int i = 0; i < 30; i++)
    {
        //start ADC conversion
        ADCSRA |= (1 << ADSC);

        delay(10);

        if ((ADCSRA & 0x40) == 0)
        {
            ADC_O_1 = ADCL;
            ADC_O_2 = ADCH;

            sensorValue = (ADC_O_2 << 8) + ADC_O_1;
            
            analogBuffer[analogBufferIndex] = sensorValue;
            analogBufferIndex++;
            if(analogBufferIndex == SCOUNT) 
                analogBufferIndex = 0;
                
            ADCSRA |= 0x40;

            if (readSensorStatus == false)
                ;

            Serial.println(sensorValue);
        }
        ADCSRA |= (1 << ADIF); //reset as required
        delay(50);
    }


       

    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX1) | _BV(MUX0);

    delay(50);
    for (int i = 0; i < 3; i++)
    {
        //start ADC conversion
        ADCSRA |= (1 << ADSC);

        delay(10);
        
        if ((ADCSRA & 0x40) == 0)
        {
            ADC_O_1 = ADCL;
            ADC_O_2 = ADCH;

            batValue = (ADC_O_2 << 8) + ADC_O_1;
            ADCSRA |= 0x40;
#if DEBUG_OUT_ENABLE
            Serial.print("BAT:");
            Serial.println(batValue);
            float bat = (float)batValue * 4.2;
            bat = bat / 1024.0;
            Serial.print(bat);
            Serial.print("V");
#endif
        }
        ADCSRA |= (1 << ADIF); //reset as required
        delay(50);
    }


    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
          
    printTimepoint = millis();
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
    //averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;
    float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
    tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value

    Serial.println(tdsValue);



    send_lora();
    delay(1000);
    radio.sleep();

    packetnum++;
    readSensorStatus = false;
    digitalWrite(SENSOR_POWER_PIN, LOW); // Sensor/RF95 power off
    delay(100);
}



void all_pins_low()
{

    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);

    delay(50);
}

void send_lora()
{
    String message = "INEDX:" + (String)packetnum + " TDS:" + (String)tdsValue + " T:" + (String)temperature + " BAT:" + (String)batValue;
    String back_str = node_id + " REPLY : TDS " + message;

#if DEBUG_OUT_ENABLE
    Serial.println(back_str);
#endif

    int state_t = radio.transmit(back_str);
    if (state_t == ERR_NONE)
      {
        // the packet was successfully transmitted
        Serial.println(F(" TRANSMIT success!"));
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
