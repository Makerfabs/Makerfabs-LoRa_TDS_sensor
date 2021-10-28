#include <RadioLib.h>

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

#define FREQUENCY 915.0
#define BANDWIDTH 125.0
#define SPREADING_FACTOR 9
#define CODING_RATE 7
#define OUTPUT_POWER 10
#define PREAMBLE_LEN 8
#define GAIN 0

SX1276 radio = new Module(LORA_CS, DIO0, LORA_RST, DIO1, SPI, SPISettings());



void setup()
{
    Serial.begin(115200);


    // initialize SX1278 with default settings
    Serial.print(F("[SX1278] Initializing ... "));

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
}

static unsigned long analogSampleTimepoint = millis();
static unsigned long printTimepoint = millis();
static unsigned long loraTimepoint = millis(); 
   
void loop()
{
   
    String str;
    int state = radio.receive(str);

    if (state == ERR_NONE)
    {
        // packet was successfully received
        Serial.println(F("success!"));

        // print the data of the packet
        Serial.print(F("[SX1278] Data:\t\t\t"));
        Serial.println(str);

        // print the RSSI (Received Signal Strength Indicator)
        // of the last received packet
        Serial.print(F("[SX1278] RSSI:\t\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        // print the SNR (Signal-to-Noise Ratio)
        // of the last received packet
        Serial.print(F("[SX1278] SNR:\t\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

        // print frequency error
        // of the last received packet
        Serial.print(F("[SX1278] Frequency error:\t"));
        Serial.print(radio.getFrequencyError());
        Serial.println(F(" Hz"));
    }
    else if (state == ERR_RX_TIMEOUT)
    {
        // timeout occurred while waiting for a packet
        //Serial.println(F("timeout!"));
    }
    else if (state == ERR_CRC_MISMATCH)
    {
        // packet was received, but is malformed
        Serial.println(F("CRC error!"));
    }
    else
    {
        // some other error occurred
        Serial.print(F("failed, code "));
        Serial.println(state);
    }

    // wait for a second before transmitting again
    delay(500);

}
