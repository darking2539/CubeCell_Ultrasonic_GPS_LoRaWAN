#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "TinyGPS++.h"
#include <softSerial.h>


//GPS parameter
softSerial serial_connection(13, GPIO3); //
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
float latidute,longitude;

//HC-SR04 parameter
const int TRIG_PIN = GPIO1;
const int ECHO_PIN = GPIO2;
int mp =0;
unsigned long int pulse_width;
float cm;
const unsigned int MAX_DIST = 23200;


/* OTAA para*/
uint8_t devEui[] = { 0x30, 0x59, 0x23, 0x19, 0x02, 0x20, 0x23, 0x32 }; //3059231902202331
uint8_t appEui[] = { 0x30, 0x59, 0x23, 0x19, 0x02, 0x20, 0x23, 0x31 };
uint8_t appKey[] = { 0x16, 0x28, 0xae, 0x2b, 0x7e, 0x15, 0xd2, 0xa6, 0xab, 0xf7, 0xcf, 0x4f, 0x3c, 0x15, 0x88,0x09 };
/* ABP para*/
uint8_t nwkSKey[] = { 0x28, 0xae, 0xd2, 0x2b, 0x7e, 0x15, 0x16, 0xa6, 0x09, 0xcf, 0xab, 0xf7, 0x15, 0x88, 0x4f,0x3c };     //"28AED22B7E1516A609CFABF715884F3C"
uint8_t appSKey[] = { 0x16, 0x28, 0xae, 0x2b, 0x7e, 0x15, 0xd2, 0xa6, 0xab, 0xf7, 0xcf, 0x4f, 0x3c, 0x15, 0x88,0x09 };     //"1628AE2B7E15D2A6ABF7CF4F3C158809"
uint32_t devAddr =  ( uint32_t )0x007e6ae2;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60000*5;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/* Prepares the payload of the frame */
int Celsius;

static void prepareTxFrame( uint8_t port )
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
    
    digitalWrite(Vext, LOW);  //Open Vext   
    unsigned char *puc1, *puc2, *puc3, *puc4 ;

    //ultrasonic reader
    readlength();


    //prepare payload ultrasonic reader
    int distance_read = abs(cm*100);
    puc4 = (unsigned char *)(&distance_read);
    
    //battery reader
    int voltage = getBatteryVoltage()/10; //this voltage = realvoltage*10
    //Serial.print("Voltage =");
    //Serial.println(voltage);
    puc3 = (unsigned char *)(&voltage);
    
    // GPS reader
    readGPS();
    
    //prepare payload GPS
    int latitude_send = gps.location.lat()*10000; 
    int longitude_send = gps.location.lng()*10000;
    puc1 = (unsigned char *)(&latitude_send);
    puc2 = (unsigned char *)(&longitude_send);

    appDataSize = 23; 
    appData[0] = 0x00; //GPS Location
    appData[1] = 0x88;
    appData[2] = puc1[2];  //Latitude
    appData[3] = puc1[1];
    appData[4] = puc1[0];
    appData[5] = puc2[2];  //Longtitude
    appData[6] = puc2[1];
    appData[7] = puc2[0];
    appData[8] = 0x00;  //Fixed Altitude
    appData[9] = 0x00;
    appData[10] = 0x00;

    appData[11] = 0x01; //ultrasonic parameter
    appData[12] = 0x02;
    appData[13] = puc4[1];
    appData[14] = puc4[0];

    appData[15] = 0x02; //Battery Monitoring
    appData[16] = 0x02;
    appData[17] = puc3[1];
    appData[18] = puc3[0];

    appData[19] = 0x03; //fixed temp parameter
    appData[20] = 0x67;
    appData[21] = 01;
    appData[22] = 10;

    digitalWrite(Vext, HIGH);  //Close Vext

}


void setup() {
  //Initialize Board
  boardInitMcu();
  Serial.begin(115200);
  serial_connection.begin(9600);//This opens up communications to the GPS
  pinMode(Vext, OUTPUT); //Open Vext

  //GPIO for GPS
  pinMode(GPIO0, INPUT); //PPS Status(GPS)
  
  //GPIO for HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  //Start program
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();

}

void loop()
{
  
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(AT_SUPPORT)
      getDevParam();
#endif
      printDevParam();
      LoRaWAN.init(loraWanClass,loraWanRegion);
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      
      prepareTxFrame( appPort );
      
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}

void readGPS()
{
  
  while(serial_connection.available()==0 or digitalRead(GPIO0) == 0){
    //waiting for serial read
  }
    
  while(gps.location.isUpdated() == 0)//While there are characters to come from the GPS
  {
    
      while(serial_connection.available())//While there are characters to come from the GPS
       {
          gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time   
       }
  }
  Serial.println("DONE READ GPS");
}

void readlength() {

   
  mp=0;
  
  // Hold the trigger pin high for at least 10 us

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);



  // Wait for pulse on echo pin

  while ( digitalRead(ECHO_PIN) == 0 );



  // Measure how long the echo pin was held high (pulse width)

  // Note: th micros() counter will overflow after ~70 min

 

  while ( digitalRead(ECHO_PIN) == 1){
    mp++;
  }
  pulse_width = mp;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).

  cm = (pulse_width-4.4115) / 71.578;

  Serial.print(cm);
  Serial.print(" cm \t");    
  Serial.println(mp);
  
}
