/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/moebu/Documents/projects/weatherstationbedroom/src/weatherstationbedroom.ino"
// This #include statement was automatically added by the Particle IDE.
//#include <Adafruit_DHT_Particle.h>


//#define DHTPIN D2     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11		// DHT 11 
//#define DHTTYPE DHT22		// DHT 22 (AM2302)
//#define DHTTYPE DHT21		// DHT 21 (AM2301)




// This #include statement was automatically added by the Particle IDE.
#include <PMS7003-Particle-Sensor-Serial.h>


#include <bsec.h>

#include "blynk.h"
#include "math.h"

//#include "Adafruit_Sensor.h"
//#include "Adafruit_BME280.h"



void setup();
void loop();
#line 29 "c:/Users/moebu/Documents/projects/weatherstationbedroom/src/weatherstationbedroom.ino"
const uint8_t bsec_config_iaq[] = {2,9,4,1,61,0,0,0,0,0,0,0,182,1,0,0,52,0,1,0,0,192,168,71,64,49,119,76,0,0,97,69,0,0,97,69,137,65,0,191,205,204,204,190,0,0,64,191,225,122,148,190,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,1,35,41,29,86,88,0,9,0,229,208,34,62,0,0,0,0,0,0,0,0,218,27,156,62,225,11,67,64,0,0,160,64,0,0,0,0,0,0,0,0,94,75,72,189,93,254,159,64,66,62,160,191,0,0,0,0,0,0,0,0,33,31,180,190,138,176,97,64,65,241,99,190,0,0,0,0,0,0,0,0,167,121,71,61,165,189,41,192,184,30,189,64,12,0,10,0,0,0,0,0,0,0,0,0,229,0,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,44,1,0,0,0,0,159,253,0,0};
#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000)
void checkIaqSensorStatus(void);
void errLeds(void);
void updateState(void);
void loadState(void);
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;
String output;




#define tempoffset -0.9F




#define TTS 1000 //Time till screen update in ms
#define TTU 20 //Time till update in TTS intervals

//DHT dht(DHTPIN, DHTTYPE);
//int loopCount;
WidgetTerminal terminal(V14); //terminal widget

int timerBlynk = TTU;

bool drawChart = FALSE;

//char auth[] = "81f749960ec34003bbab37a5b4ffa61e"; //BLYNK
char auth[] = "eT_7FL7IUpqonthsAr-58uTK_-su_GYy"; //BLYNK

//Adafruit_BME280 bme; // I2C

float abshumBME, tempBME, presBME, humBME, gasBME, dewpoint, humidex;
float bmeiaq, bmeiaqAccuracy, bmestaticIaq, bmeco2Equivalent, bmebreathVocEquivalent, bmestabStatus, bmerunInStatus, bmegasPercentage;


float amtemp, amhum;

int timer1 = TTU; // How often to send sensor updates to Ubidots
int screentime = (TTS / 1000);
int debugcounter = 105;
int firstvalue = 1;

int history = 2;
int historycount = history;



float sensor;



int last_pushed = 0; // global variable

int ranged = 0;

float deltapres, deltatemp, deltahum = 0;
float old1p0, old2p5, old10, new1p0, new2p5, new10;
float tempDS18;

unsigned long lastmillis = 0;
unsigned long millisBlynk = 0;
unsigned long deltamillis = 0;
unsigned long rgbmillis = 0;
int pagetimer = 0;

int xPos = 0;


int zebraR, zebraG, zebraB, sliderValue;
int menuValue = 2;
float  pmR, pmG, pmB;
bool rgbON = true;





PMS7003Serial<USARTSerial> pms7003(Serial1, D6);



BLYNK_WRITE(V15)
{
   menuValue = param.asInt(); // assigning incoming value from pin V1 to a variable
}

BLYNK_WRITE(V16)
{
     zebraR = param[0].asInt();
     zebraG = param[1].asInt();
     zebraB = param[2].asInt();
}

BLYNK_WRITE(V17)
{
      RGB.brightness(param.asInt()); // assigning incoming value from pin V1 to a variable   
}


BLYNK_WRITE(V14)
{terminal.flush();}


void setup() { //This is where all Arduinos store the on-bootup code
  Serial.begin();
    iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
  RGB.control(true); //Turn off Photon's pulsing blue LED

//	dht.begin();
  
  delay(5000); //Required to stabilize wifi
  
  Blynk.begin(auth, IPAddress(192,168,50,197), 8080);
  Time.zone(-4);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
                terminal.clear();
				terminal.println("-----------------------------");
				terminal.println("STARTING BEDROOM BLYNK SERVER");
				terminal.println(Time.timeStr()); //print current time to Blynk terminal
				terminal.print("Default brightness: ");
				terminal.println(RGB.brightness());
				RGB.brightness(255);
				terminal.print("Adjusted brightness: ");
				terminal.println(RGB.brightness());
                terminal.println("-----------------------------");
				terminal.flush();
checkIaqSensorStatus();
    bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };
  iaqSensor.setTemperatureOffset(1.0);
  checkIaqSensorStatus();
  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();
   loadState();
  iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
  
  /*bme.begin(0x76);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                Adafruit_BME280::SAMPLING_X1, // temperature
                Adafruit_BME280::SAMPLING_X1, // pressure
                Adafruit_BME280::SAMPLING_X1, // humidity
                Adafruit_BME280::FILTER_OFF   );*/
                

}

void loop() { //This is where all Arduinos store their "do this all the time" code
    pms7003.Read();

    Blynk.run();
     
unsigned long time_trigger = millis();
                    if (iaqSensor.run()) { // If new data is available
                
                output = String(time_trigger);
                output += ", " + String(iaqSensor.iaq);
                output += ", " + String(iaqSensor.iaqAccuracy);
                output += ", " + String(iaqSensor.staticIaq);
                output += ", " + String(iaqSensor.co2Equivalent);
                output += ", " + String(iaqSensor.breathVocEquivalent);
                output += ", " + String(iaqSensor.rawTemperature);
                output += ", " + String(iaqSensor.pressure);
                output += ", " + String(iaqSensor.rawHumidity);
                output += ", " + String(iaqSensor.gasResistance);
                output += ", " + String(iaqSensor.stabStatus);
                output += ", " + String(iaqSensor.runInStatus);
                output += ", " + String(iaqSensor.temperature);
                output += ", " + String(iaqSensor.humidity);
                output += ", " + String(iaqSensor.gasPercentage);
                updateState();
                
            } else {
                checkIaqSensorStatus();
            }

    

    if (menuValue == 1) {RGB.color(0, 0, 0);}
    if (menuValue == 2) 
        {
        if (Time.hour() > 7)
            {
                pmG = 55 - sliderValue;
                if (pmG < 0) {pmG = 0;}
                pmG *= (255.0/55.0);
                if (pmG > 255) {pmG = 255;}
                
                pmR = sliderValue;
                if (pmR < 0) {pmR = 0;}
                pmR *= (255.0/55.0);
                if (pmR > 255) {pmR = 255;}
                
                pmB = sliderValue - 100;
                if (pmB < 0) {pmB = 0;}
                pmB *= (255.0/55.0);
                if (pmB > 255) {pmB = 255;}
                
                if (rgbON == true) {RGB.color(pmR, pmG, pmB);}
                else {RGB.color(0, 0, 0);}
                if (sliderValue > 55)
                    {
                        if (millis() - rgbmillis >= 500)
                        {
                            if (rgbON) {rgbON = false;}
                            else {rgbON = true;}
                            rgbmillis = millis();
                        }
                    }
                else rgbON = true;
            }
            else {RGB.color(0, 0, 0);}
        }
    if (menuValue == 3) {RGB.color(zebraR, zebraG, zebraB);}
  

  
    if ( (millis() - millisBlynk >= 30000) || (firstvalue == 1) ) //if it's been 30 seconds OR we just booted up, skip the 30 second wait
    {
        //me.takeForcedMeasurement();
        tempBME = iaqSensor.temperature;
        humBME = iaqSensor.humidity;
        gasBME = (1 / (iaqSensor.gasResistance / 1000.0)) * 10;
        presBME = (iaqSensor.pressure / 100.0);
        bmeiaq = iaqSensor.iaq;
        bmeiaqAccuracy = iaqSensor.iaqAccuracy;
        bmestaticIaq = iaqSensor.staticIaq;
        bmeco2Equivalent = iaqSensor.co2Equivalent;
        bmebreathVocEquivalent = iaqSensor.breathVocEquivalent;
        bmestabStatus = iaqSensor.stabStatus;
        bmerunInStatus = iaqSensor.runInStatus;
        bmegasPercentage = iaqSensor.gasPercentage;
        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME + 243.5))) * humBME * 2.1674)/(273.15 + tempBME);
        dewpoint = tempBME - ((100 - humBME)/5); //calculate dewpoint
        humidex = tempBME + 0.5555 * (6.11 * pow(2.71828, 5417.7530*( (1/273.16) - (1/(273.15 + dewpoint)) ) ) - 10); //calculate humidex using Environment Canada formula
        millisBlynk = millis();
        
        Blynk.virtualWrite(V1, presBME);
        Blynk.virtualWrite(V2, tempBME);
        Blynk.virtualWrite(V3, humBME);
        Blynk.virtualWrite(V4, abshumBME);
        
        new1p0 = pms7003.GetData(pms7003.pm1_0);
        new2p5 = pms7003.GetData(pms7003.pm2_5);
        new10 = pms7003.GetData(pms7003.pm10);
        if (firstvalue == 0)
        {
            if (new1p0 > 200) {new1p0 = old1p0;}
            if (new2p5 > 200) {new2p5 = old2p5;}
            if (new10 > 200) {new10 = old10;}
            if (new1p0 - old1p0 > 50) {new1p0 = old1p0;}
            if (new2p5 - old2p5 > 50) {new2p5 = old2p5;}
            if (new10 - old10 > 50) {new10 = old10;}
        }
        sliderValue = new2p5; //set RGB value to pm25 value
        Blynk.virtualWrite(V5, new1p0);
        Blynk.virtualWrite(V6, new2p5);
        Blynk.virtualWrite(V7, new10);
        Blynk.virtualWrite(V8, pms7003.GetData(pms7003.count0_3um));
        Blynk.virtualWrite(V9, pms7003.GetData(pms7003.count0_5um));
        Blynk.virtualWrite(V10, pms7003.GetData(pms7003.count1um));
        Blynk.virtualWrite(V11, pms7003.GetData(pms7003.count2_5um));
        Blynk.virtualWrite(V12, pms7003.GetData(pms7003.count5um));
        Blynk.virtualWrite(V13, pms7003.GetData(pms7003.count10um));
        //14-16 reserved
        Blynk.virtualWrite(V18, pmR); //debug for pm25 RGB LED
        Blynk.virtualWrite(V19, pmG);
        Blynk.virtualWrite(V20, pmB);

        
        Blynk.virtualWrite(V23, bmeiaq);
        Blynk.virtualWrite(V24, bmeiaqAccuracy);
        Blynk.virtualWrite(V25, bmestaticIaq);
        Blynk.virtualWrite(V26, bmeco2Equivalent);
        Blynk.virtualWrite(V27, bmebreathVocEquivalent);
        Blynk.virtualWrite(V28, bmestabStatus);
        Blynk.virtualWrite(V29, bmerunInStatus);
        Blynk.virtualWrite(V30, bmegasPercentage);
        Blynk.virtualWrite(V31, humidex);
        Blynk.virtualWrite(V32, dewpoint);
        Blynk.virtualWrite(V33, gasBME);
  
        /*terminal.print("Last update: ");
        terminal.print(Time.timeStr()); //print current time to Blynk terminal
        terminal.println("");
        terminal.flush();*/
        old1p0 = new1p0;
        old2p5 = new2p5;
        old10 = new10;
        firstvalue = 0;
    }

}

void checkIaqSensorStatus(void)
{
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      terminal.println(output);
      //for (;;)
        //errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      terminal.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      terminal.println(output);
      //for (;;)
        //errLeds(); /* Halt in case of failure */
    } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      terminal.println(output);
    }
  }
}

void errLeds(void)
{

}

void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    terminal.println("**Reading state from EEPROM");
            terminal.println(Time.timeStr()); //print current time to Blynk terminal
        
       
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      terminal.print(bsecState[i], HEX);

    }
    terminal.println("**");
    terminal.flush();
    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    terminal.println("**Erasing EEPROM");
        terminal.print(Time.timeStr()); //print current time to Blynk terminal
        terminal.println("**");
        terminal.flush();
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    //EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;
  if (stateUpdateCounter == 0) {
    /* First state update when IAQ accuracy is >= 3 */
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD minutes */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    terminal.println("**Writing state to EEPROM");
        terminal.println(Time.timeStr()); //print current time to Blynk terminal
        
        
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      terminal.print(bsecState[i], HEX);
      
    }
    terminal.println("**");
    terminal.flush();
    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    //EEPROM.commit();
  }
}
