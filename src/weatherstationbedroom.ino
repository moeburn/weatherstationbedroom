// This #include statement was automatically added by the Particle IDE.
//#include <Adafruit_DHT_Particle.h>


//#define DHTPIN D2     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11		// DHT 11 
//#define DHTTYPE DHT22		// DHT 22 (AM2302)
//#define DHTTYPE DHT21		// DHT 21 (AM2301)




// This #include statement was automatically added by the Particle IDE.
#include <PMS7003-Particle-Sensor-Serial.h>
#include "NtpTime.h"
NtpTime ntptime;

#include "neopixel.h"


// IMPORTANT: Set pixel COUNT, PIN and TYPE
#if (PLATFORM_ID == 32)
// MOSI pin MO
#define PIXEL_PIN SPI
// MOSI pin D2
// #define PIXEL_PIN SPI1
#else // #if (PLATFORM_ID == 32)
#define PIXEL_PIN A4
#endif
#define PIXEL_COUNT 3
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

// Prototypes for local build, ok to leave in for Build IDE
void rainbow(uint8_t wait);
uint32_t Wheel(byte WheelPos);


#include <bsec2.h>

#include "blynk.h"
#include "math.h"

//#include "Adafruit_Sensor.h"
//#include "Adafruit_BME280.h"
#include "config/bme680_iaq_33v_3s_28d/bsec_iaq.h"
#include <Average.h>
#include "rgb-controls.h"
using namespace RGBControls;
//int LEDApin3 = A4; //for SMD RGB LED
//int LEDApin1 = A5;
//int LEDApin2 = A7;



int LEDApin3 = A0; //A4
int LEDApin2 = A5;
int LEDApin1 = A7;


//Led PMled2(LEDBpin1, LEDBpin2, LEDBpin3);
Led PMled1(LEDApin1, LEDApin2, LEDApin3);
Color red1(255, 0, 0);
Color green1(0, 255, 0);
Color blue1(0, 0, 255);
Color colors[3] = { red1, green1, blue1 };


Average<float> pm1Avg(30);
Average<float> pm25Avg(30);
Average<float> pm10Avg(30);
Average<float> wifiAvg(30);
Average<float> a0Avg(30);


//const uint8_t bsec_config_iaq[] = {2,9,4,1,61,0,0,0,0,0,0,0,182,1,0,0,52,0,1,0,0,192,168,71,64,49,119,76,0,0,97,69,0,0,97,69,137,65,0,191,205,204,204,190,0,0,64,191,225,122,148,190,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,1,35,41,29,86,88,0,9,0,229,208,34,62,0,0,0,0,0,0,0,0,218,27,156,62,225,11,67,64,0,0,160,64,0,0,0,0,0,0,0,0,94,75,72,189,93,254,159,64,66,62,160,191,0,0,0,0,0,0,0,0,33,31,180,190,138,176,97,64,65,241,99,190,0,0,0,0,0,0,0,0,167,121,71,61,165,189,41,192,184,30,189,64,12,0,10,0,0,0,0,0,0,0,0,0,229,0,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,44,1,0,0,0,0,159,253,0,0};
#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000)
#define USE_EEPROM


/**
 * @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
 * @param[in] bsec  : Bsec2 class object
 */
void checkBsecStatus(Bsec2 bsec);

/**
 * @brief : This function updates/saves BSEC state
 * @param[in] bsec  : Bsec2 class object
 */
void updateBsecState(Bsec2 bsec);

/**
 * @brief : This function is called by the BSEC library when a new output is available
 * @param[in] input     : BME68X sensor data before processing
 * @param[in] outputs   : Processed BSEC BSEC output data
 * @param[in] bsec      : Instance of BSEC2 calling the callback
 */
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

/**
 * @brief : This function retrieves the existing state
 * @param : Bsec2 class object
 */
bool loadState(Bsec2 bsec);

/**
 * @brief : This function writes the state into EEPROM
 * @param : Bsec2 class object
 */
bool saveState(Bsec2 bsec);
void errLeds(void);
//void updateState(void);
//void loadState(void);
Bsec2 iaqSensor;

static uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
uint16_t stateUpdateCounter = 0;
String output;




#define tempoffset -0.9F




#define TTS 1000 //Time till screen update in ms
#define TTU 20 //Time till update in TTS intervals

//DHT dht(DHTPIN, DHTTYPE);
//int loopCount;
WidgetTerminal terminal(V14); //terminal widget
WidgetBridge bridge1(V70);

char auth[] = "xxxxxxxxxxxxxxxxxxxxxx; //BLYNK
char remoteAuth[] = "xxxxxxxxxxxxxxxxxxxx";

BLYNK_CONNECTED() {
  bridge1.setAuthToken (remoteAuth);
}

int timerBlynk = TTU;

bool drawChart = FALSE;

//char auth[] = "81f749960ec34003bbab37a5b4ffa61e"; //BLYNK


//Adafruit_BME280 bme; // I2C

float abshumBME, tempBME, presBME, humBME, gasBME, dewpoint, humidex;
float bmeiaq, bmeiaqAccuracy, bmestaticIaq, bmeco2Equivalent, bmebreathVocEquivalent, bmestabStatus, bmerunInStatus, bmegasPercentage;


float amtemp, amhum;

int timer1 = TTU; // How often to send sensor updates to Ubidots
int screentime = (TTS / 1000);
int debugcounter = 105;
int firstvalue = 1;
int blynkWait = 30000;
int history = 2;
int historycount = history;



float sensor;
float bridgedata, bridgetemp, bridgehum;


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
float  pmR2, pmG2, pmB2;
float  pmR3, pmG3, pmB3;
bool rgbON = true;
float LEDbrightness = 6; 
float RGBbrightness = 6; 
int sliderboostbrightness = 0; 
int redboost = 0;
int sliderLEDbrightness = 38; 
int sliderRGBbrightness = 18;

    int leddelay = 1000;

bool rapidfire = false;


PMS7003Serial<USARTSerial> pms7003(Serial1, D6);

BLYNK_WRITE(V61){
    float pinData = param.asFloat();
    bridgedata = pinData;
}

BLYNK_WRITE(V62){
    bridgetemp = param.asFloat();

}

BLYNK_WRITE(V63){
    bridgehum = param.asFloat();

}




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
    sliderRGBbrightness = param.asInt();

}

/*BLYNK_WRITE(V34)
{
    sliderboostbrightness = param.asInt();
}

BLYNK_WRITE(V35)
{
    redboost = param.asInt();
}*/


BLYNK_WRITE(V21)
{
    sliderLEDbrightness = param.asInt();

}


BLYNK_WRITE(V14)
{
    if (String("help") == param.asStr()) 
    {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("temps");
    terminal.println("wets");
    terminal.println("particles");
    terminal.println("bsec");
    terminal.println("erase");
    terminal.println("a1");
    terminal.println("a2");
    terminal.println("a3");
    terminal.println("ledcycle");
    terminal.println("rapidon");
    terminal.println("rapidoff");
    terminal.println("redboost+");
    terminal.println("redboost-");
    terminal.println("connect");
    terminal.println("disconnect");
     terminal.println("==End of list.==");
    }
        if (String("wifi") == param.asStr()) 
    {
        terminal.print("Connected to: ");
        terminal.println(WiFi.SSID());
        terminal.print("IP address:");
        terminal.println(WiFi.localIP());
        terminal.print("Signal strength: ");
        terminal.println(WiFi.RSSI());
        terminal.println(Time.timeStr());
    }
if (String("erase") == param.asStr()) 
        {
        /* Erase the EEPROM with zeroes */
        terminal.println("Erasing EEPROM");

        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
            EEPROM.write(i, 0);
        terminal.flush();
        //EEPROM.commit();
    }


    if (String("temps") == param.asStr()) {
        terminal.print("tempBME[v0],tempPool[v5],humidex[v17],dewpoint[v2]: ");
        terminal.print(tempBME);
        terminal.print(",,,");
        terminal.print(humidex);
        terminal.print(",,,");
        terminal.println(dewpoint);
    }
    if (String("wets") == param.asStr()) {
        terminal.print("humBME[v3],abshumBME[v4],presBME[v1],gasBME[v7]: ");
        terminal.print(humBME);
        terminal.print(",,,");
        terminal.print(abshumBME);
        terminal.print(",,,");
        terminal.print(presBME);
        terminal.print(",,,");
        terminal.println(gasBME);
    }
    if (String("particles") == param.asStr()) {
         if (!pms7003.Read()) {terminal.println("Can't find PMS sensor");}
        terminal.print("pm1[v8],pm2.5[v9],pm10[v10],0.3um[v11],0.5um[v12],1um[v13],2.5um[v14],5um[v15],10um[v16]");
        terminal.print(pms7003.GetData(pms7003.pm1_0));
        terminal.print(",,,");
        terminal.print(pms7003.GetData(pms7003.pm2_5));
        terminal.print(",,,");
        terminal.print(pms7003.GetData(pms7003.pm10));
        terminal.print(",,,");
        terminal.print(pms7003.GetData(pms7003.count0_3um));
        terminal.print(",,,");
        terminal.print(pms7003.GetData(pms7003.count0_5um));
        terminal.print(",,,");
        terminal.print(pms7003.GetData(pms7003.count1um));
        terminal.print(",,,");
        terminal.print(pms7003.GetData(pms7003.count2_5um));
        terminal.print(",,,");
        terminal.print(pms7003.GetData(pms7003.count5um));
        terminal.print(",,,");
        terminal.println(pms7003.GetData(pms7003.count10um));
    }
    if (String("bsec") == param.asStr()) {
        terminal.print("bmeiaq[v23],bmeiaqAccuracy[v24],bmestaticIaq[v25],bmeco2Equivalent[v26],bmebreathVocEquivalent[v27],bmestabStatus[v28],bmerunInStatus[v29],bmegasPercentage[v30]:");
        terminal.print(bmeiaq);
        terminal.print(",,,");
        terminal.print(bmeiaqAccuracy);
        terminal.print(",,,");
        terminal.print(bmestaticIaq);
        terminal.print(",,,");
        terminal.print(bmeco2Equivalent);
        terminal.print(",,,");
        terminal.print(bmebreathVocEquivalent);
        terminal.print(",,,");
        terminal.print(bmestabStatus);
        terminal.print(",,,");
        terminal.print(bmerunInStatus);
        terminal.print(",,,");
        terminal.println(bmegasPercentage);
    }
    if (String("a1") == param.asStr()) {
        analogWrite(LEDApin1, 255);
        delay(leddelay);
        analogWrite(LEDApin1, 0);
        terminal.println("a1");
    }
        if (String("a2") == param.asStr()) {
        analogWrite(LEDApin2, 255);
        delay(leddelay);
        analogWrite(LEDApin2, 0);
        terminal.println("a2");
    }
        if (String("a3") == param.asStr()) {
        analogWrite(LEDApin3, 255);
        delay(leddelay);
        analogWrite(LEDApin3, 0);
        terminal.println("a3");
    }
    if (String("rapidon") == param.asStr()) {
        blynkWait = 50;
    }
    if (String("ledcycle") == param.asStr()) {
        ledcycle();
        terminal.println();
        terminal.print("Max frequency: ");
        terminal.print(analogWriteMaxFrequency(LEDApin1));
        terminal.println("hz.");
    }

    if (String("connect") == param.asStr()) {
        Particle.connect();
        terminal.print("> Connecting to Particle cloud...");
        while (!Particle.connected()) {
            terminal.print(".");
            terminal.flush();
            delay(250);
            }
        terminal.println("connected.");
    }
    if (String("disconnect") == param.asStr()) {
        Particle.disconnect();
        terminal.println("> Disconnecting from Particle cloud.");
    }

    

    if (String("rapidoff") == param.asStr()) {
      blynkWait = 30000;
    }

    if (String("redboost+") == param.asStr()) {
      redboost++;
      terminal.print("> redboost is now [");
      terminal.print(redboost);
      terminal.println("]");
    }
    if (String("redboost-") == param.asStr()) {
      redboost--;
      terminal.print("> redboost is now [");
      terminal.print(redboost);
      terminal.println("]");
    }
    
    terminal.flush();

}

BLYNK_WRITE(V40)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  
  if (pinValue == 1){
        Particle.connect();
        terminal.print("> Connecting to Particle cloud...");
        while (!Particle.connected()) {
            terminal.print(".");
            terminal.flush();
            delay(250);
            }
        terminal.println("connected.");
  } else if (pinValue == 0) {
   // do something when button is released;
  }

}

void ledcycle(void) {
analogWrite(LEDApin1, 127, 10);
analogWrite(LEDApin2, 127, 10);
analogWrite(LEDApin3, 127, 10);
delay(leddelay);
analogWrite(LEDApin1, 0);
analogWrite(LEDApin2, 0);
analogWrite(LEDApin3, 0);
delay(leddelay);
analogWrite(LEDApin1, 127, (analogWriteMaxFrequency(LEDApin1)/500));
analogWrite(LEDApin2, 127, (analogWriteMaxFrequency(LEDApin2)/500));
analogWrite(LEDApin3, 127, (analogWriteMaxFrequency(LEDApin3)/500));
delay(leddelay);
analogWrite(LEDApin1, 0);
analogWrite(LEDApin2, 0);
analogWrite(LEDApin3, 0);
}



SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);


void setup() { //This is where all Arduinos store the on-bootup code
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.setPixelColor(2, strip.Color(100, 0, 0)); 
   strip.show();
          //  Set pixel's color (in RAM)
    delay(250);
    strip.setPixelColor(2, strip.Color(0, 0, 0));
    strip.setPixelColor(1, strip.Color(100, 100, 0));
  strip.show();
 WiFi.on();
  WiFi.connect() ;
  while (WiFi.connecting()){}

  Particle.disconnect();
  Serial.begin();
  Wire.begin();
    iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
  RGB.control(true); //Turn off Photon's pulsing blue LED
ntptime.start();
//	dht.begin();
  
   //Required to stabilize wifi
   // pinMode(LEDApin1, OUTPUT);
   // pinMode(LEDApin2, OUTPUT);
   // pinMode(LEDApin3, OUTPUT);



  Blynk.begin(auth, IPAddress(xxx,xxx,xxx,xxx), 8080);
  Time.zone(-4);
  strip.setPixelColor(1, strip.Color(0, 0, 0));
  strip.setPixelColor(0, strip.Color(0, 100, 0)); 
strip.show();
  delay(2000);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
                //terminal.clear();
				terminal.println("----------------------------------");
				terminal.println("STARTING BEDROOM BLYNK SERVER v1.6");
                terminal.println(output);
				terminal.println(Time.timeStr()); //print current time to Blynk terminal
                terminal.println("----------------------------------");
                terminal.println("Type 'connect' to flash firmware");
                terminal.println("Type 'help' for a list of commands");
				terminal.flush();

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
  
  
  iaqSensor.setConfig(bsec_config_iaq);
  
  iaqSensor.setTemperatureOffset(1.0);
   loadState(iaqSensor);
  iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
  
  iaqSensor.attachCallback(newDataCallback);
  /*bme.begin(0x76);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                Adafruit_BME280::SAMPLING_X1, // temperature
                Adafruit_BME280::SAMPLING_X1, // pressure
                Adafruit_BME280::SAMPLING_X1, // humidity
                Adafruit_BME280::FILTER_OFF   );*/
  strip.setPixelColor(0, strip.Color(0, 0, 0)); 
  strip.setPixelColor(1, strip.Color(0, 0, 0)); 
  strip.setPixelColor(2, strip.Color(0, 0, 0));         //  Set pixel's color (in RAM)
  strip.show();    

}

unsigned long last = 0;
unsigned long last_pm_reading = 0;
unsigned long samplemillis = 0;

void loop() { //This is where all Arduinos store their "do this all the time" code
unsigned long now = millis();
    iaqSensor.run(); 
      if (pms7003.Read()) {
    last_pm_reading = now;
  }
  
    Blynk.run();
       
  
    /*if (rapidfire) 
    {
        Blynk.virtualWrite(V69, analogRead(A0));
    }*/

if(!WiFi.ready())
{WiFi.connect();}

        

    if (menuValue == 1) {RGB.color(0, 0, 0);}
    if (menuValue == 2) 
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

                pmG2 = 55 - bridgedata;
                if (pmG2 < 0) {pmG2 = 0;}
                pmG2 *= (255.0/55.0);
                if (pmG2 > 255) {pmG2 = 255;}
                
                
                pmR2 = bridgedata;
                if (pmR2 < 0) {pmR2 = 0;}
                pmR2 *= (255.0/55.0);
                if (pmR2 > 255) {pmR2 = 255;}
                
                
                pmB2 = bridgedata - 100;
                if (pmB2 < 0) {pmB2 = 0;}
                pmB2 *= (255.0/55.0);
                if (pmB2 > 255) {pmB2 = 255;}

                

                pmG3 = 155 - bmeiaq;
                if (pmG3 < 0) {pmG3 = 0;}
                pmG3 *= (255.0/155.0);
                if (pmG3 > 255) {pmG3 = 255;}
                
                
                pmR3 = bmeiaq;
                if (pmR3 < 0) {pmR3 = 0;}
                pmR3 *= (255.0/155.0);
                if (pmR3 > 255) {pmR3 = 255;}
                
                
                pmB3 = bmeiaq - 155;
                if (pmB3 < 0) {pmB3 = 0;}
                pmB3 *= (255.0/155.0);
                if (pmB3 > 255) {pmB3 = 255;}

                //Color PMled1color(pmR2, pmG2, pmB2);
                RGB.color(0, 0, 0);

                //PMled1.setColor(PMled1color.withBrightness(LEDbrightness));
                strip.setBrightness(LEDbrightness);
                strip.setPixelColor(2, strip.Color(pmR3, pmG3, pmB3)); 
                  strip.setPixelColor(0, strip.Color(pmR, pmG, pmB)); 
                strip.setPixelColor(1, strip.Color(pmR2, pmG2, pmB2)); 

                //strip.setColorDimmed(1, pmR, pmG, pmB, RGBbrightness);
                //strip.setColorDimmed(2, pmR2, pmG2, pmB2, RGBbrightness);
                strip.show();  
                //PMled2.setColor(PMled2color.withBrightness(LEDbrightness));

                
                //if (rgbON == true) {RGB.color(pmR, pmG, pmB);}
                //else {RGB.color(0, 0, 0);}
                /*if (sliderValue > 55)
                    {
                        if (millis() - rgbmillis >= 500)
                        {
                            if (rgbON) {rgbON = false;}
                            else {rgbON = true;}
                            rgbmillis = millis();
                        }
                    }
                else rgbON = true;*/

        }
    if (menuValue == 3) {
        RGB.color(zebraR, zebraG, zebraB);
        //Color PMled2color(zebraR, (zebraG/3), zebraB);
        //PMled2.setColor(PMled2color);
        //PMled1.setColor(PMled2color);
                        strip.setBrightness(sliderLEDbrightness);
                strip.setPixelColor(0, strip.Color(zebraR, zebraG, zebraB)); 
                strip.setPixelColor(1, strip.Color(zebraR, zebraG, zebraB));
                strip.setPixelColor(2, strip.Color(zebraR, zebraG, zebraB));
                strip.show();  
        }

    if  (millis() - samplemillis >= 1000) //if it's been 30 seconds OR we just booted up, skip the 30 second wait
    {
        samplemillis = millis();
        wifiAvg.push(WiFi.RSSI());
        a0Avg.push(analogRead(A0));
          if (Time.hour() < 7)
            {
                RGB.brightness(0);
                LEDbrightness = 0;
            }
        else
        {
            RGBbrightness = (((analogRead(A0) * 0.0005) * sliderRGBbrightness) + 1);
            if (RGBbrightness > 255) {RGBbrightness = 255;}
            LEDbrightness = ((analogRead(A0) * 0.0005) * sliderLEDbrightness) + sliderboostbrightness;
            if (LEDbrightness > 255) {LEDbrightness = 255;}
            if (LEDbrightness < 1) {LEDbrightness = 1;}
            RGB.brightness(RGBbrightness);
        }
    }
    
    if  (millis() - millisBlynk >= blynkWait) //if it's been blynkWait seconds 
    {
        pms7003.Read();
        //me.takeForcedMeasurement();
        millisBlynk = millis();
        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME + 243.5))) * humBME * 2.1674)/(273.15 + tempBME);
        dewpoint = tempBME - ((100 - humBME)/5); //calculate dewpoint
        humidex = tempBME + 0.5555 * (6.11 * pow(2.71828, 5417.7530*( (1/273.16) - (1/(273.15 + dewpoint)) ) ) - 10); //calculate humidex using Environment Canada formula
        
        if ((tempBME == 0) && (humBME == 0)) {}
        else {
        Blynk.virtualWrite(V1, presBME);
        Blynk.virtualWrite(V2, tempBME);
        Blynk.virtualWrite(V3, humBME);
        Blynk.virtualWrite(V4, abshumBME);
        Blynk.virtualWrite(V31, humidex);
        Blynk.virtualWrite(V32, dewpoint);
        }
        sliderValue = new2p5; //set RGB value to pm25 value*/
        Blynk.virtualWrite(V5, pm1Avg.mean());
        Blynk.virtualWrite(V6, pm25Avg.mean());
        bridge1.virtualWrite(V71, pm25Avg.mean());
        bridge1.virtualWrite(V72, bridgedata);
        bridge1.virtualWrite(V73, bridgetemp);
        bridge1.virtualWrite(V74, bridgehum);
        Blynk.virtualWrite(V7, pm10Avg.mean());
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

        Blynk.virtualWrite(V33, gasBME);
        Blynk.virtualWrite(V34, wifiAvg.mean());
        Blynk.virtualWrite(V69, a0Avg.mean());
        /*terminal.print("Last update: ");
        terminal.print(Time.timeStr()); //print current time to Blynk terminal
        terminal.println("");
        terminal.flush();*/
        old1p0 = new1p0;
        old2p5 = new2p5;
        old10 = new10;
        firstvalue = 0;
    }
 if ((now - last) > 2300) {
    // Let us be generous. Active state the device
    // reports at least every 2.3 seconds.
    if ((now - last_pm_reading) > 2300) {
      //Serial.println("No reading for at least 10 seconds!");
    } else {
        new1p0 = pms7003.GetData(pms7003.pm1_0);
        new2p5 = pms7003.GetData(pms7003.pm2_5);
        new10 = pms7003.GetData(pms7003.pm10);
        if (firstvalue == 0)  //do not do this on the first run
        {
            if (new1p0 > 200) {new1p0 = old1p0;} //check for data spikes in particle counter, ignore data that is >200
            if (new2p5 > 200) {new2p5 = old2p5;} //data spikes ruin pretty graph
            if (new10 > 200) {new10 = old10;}
            if (new1p0 - old1p0 > 50) {new1p0 = old1p0;} //also ignore data that is >50 off from last data
            if (new2p5 - old2p5 > 50) {new2p5 = old2p5;}
            if (new10 - old10 > 50) {new10 = old10;}
        }
        pm1Avg.push(new1p0);
        pm25Avg.push(new2p5);
        pm10Avg.push(new10);
        
        old1p0 = new1p0; //reset data spike check variable
        old2p5 = new2p5;
        old10 = new10;
        firstvalue = 0;
    }
    last = now;
  }
}



void errLeds(void)
{

}

/*void loadState(void)
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
}*/

bool loadState(Bsec2 bsec)
{
#ifdef USE_EEPROM
    

    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
    {
        /* Existing state in EEPROM */
        terminal.println("Reading state from EEPROM");
        terminal.print("State file: ");
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        {
            bsecState[i] = EEPROM.read(i + 1);
            terminal.print(String(bsecState[i], HEX) + "");
        }
        terminal.println();
        terminal.flush();

        if (!bsec.setState(bsecState))
            return false;
    } else
    {
        /* Erase the EEPROM with zeroes */
        terminal.println("Erasing EEPROM");

        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
            EEPROM.write(i, 0);
        terminal.flush();
        //EEPROM.commit();
    }
#endif
    return true;
}



void updateBsecState(Bsec2 bsec)
{
    static uint16_t stateUpdateCounter = 0;
    bool update = false;

    if (!stateUpdateCounter || (stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
        /* Update every STATE_SAVE_PERIOD minutes */
        update = true;
        stateUpdateCounter++;
    }

    if (update && !saveState(bsec))
        checkBsecStatus(bsec);
}





bool saveState(Bsec2 bsec)
{

    if (!bsec.getState(bsecState))
        return false;

    terminal.println("**Writing state to EEPROM");
        terminal.println(Time.timeStr()); //print current time to Blynk terminal

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
        EEPROM.write(i + 1, bsecState[i]);
        terminal.print(bsecState[i], HEX);
    }
    terminal.println("**");
    terminal.flush();
    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);


    return true;
}

void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK)
    {
        terminal.println("BSEC error code : " + String(bsec.status));
        terminal.flush();
        //errLeds(); /* Halt in case of failure */
    } else if (bsec.status > BSEC_OK)
    {
        terminal.println("BSEC warning code : " + String(bsec.status));
        terminal.flush();
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        terminal.println("BME68X error code : " + String(bsec.sensor.status));
        terminal.flush();
        //errLeds(); /* Halt in case of failure */
    } else if (bsec.sensor.status > BME68X_OK)
    {
        terminal.println("BME68X warning code : " + String(bsec.sensor.status));
        terminal.flush();
    }
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
        return;

    Serial.println("BSEC outputs:\n\ttimestamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output  = outputs.output[i];
        switch (output.sensor_id)
        {
        



            case BSEC_OUTPUT_RAW_GAS:
                gasBME = (1 / (output.signal / 1000.0)) * 10;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                tempBME = output.signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                humBME = output.signal;
                break;
            case BSEC_OUTPUT_IAQ:
            bmeiaq = output.signal;
            bmeiaqAccuracy = output.accuracy;
                break;
            case BSEC_OUTPUT_STATIC_IAQ:
            bmestaticIaq = output.signal;
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
            bmeco2Equivalent = output.signal;
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            bmebreathVocEquivalent = output.signal;
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
            presBME = (output.signal / 100.0);
                break;
            case BSEC_OUTPUT_STABILIZATION_STATUS:
            bmestabStatus = output.signal;
                break;
            case BSEC_OUTPUT_RUN_IN_STATUS:
            bmerunInStatus = output.signal;
                break;
            case BSEC_OUTPUT_GAS_PERCENTAGE:
            bmegasPercentage = output.signal;
                break;

            default:
                break;
        }
    }

    updateBsecState(iaqSensor);
}
