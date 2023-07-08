// This #include statement was automatically added by the Particle IDE.
//#include <Adafruit_DHT_Particle.h>


//#define DHTPIN D2     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11		// DHT 11 
//#define DHTTYPE DHT22		// DHT 22 (AM2302)
//#define DHTTYPE DHT21		// DHT 21 (AM2301)




// This #include statement was automatically added by the Particle IDE.
#include <PMS7003-Particle-Sensor-Serial.h>




#include "blynk.h"
#include "math.h"

#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"






#define BME_SCK D4
#define BME_MISO D3
//#define BME_MOSI D2
#define BME_CS D5
#define SEALEVELPRESSURE_HPA (1013.25)

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

Adafruit_BME280 bme; // I2C

float abshumBME, tempBME, presBME, humBME;

float amtemp, amhum;

int timer1 = TTU; // How often to send sensor updates to Ubidots
int screentime = (TTS / 1000);
int debugcounter = 105;
int firstvalue = 1;

int history = 2;
int historycount = history;

float tempmax = -100000;
float tempmin = 100000;

float sensor;



int last_pushed = 0; // global variable

int ranged = 0;

float deltapres, deltatemp, deltahum = 0;
float old1p0, old2p5, old10, new1p0, new2p5, new10;
float tempDS18;

unsigned long lastmillis = 0;
unsigned long millisBlynk = 0;
unsigned long deltamillis = 0;
unsigned long pagemillis = 0;
int pagetimer = 0;

int xPos = 0;







PMS7003Serial<USARTSerial> pms7003(Serial1, D6);



                            // counter




void setup() { //This is where all Arduinos store the on-bootup code
  Serial.begin();

  RGB.control(true); //Turn off Photon's pulsing blue LED
  
//	dht.begin();
  
  delay(5000); //Required to stabilize wifi
				terminal.println("-----------------------------");
				terminal.println("STARTING BEDROOM BLYNK SERVER");
				terminal.println(Time.timeStr()); //print current time to Blynk terminal
                terminal.println("-----------------------------");
				terminal.flush();
  Blynk.begin(auth, IPAddress(192,168,50,197), 8080);
  Time.zone(-5);
  bme.begin(0x76);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                Adafruit_BME280::SAMPLING_X1, // temperature
                Adafruit_BME280::SAMPLING_X1, // pressure
                Adafruit_BME280::SAMPLING_X1, // humidity
                Adafruit_BME280::FILTER_OFF   );
                
  pagemillis = millis();
}

void loop() { //This is where all Arduinos store their "do this all the time" code



pms7003.Read();

  Blynk.run();
  if (millis() - millisBlynk >= 2000)
  {

    //amtemp = dht.getTempCelcius();
    //amhum = dht.getHumidity();
    bme.takeForcedMeasurement();
  //  am2315.readTemperatureAndHumidity(amtemp, amhum);
    tempBME = (bme.readTemperature() + tempoffset);
    presBME = (bme.readPressure() / 100.0F);
    humBME = bme.readHumidity();
    abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME+243.5))) * humBME * 2.1674)/(273.15 + tempBME);
    millisBlynk = millis();
   // DS18sensor.read();
    timerBlynk -= 2;
      if (timerBlynk <= 0)
      {
       // amhum = dht.getHumidity();
      //  amtemp = dht.getTempCelcius();
        timerBlynk = TTU;
        //timerPhant -= 1;
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

        		Blynk.virtualWrite(V5, new1p0);
				Blynk.virtualWrite(V6, new2p5);
				Blynk.virtualWrite(V7, new10);
				Blynk.virtualWrite(V8, pms7003.GetData(pms7003.count0_3um));
				Blynk.virtualWrite(V9, pms7003.GetData(pms7003.count0_5um));
				Blynk.virtualWrite(V10, pms7003.GetData(pms7003.count1um));
				Blynk.virtualWrite(V11, pms7003.GetData(pms7003.count2_5um));
				Blynk.virtualWrite(V12, pms7003.GetData(pms7003.count5um));
				Blynk.virtualWrite(V13, pms7003.GetData(pms7003.count10um));
				terminal.print("Last update: ");
				terminal.print(Time.timeStr()); //print current time to Blynk terminal
				terminal.println("");
				terminal.flush();
				old1p0 = new1p0;
				old2p5 = new2p5;
				old10 = new10;
				firstvalue = 0;

      
      }
  }






}
