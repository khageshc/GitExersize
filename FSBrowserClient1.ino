//#include <PingerResponse.h>


/*  Connects to the home WiFi network
    Asks some network parameters
    Sends and receives message from the server in every 2 seconds
    Communicates: wifi_server_01.ino
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <Arduino_JSON.h>
#include <ESP8266HTTPClient.h>
#include <IPAddress.h>
#include <LittleFS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <pins_arduino.h>

// Include WiFiManager Library
#include <WiFiManager.h>
// JSON configuration file
#define JSON_CONFIG_FILE "config.json"

// Flag for saving data
bool shouldSaveConfig = false;

// Variables to hold data from custom textboxes
char testString[50] = "test value";
int testNumber = 1234;


extern "C"
{
#include <lwip/icmp.h> // needed for icmp packet definitions
}
//#define GNERIC_ESP8266

#ifdef GNERIC_ESP8266

#define SDA_PIN 4
#define SCL_PIN 5
#define GPIO_PIN_SET 1

#define  D0  16
#define  D1  5
#define  D2  4
#define  D3  0
#define  D4  2
#define  D5  14
#define  D6  12
#define  D7  13
#define  D8  15
#define  D9  3
#define  D10 1

#else

#define SDA_PIN D2
#define SCL_PIN D1
#define GPIO_PIN_SET 1

#endif
#define DEVELOP	1
#define I2C_CLEAR_SDA digitalWrite(SDA_PIN, 0);
#define I2C_SET_SDA digitalWrite(SDA_PIN, 1);
//#define I2C_READ_SDA {if (digitalRead(SDA_PIN)) == GPIO_PIN_SET) return 1; else return 0; return 0;};
#define I2C_CLEAR_SCL digitalWrite(SCL_PIN, 0);
#define I2C_SET_SCL digitalWrite(SCL_PIN, 1);
#define I2C_DELAY delayMicroseconds(3); // 5 microsecond delay

#define I2C_MASTER  0x42
#define I2C_ADC_SLAVE   0x92
#define I2C_PORT_SLAVE  0x40
#define I2C_SRAM_RW_SLAVE  0xA4
#define I2C_SRAM_CTRL_SLAVE  0x34

#define DBG_OUTPUT_PORT Serial1


//void I2C_bus_init(unsigned char scl_pin, unsigned char sda_pin, unsigned char port);

void I2C_init(void);

void I2C_start_cond(void);

void I2C_stop_cond(void);

void I2C_write_bit(unsigned char b);

unsigned char I2C_read_SDA(void);

// Reading a bit in I2C:
unsigned char I2C_read_bit(void);

unsigned char I2C_write_byte(unsigned char B, unsigned char start, unsigned char stop);

unsigned char I2C_read_byte(unsigned char ack, unsigned char stop);

unsigned char I2C_send_byte(unsigned char address, unsigned char data);

unsigned char I2C_receive_byte(unsigned char address);

unsigned char I2C_send_byte_data(unsigned char address, unsigned char reg, unsigned char data);

unsigned char I2C_receive_byte_data(unsigned char address, unsigned char reg);

unsigned char I2C_transmit(unsigned char address, unsigned char data[], unsigned char size);

unsigned char I2C_receive(unsigned char address, unsigned char reg[], unsigned char *data, unsigned char reg_size, unsigned char size);
unsigned int I2C_receive_adc(unsigned char address, unsigned char *data, unsigned char size);
unsigned char I2C_transmit_read_SRAM(unsigned char address, unsigned char data[], unsigned char size);



void WithoutIp(void);


const char* ssid;
const char* password;
WiFiServer server(80);
WiFiClient client;

unsigned int s16ADCValue = 0;
int DebugMessage;
//const IPAddress remote_ip(192, 168, 1.2)

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
char  replyPacket[255] = "Hi there! Got the message from Air :-)";  // a reply string to send back
char  ManualreplyPacket[255] = "UDP MANUAL DEBUG";  // a reply string to send back


volatile int packetSize = 0;
char TempTubeLight = 0;
char TempPlug = 0;
char TempLight = 0;
char TempLight2 = 0;

char ManualTempTubeLight = 0;
char ManualTempPlug = 0;
char ManualTempLight = 0;
char ManualTempLight2 = 0;

volatile unsigned int FanSpeed = 0;
volatile unsigned int FanSpeedNet = 0;
volatile unsigned int FanSpeedUpdate = 0;
static unsigned char u8TimerLoop = 0;
static unsigned int FanSPeedPrevios = 0;
static unsigned int FanSpeedDiff = 0;


unsigned int FanSpeed_ADC = 0;
unsigned int FanSpeedUdp = 0;
unsigned int PreviosFanSpeed = 0;
char PreviousTubelight = 0;
char PreviousPlug = 0;
char PreviousLight = 0;
char PreviousLight2 = 0;
char u8BUtton1Pressed = 0;
char u8BUtton2Pressed = 0;
char u8BUtton3Pressed = 0;
char u8BUtton4Pressed = 0;

unsigned char data1[2] = {0x00};
unsigned char size1   = 0x01;
unsigned char data[2] = {0x00};
unsigned char IOPortdata = {0x00};


char FanCtrl = 0;
char FanSelectionTimer = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long interval =100;
IPAddress gatewayIP;

unsigned char Memorydata[5] = {0x00, 0x00, 0x02,0x00,0x00};


void IRAM_ATTR Button1DetectionAct (void)
{
	delayMicroseconds(750);
	if((digitalRead(D3) == 0))
	{
		DBG_OUTPUT_PORT.printf("B1=ON\n");
		digitalWrite(D6, 0);
		u8BUtton1Pressed = 0x01;
	}
	else if((digitalRead(D3) == 1))
	{
		DBG_OUTPUT_PORT.printf("B1=OFF\n");
		digitalWrite(D6, 1);
	}
}

void IRAM_ATTR Button2DetectionAct (void)
{
	delayMicroseconds(1000);

	if((digitalRead(D8) == 0))
	{
		DBG_OUTPUT_PORT.printf("B2=ON\n");

		digitalWrite(D1, 0);
		u8BUtton2Pressed = 0x01;
	}
	else if((digitalRead(D8) == 1))
	{

		DBG_OUTPUT_PORT.printf("B2=OFF\n");
		digitalWrite(D1, 1);


	}

}

void IRAM_ATTR Button3DetectionAct (void)
{
	delayMicroseconds(750);
	if((digitalRead(D7) == 0))
	{
		DBG_OUTPUT_PORT.printf("B3=ON\n");
		digitalWrite(D0, 0);
	}
	else if((digitalRead(D7) == 1))
	{
		DBG_OUTPUT_PORT.printf("B3=OFF\n");
		digitalWrite(D0, 1);

	}
}

void IRAM_ATTR Button4DetectionAct (void)
{
	delayMicroseconds(750);

	if((digitalRead(D9) == 0))
	{
		DBG_OUTPUT_PORT.printf("B4=ON\n");
		digitalWrite(D10, 0);
		u8BUtton4Pressed = 0x01;
	}
	else if((digitalRead(D9) == 1))
	{
		DBG_OUTPUT_PORT.printf("B4=OFF\n");
		digitalWrite(D10, 1);

	}
}


void IRAM_ATTR ZeroCrossingDetector (void)
{

	if(FanCtrl == 0x00)
	{
		FanSpeedUpdate = analogRead(A0);

	}
	else
	{
		FanSpeedUpdate = FanSpeedNet;
	}



	if((FanSpeedUpdate > 60) && (FanSpeedUpdate < 950))
	{
	  analogWrite(D5, FanSpeedUpdate);
	}
	else if(FanSpeedUpdate < 60)
	{
	  analogWrite(D5, 0);

	}
	else if(FanSpeedUpdate > 950)
	{
	  analogWrite(D5, 1023);

	}
	else
	{

	}
}


void listDir(const char * dirname) {
  DBG_OUTPUT_PORT.printf("Listing directory: %s\n", dirname);

  Dir root = LittleFS.openDir(dirname);

  while (root.next()) {
    File file = root.openFile("r");
    DBG_OUTPUT_PORT.print("  FILE: ");
    DBG_OUTPUT_PORT.print(root.fileName());
    DBG_OUTPUT_PORT.print("  SIZE:  ");
    DBG_OUTPUT_PORT.print(file.size());
    if (file.size() > 1024)
    {
      DBG_OUTPUT_PORT.print("  Over Size:  ");
      return;
    }
    file.close();
  }
}

void writeData(String data)
{
  //Open the file
  File file = LittleFS.open("/config.json", "w");
  //Write to the file
  file.print(data);
  //Close the file
  file.close();
  delay(1);
  DBG_OUTPUT_PORT.println("Write successful");
}
void deleteData()
{
   //Remove the file
   LittleFS.remove("/config.json");
}


void readFile(const char * path)
{

  DBG_OUTPUT_PORT.printf("Reading file: %s\n", path);

  char FileRead[1024] = {0, 0};
  uint32_t TempCount = 0;

  File file = LittleFS.open(path, "r");
  if (!file) {
    DBG_OUTPUT_PORT.println("Failed to open file for reading");
    return;
  }

  DBG_OUTPUT_PORT.print("Read from file: ");
  while (file.available())
  {

    char temp = file.read();
    DBG_OUTPUT_PORT.write(temp);
    FileRead[TempCount++] = temp;
  }
#if 1
  // Allocate the memory pool on the stack.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<1024> doc;

  // Parse the root object
  //JsonObject &root = jsonBuffer.parseObject(configFile);

  auto error = deserializeJson(doc, FileRead);

  if (error) {
    DBG_OUTPUT_PORT.print(F("deserializeJson() failed with code "));
    DBG_OUTPUT_PORT.println(error.c_str());
    return;
  }




  file.close();
  // Copy values from the JsonObject to the Config
  ssid =  doc["testString"];
  password = doc["testNumber"];

  DBG_OUTPUT_PORT.print(ssid);
  DBG_OUTPUT_PORT.print(password);


#endif


}

bool loadConfigFile()
// Load existing configuration file
{
  // Uncomment if we need to format filesystem
  // SPIFFS.format();

  // Read configuration from FS json
	DBG_OUTPUT_PORT.println("Mounting File System...");

  // May need to make it begin(true) first time you are using SPIFFS
  if (LittleFS.begin())
  {
    DBG_OUTPUT_PORT.println("mounted file system");
    if (LittleFS.exists(JSON_CONFIG_FILE))
    {
      // The file exists, reading and loading
      DBG_OUTPUT_PORT.println("reading config file");
      File configFile = LittleFS.open(JSON_CONFIG_FILE, "r");
      if (configFile)
      {
        DBG_OUTPUT_PORT.println("Opened configuration file");
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJsonPretty(json, DBG_OUTPUT_PORT);
        if (!error)
        {
          DBG_OUTPUT_PORT.println("Parsing JSON");

          strcpy(testString, json["testString"]);
          testNumber = json["testNumber"].as<int>();

          return true;
        }
        else
        {
          // Error loading JSON data
          DBG_OUTPUT_PORT.println("Failed to load json config");
        }
      }
    }
  }
  else
  {
    // Error mounting file system
    DBG_OUTPUT_PORT.println("Failed to mount FS");
  }

  return false;
}






void saveConfigFile()
// Save Config in JSON format
{
	DBG_OUTPUT_PORT.println(F("Saving configuration..."));

  // Create a JSON document
  StaticJsonDocument<512> json;
  json["testString"] = testString;
  json["testNumber"] = testNumber;

  // Open config file
  File configFile = LittleFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile)
  {
    // Error, file did not open
	DBG_OUTPUT_PORT.println("failed to open config file for writing");
  }

  // Serialize JSON data to write to file
  serializeJsonPretty(json, DBG_OUTPUT_PORT);
  if (serializeJson(json, configFile) == 0)
  {
    // Error writing file
	  DBG_OUTPUT_PORT.println(F("Failed to write to file"));
  }
  // Close file
  configFile.close();
}


void saveConfigCallback()
// Callback notifying us of the need to save configuration
{
	DBG_OUTPUT_PORT.println("Should save config");
  shouldSaveConfig = true;
}

void configModeCallback(WiFiManager *myWiFiManager)
// Called when config mode launched
{
	DBG_OUTPUT_PORT.println("Entered Configuration Mode");

	DBG_OUTPUT_PORT.print("Config SSID: ");
	DBG_OUTPUT_PORT.println(myWiFiManager->getConfigPortalSSID());

	DBG_OUTPUT_PORT.print("Config IP Address: ");
	DBG_OUTPUT_PORT.println(WiFi.softAPIP());
}

// Set global to avoid object removing after setup() routine
//Pinger pinger;
#define NO_HARD_CODE
void setup()
{
   unsigned char u8Timwout= 0;

  // Init serial debug Communication
  DBG_OUTPUT_PORT.begin(115200);
  DBG_OUTPUT_PORT.print("\n");
  DBG_OUTPUT_PORT.setDebugOutput(true);

#ifdef NO_HARD_CODE
  // Change to true when testing to force configuration every time we run
  bool forceConfig = false;
  // Create WiFiManager object
  WiFiManager wfm;
#ifdef DEVELOP
  //In Development mode there should be no delay for Router Power Cycle
#else
  //In Release mode there should be delay for Router Power Cycle for 4 min
  wfm.setConfigPortalTimeout(240);
#endif

  // Supress Debug information
  wfm.setDebugOutput(false);

  // Remove any previous network settings
  // wfm.resetSettings();

   // Set config save notify callback
  wfm.setSaveConfigCallback(saveConfigCallback);

  // Set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wfm.setAPCallback(configModeCallback);

  // Define a text box, 50 characters maximum
  WiFiManagerParameter custom_text_box("my_text", "Enter your string here", "default string", 50);


  // Need to convert numerical input to string to display the default value.
   char convertedValue[6];
   sprintf(convertedValue, "%d", testNumber);

   // Text box (Number) - 7 characters maximum
   WiFiManagerParameter custom_text_box_num("key_num", "Enter your number here", convertedValue, 7);

  // Add custom parameter
  wfm.addParameter(&custom_text_box);
  wfm.addParameter(&custom_text_box_num);

  //DBG_OUTPUT_PORT.println("Mount LittleFS");
  if (!LittleFS.begin()) {
    DBG_OUTPUT_PORT.println("LittleFS mount failed");
    return;
  }

  listDir("/");
//  bool spiffsSetup = LittleFS.open("/config.json","r");
  bool spiffsSetup = loadConfigFile();

  if (!spiffsSetup)
  {
	 DBG_OUTPUT_PORT.println(F("Forcing config mode as there is no saved config"));
    forceConfig = true;
  }
  //WIFI INIT
  String stringone = "GarudaAnsh";
  stringone.concat(WiFi.macAddress());

 const char *MacPointer;
  MacPointer = &stringone[0];

  if (forceConfig)
      // Run if we need a configuration
    {

      if (!wfm.startConfigPortal(MacPointer, "password"))
      {
    	  DBG_OUTPUT_PORT.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.restart();
        delay(5000);
      }
    }
    else
    {
      if (!wfm.autoConnect(MacPointer, "password"))
      {
    	  DBG_OUTPUT_PORT.println("failed to connect and hit timeout");
        delay(3000);
        // if we still have not connected restart and try all over again
        ESP.restart();
        delay(5000);
      }
    }

  readFile("/config.json");
  // If we get here, we are connected to the WiFi

  DBG_OUTPUT_PORT.println("");
  DBG_OUTPUT_PORT.println("WiFi connected");
  DBG_OUTPUT_PORT.print("IP address: ");
  DBG_OUTPUT_PORT.println(WiFi.localIP());

    // Lets deal with the user config values

    // Copy the string value
    strncpy(testString, custom_text_box.getValue(), sizeof(testString));
    DBG_OUTPUT_PORT.print("testString: ");
    DBG_OUTPUT_PORT.println(testString);

    //Convert the number value
    testNumber = atoi(custom_text_box_num.getValue());
    DBG_OUTPUT_PORT.print("testNumber: ");
    DBG_OUTPUT_PORT.println(testNumber);


    // Save the custom parameters to FS
	 DBG_OUTPUT_PORT.printf("shouldSaveConfig %d ", shouldSaveConfig);

    if (shouldSaveConfig)
    {
    	 DBG_OUTPUT_PORT.printf("shouldSaveConfig %d ", shouldSaveConfig);
      saveConfigFile();
    }

  WiFi.setSleepMode(WIFI_NONE_SLEEP);
#if 1
  //Zero Cross detector Init
  //PWM Dutycycle change for FAN output
  analogWrite(D5, 0);

  pinMode(D1, OUTPUT);   /*O/P 1 */
  pinMode(D6, OUTPUT);   /*O/P 2 */
  pinMode(D0, OUTPUT); 	 /*O/P 3 */
  pinMode(D10, OUTPUT);  /*O/P 4 */

  digitalWrite(D1,0);
  digitalWrite(D6,0);
  digitalWrite(D0,0);
  digitalWrite(D10,0);
#endif

  Udp.begin(localUdpPort);
  DBG_OUTPUT_PORT.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

   ArduinoOTA.onStart([]() {
     String type;
     if (ArduinoOTA.getCommand() == U_FLASH) {
       type = "sketch";
     } else { // U_FS
       type = "filesystem";
     }
    // analogWrite(D5, 0);
     detachInterrupt(D2);// disable Interrupt for down load while OTA update
     // NOTE: if updating FS this would be the place to unmount FS using FS.end()
     DBG_OUTPUT_PORT.println("Start updating " + type);
   });
   ArduinoOTA.onEnd([]() {
     DBG_OUTPUT_PORT.println("\nEnd");
   });
   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     DBG_OUTPUT_PORT.printf("Progress: %u%%\r", (progress / (total / 100)));
   });
   ArduinoOTA.onError([](ota_error_t error) {
     DBG_OUTPUT_PORT.printf("Error[%u]: ", error);
     if (error == OTA_AUTH_ERROR) {
       DBG_OUTPUT_PORT.println("Auth Failed");
     } else if (error == OTA_BEGIN_ERROR) {
       DBG_OUTPUT_PORT.println("Begin Failed");
     } else if (error == OTA_CONNECT_ERROR) {
       DBG_OUTPUT_PORT.println("Connect Failed");
     } else if (error == OTA_RECEIVE_ERROR) {
       DBG_OUTPUT_PORT.println("Receive Failed");
     } else if (error == OTA_END_ERROR) {
       DBG_OUTPUT_PORT.println("End Failed");
     }
   });
   ArduinoOTA.begin();

   /*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

  //Zero Cross detector Init
  pinMode(D2, INPUT_PULLUP);
  attachInterrupt(D2, ZeroCrossingDetector, FALLING);

  //Button Input at RESET

  //Digital GPIO Configured as input
  pinMode(D3, INPUT_PULLUP );  /*I/P 1 */
  /***************************************************************/
  pinMode(D8, OUTPUT );  	   /*O/P 2 */ //Force fully as Output
  /***************************************************************/
  pinMode(D7, INPUT_PULLUP );  /*I/P 3 */
  pinMode(D9, INPUT_PULLUP );  /*I/P 4 */


//  attachInterrupt(D3, Button1DetectionAct, CHANGE);
//  attachInterrupt(D8, Button2DetectionAct, CHANGE);
//  attachInterrupt(D7, Button3DetectionAct, CHANGE);
//  attachInterrupt(D9, Button4DetectionAct, CHANGE);

  /***************************************************************/
 // digitalWrite(D8,0);  	   /*O/P 2 */ /*Force fully as Output for button detection*/
  /***************************************************************/

//  gatewayIP = WiFi.gatewayIP();

  DBG_OUTPUT_PORT.println("\n");
  DBG_OUTPUT_PORT.print("Gateway! IP address: ");
  DBG_OUTPUT_PORT.print(WiFi.gatewayIP());

  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);   /*SDA */
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);   /*SCL */



  /****************************************************************************/

  /* Start Control register configuration with Autp store enable */

  DBG_OUTPUT_PORT.printf("\nMemory == %d\n", I2C_transmit(I2C_SRAM_CTRL_SLAVE, Memorydata, 3));
  /*End Control register configuration with Autp store enable */

  /* Start Read the SRAM for Digital IO and Analog Input*/
  Memorydata[0] = 0x00;
  Memorydata[1] = 0x00;

  delay(10);

  DBG_OUTPUT_PORT.printf("MemoryWrite == %d\n",I2C_transmit_read_SRAM(I2C_SRAM_RW_SLAVE, Memorydata, 2));
  DBG_OUTPUT_PORT.printf("MemoryRead == %d\n", I2C_receive_SRAM(I2C_SRAM_RW_SLAVE, Memorydata, 5));

  for(unsigned char i = 0; i< 5; i++)
  {
	  DBG_OUTPUT_PORT.printf("MemoryRead == %d\n",Memorydata[i]);
  }
  /* END Read the SRAM for Digital IO and Analog Input*/
  /***************************************************************************/

#endif
}


void loop ()
{

 static unsigned char u8FanTriggerInc = 0;
 static unsigned char u8FanTriggerDec = 0;
 static unsigned char SwUpdateRequest = 0x00;
 static unsigned int WiFIScan = 0;
 static unsigned int IsquareScan = 0;


 ArduinoOTA.handle();

 if(SwUpdateRequest == 0x00)
 {
	 ArduinoOTA.handle();
 	 currentMillis = millis();
 	// if WiFi is down, try reconnecting
 	if(currentMillis > previousMillis)
 	{
 		if (((currentMillis - previousMillis)) >= interval)
		{
 			if(WiFIScan == 3000)
 			{
				if((WiFi.status() != WL_CONNECTED))
				{
					DBG_OUTPUT_PORT.print(millis());
					DBG_OUTPUT_PORT.println("Reconnecting to WiFi...");
					WiFi.disconnect();
					WiFi.reconnect();
				}
				else
				{
					DBG_OUTPUT_PORT.println("LOOP1");
					Udp.beginPacket(gatewayIP, 57650);
					Udp.write("A");
					Udp.endPacket();
				}

				WiFIScan = 0;
 			}
 			else
 			{
 				WiFIScan++;
 			}

 			if(IsquareScan == 5)
 			{
 				DBG_OUTPUT_PORT.printf("F1 == %d\n", I2C_receive_adc(I2C_ADC_SLAVE, data, 2));
 				IsquareScan =0;
 			}
 			else
 			{
 				IsquareScan++;
 			}
 			previousMillis = currentMillis;
		}
 	}
 	else
 	{
 		if (((currentMillis) - (0xFFFFFFFF - previousMillis)) >= interval)
		{

 			if(WiFIScan == 3000)
 			{
				if((WiFi.status() != WL_CONNECTED))
				{
					DBG_OUTPUT_PORT.print(millis());
					DBG_OUTPUT_PORT.println("Reconnecting to WiFi...");
					WiFi.disconnect();
					WiFi.reconnect();

				}
				else
				{
					DBG_OUTPUT_PORT.println("LOOP2");
					Udp.beginPacket(gatewayIP, 57650);
					Udp.write("B");
					Udp.endPacket();
				}
				WiFIScan = 0;

 			}
 			else
 			{
 				WiFIScan++;
 			}

 			if(IsquareScan == 5)
 			{
				DBG_OUTPUT_PORT.printf("F1 == %d\n", I2C_receive_adc(I2C_ADC_SLAVE, data, 2));
 			  //I2C_receive_adc(I2C_ADC_SLAVE, data, 2);
 				IsquareScan =0;
 			}
 			else
 			{
 				IsquareScan++;
 			}
		}
		previousMillis = currentMillis;


 	}

 	ESP.wdtFeed();

 	packetSize = Udp.parsePacket();
	if (packetSize)
		{
			packetSize = 0;

			// receive incoming UDP packets
			DBG_OUTPUT_PORT.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
			int len = Udp.read(incomingPacket, 255);
			if (len > 0)
			{
			  incomingPacket[len] = '\0';
			}
     		 DBG_OUTPUT_PORT.printf("UDP packet contents: %s\n", incomingPacket);

				/*********************Code For  OnOff with UDP message and Input Buttons************************************************/


				  if (((strcmp(incomingPacket, "FAN ON")  == 0)))
				  {
					digitalWrite(D10, 0);
					strcpy(replyPacket, "ON FAN");
					IOPortdata = 0xFF;
	 				DBG_OUTPUT_PORT.printf("Port == %d\n", I2C_transmit(I2C_PORT_SLAVE, &IOPortdata, 1));

				  }
				  else if (((strcmp(incomingPacket, "FAN OFF") == 0)))
				  {
					digitalWrite(D10, 1);
					strcpy(replyPacket, "OFF FAN");
					IOPortdata = 0x00;
					DBG_OUTPUT_PORT.printf("Port == %d\n", I2C_transmit(I2C_PORT_SLAVE, &IOPortdata, 1));
				  }
				  /*********************Code For OnOff with UDP message and Input Buttons************************************************/


			 /*********************Code For Tubelight OnOff with UDP message and Input Buttons************************************************/

			  if (((strcmp(incomingPacket, "TUBELIGHT ON") == 0)))
			  {
				digitalWrite(D0, 0);
				strcpy(replyPacket, "UDP TUBELIGHT ON");
				Memorydata[0] = 0x00;
				Memorydata[1] = 0x01;
				Memorydata[2] = 111; //^ Memorydata[2];
				DBG_OUTPUT_PORT.printf("MemoryWrite == %d\n", I2C_transmit(I2C_SRAM_RW_SLAVE, Memorydata, 3));

			  }
			  else if (((strcmp(incomingPacket, "TUBELIGHT OFF") == 0)))
			  {
				digitalWrite(D0, 1);
				strcpy(replyPacket, "UDP TUBELIGHT OFF");

				Memorydata[0] = 0x00;
				Memorydata[1] = 0x01;
				Memorydata[2] = 99 ;//^ Memorydata[2];
				DBG_OUTPUT_PORT.printf("MemoryWrite == %d\n", I2C_transmit(I2C_SRAM_RW_SLAVE, Memorydata, 3));
			  }
			  /*********************Code For Tubelight OnOff with UDP message and Input Buttons************************************************/


			/*********************Code For PLug OnOff with UDP message and Input Buttons************************************************/
			  if (((strcmp(incomingPacket, "PLUG ON")  == 0)))
			  {
				digitalWrite(D6, 0);
				strcpy(replyPacket, "UDP PLUG ON");
			  }
			  else if (((strcmp(incomingPacket, "PLUG OFF") ==0 )))
			  {
				digitalWrite(D6, 1);
				strcpy(replyPacket, "UDP PLUG OFF");
			  }
    	  /*********************Code For Plug OnOff with UDP message and Input Buttons************************************************/

			/*********************Code For Balcony Light OnOff with UDP message and Input Buttons************************************************/
			  if (((strcmp(incomingPacket, "LIGHT OFF") == 0U)))
			  {
				digitalWrite(D1, 1);
				strcpy(replyPacket, "UDP LIGHT OFF");
			  }
			  else if (((strcmp(incomingPacket, "LIGHT ON")  == 0U)))
			  {
				digitalWrite(D1, 0);
				strcpy(replyPacket, "UDP LIGHT ON");
			  }
			  /*********************Code For Balcony light OnOff with UDP message and Input Buttons*****************************8****/
			  /*********************Code For  OnOff with UDP message and Input Buttons************************************************/


			  if (((strcmp(incomingPacket, "FAN INC") == 0)))
			  {

				  FanCtrl = 1;
				  if(u8FanTriggerInc == 0x00)
				  {
					  FanSpeedNet = analogRead(A0);
					  u8FanTriggerInc = 0x01;
				  }
				  if(FanSpeedNet < 950)
				  {
					  FanSpeedNet += 50;
				  }
				  strcpy(replyPacket, "UDP FAN INC");
				  data[0] = 0xFF;
				while(!(I2C_transmit(I2C_PORT_SLAVE, data, 1)));


			  }
			  else if ((strcmp(incomingPacket, "FAN DEC")) == 0)
			  {
				  FanCtrl = 1;
				  if(u8FanTriggerDec == 0x00)
					  {
						  FanSpeedNet = analogRead(A0);
						  u8FanTriggerDec = 0x01;
					  }
				  if(FanSpeedNet > 50)
				  {
					  FanSpeedNet -= 50;
				  }
				  strcpy(replyPacket, "UDP FAN DEC");

				  data[0] = 0x00;
				  while(!(I2C_transmit(I2C_PORT_SLAVE, data, 1)));
			  }
			  else
			  {
				  //Do Nothing

			  }


			  if (((strcmp(incomingPacket, "MANUAL DEBUG") == 0)))
			  {
					// send back a reply, to the IP address and port we got the packet from
					Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
					Udp.write(ManualreplyPacket);
					Udp.endPacket();
			  }
			  else if (((strcmp(incomingPacket, "Software Update Request") == 0)))
			  {
				  SwUpdateRequest = 0x55;
				  strcpy(replyPacket, "UDP ACK Software Update Requested");

					// send back a reply, to the IP address and port we got the packet from
					Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
					Udp.write(replyPacket);
					Udp.endPacket();
			  }
			  else if (((strcmp(incomingPacket, "Are U There?") == 0)))
			  {
				  strcpy(replyPacket, "UDP ACK Yes I Am here?");

					// send back a reply, to the IP address and port we got the packet from
					Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
					Udp.write(replyPacket);
					Udp.endPacket();
			  }
			  else
			  {
				 // delay(500);
					// send back a reply, to the IP address and port we got the packet from
					Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
					Udp.write(replyPacket);
					Udp.endPacket();
			  }


/*********************Code For OnOff with UDP message and Input Buttons************************************************/
		}
	else
	{

	}

/****************************Manually Override the System****************************************************************/
	if(FanCtrl == 1)
	{
		FanSpeedDiff = analogRead(A0);

		if((u8TimerLoop >= 10))
		{
			if(abs((int) FanSpeedDiff - (int)FanSPeedPrevios) >= 100)
			{
				u8TimerLoop = 0;
				FanSPeedPrevios = analogRead(A0);
				FanCtrl = 0;
				u8FanTriggerDec = 0x00;
				u8FanTriggerDec = 0x00;
				FanSpeedUpdate = FanSpeedNet;
			}
		}
		u8TimerLoop++;

	}
	else
	{
		/* Do Nothing*/
	}
 }
 else
 {
	 ArduinoOTA.handle();
 }

}


void WithoutIp(void)
{

#if 1
  //Zero Cross detector Init
  //PWM Dutycycle change for FAN output
  analogWrite(D5, 0);

  pinMode(D1, OUTPUT);   /*O/P 1 */
  pinMode(D6, OUTPUT);   /*O/P 2 */
  pinMode(D0, OUTPUT); 	 /*O/P 3 */
  pinMode(D10, OUTPUT);  /*O/P 4 */

  digitalWrite(D1,0);
  digitalWrite(D6,0);
  digitalWrite(D0,0);
  digitalWrite(D10,0);

  //Button Input at RESET

  //Digital GPIO Configured as input
  pinMode(D3, INPUT_PULLUP );  /*I/P 1 */
  /***************************************************************/
  pinMode(D8, OUTPUT );  	   /*O/P 2 */ //Force fully as Output
  /***************************************************************/
  pinMode(D7, INPUT_PULLUP );  /*I/P 3 */
  pinMode(D9, INPUT_PULLUP );  /*I/P 4 */


  attachInterrupt(D3, Button1DetectionAct, CHANGE);
  attachInterrupt(D8, Button2DetectionAct, CHANGE);
  attachInterrupt(D7, Button3DetectionAct, CHANGE);
  attachInterrupt(D9, Button4DetectionAct, CHANGE);

  /***************************************************************/
  digitalWrite(D8,1);  	   /*O/P 2 */ /*Force fully as Output for button detection*/
  /***************************************************************/
#endif

	FanCtrl = 0;
	  //Zero Cross detector Init
	  pinMode(D2, INPUT_PULLUP);
	  attachInterrupt(D2, ZeroCrossingDetector, FALLING);

	while(1)
	{
		unsigned long currentMillis = millis();
		// if WiFi is down, try reconnecting
		if ((((currentMillis - previousMillis)) >=interval))
		{

			if((WiFi.status() != WL_CONNECTED))
			{
				DBG_OUTPUT_PORT.print(millis());
				DBG_OUTPUT_PORT.println("Reconnecting to WiFi...");
				WiFi.disconnect();
				WiFi.reconnect();
				previousMillis = currentMillis;
			}
			else
			{
				break;
			}
		}
		ESP.wdtFeed();
        DBG_OUTPUT_PORT.println("IN WITHOUT_IP_LOOP");
	}
}


//void I2C_bus_init(unsigned char scl_pin, unsigned char sda_pin, unsigned char port){
//	  /*Configure GPIO pins : SW_I2C_SCL_Pin SW_I2C_SDA_Pin */
//	  GPIO_InitStruct.Pin = SW_I2C_SCL_Pin|SW_I2C_SDA_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//	  GPIO_InitStruct.Pull = GPIO_PULLUP;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//}

#define true 1
#define false 0

void I2C_init(void)
{
    I2C_SET_SDA;
    I2C_SET_SCL;
}

void I2C_start_cond(void)
{
    I2C_SET_SCL
    I2C_SET_SDA
    I2C_DELAY
    I2C_CLEAR_SDA
    I2C_DELAY
    I2C_CLEAR_SCL
    I2C_DELAY
}

void I2C_stop_cond(void)
{
    I2C_CLEAR_SDA
    I2C_DELAY
    I2C_SET_SCL
    I2C_DELAY
    I2C_SET_SDA
    I2C_DELAY
}

void I2C_write_bit(unsigned char b)
{
    if (b > 0)
        I2C_SET_SDA
    else
        I2C_CLEAR_SDA

    I2C_DELAY
    I2C_SET_SCL
    I2C_DELAY
    I2C_CLEAR_SCL
}

unsigned char I2C_read_SDA(void)
{
    if (digitalRead(SDA_PIN) == GPIO_PIN_SET)
        return 1;
    else
        return 0;
    return 0;
}

// Reading a bit in I2C:
unsigned char I2C_read_bit(void)
{
    unsigned char b;

    I2C_SET_SDA
    I2C_DELAY
    I2C_SET_SCL
    I2C_DELAY



    b = I2C_read_SDA();

    I2C_CLEAR_SCL

    return b;
}

unsigned char I2C_write_byte(unsigned char B,
                     unsigned char start,
                     unsigned char stop)
{
    unsigned char ack = 0;

    if (start)
        I2C_start_cond();

    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        I2C_write_bit(B & 0x80); // write the most-significant bit
        B <<= 1;
    }

    ack = I2C_read_bit();

    if (stop)
        I2C_stop_cond();

    return !ack; //0-ack, 1-nack
}

// Reading a byte with I2C:
unsigned char I2C_read_byte(unsigned char ack, unsigned char stop)
{
    unsigned char B = 0;

    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        B <<= 1;
        B |= I2C_read_bit();
    }

    if (ack)
        I2C_write_bit(0);
    else
        I2C_write_bit(1);

    if (stop)
        I2C_stop_cond();

    return B;
}

// Sending a byte with I2C:
unsigned char I2C_send_byte(unsigned char address,
                    unsigned char data)
{
    //    if( I2C_write_byte( address << 1, true, false ) )   // start, send address, write
    if (I2C_write_byte(address, true, false)) // start, send address, write
    {
        // send data, stop
        if (I2C_write_byte(data, false, true))
            return true;
    }

    I2C_stop_cond(); // make sure to impose a stop if NAK'd
    return false;
}

// Receiving a byte with a I2C:
unsigned char I2C_receive_byte(unsigned char address)
{
    if (I2C_write_byte((address << 1) | 0x01, true, false)) // start, send address, read
    {
        return I2C_read_byte(false, true);
    }

    return 0; // return zero if NAK'd
}

// Sending a byte of data with I2C:
unsigned char I2C_send_byte_data(unsigned char address,
                         unsigned char reg,
                         unsigned char data)
{
    //    if( I2C_write_byte( address << 1, true, false ) )   // start, send address, write
    if (I2C_write_byte(address, true, false))
    {
        if (I2C_write_byte(reg, false, false)) // send desired register
        {
            if (I2C_write_byte(data, false, true))
                return true; // send data, stop
        }
    }

    I2C_stop_cond();
    return false;
}

// Receiving a byte of data with I2C:
unsigned char I2C_receive_byte_data(unsigned char address,
                              unsigned char reg)
{
    //if( I2C_write_byte( address << 1, true, false ) )   // start, send address, write
    if (I2C_write_byte(address, true, false))
    {
        if (I2C_write_byte(reg, false, false)) // send desired register
        {
            if (I2C_write_byte((address << 1) | 0x01, true, false)) // start again, send address, read
            {
                return I2C_read_byte(false, true); // read data
            }
        }
    }

    I2C_stop_cond();
    return 0; // return zero if NACKed
}

unsigned char I2C_transmit(unsigned char address, unsigned char data[], unsigned char size)
{
    if (I2C_write_byte(address, true, false)) // first byte
    {
        for (int i = 0; i < size; i++)
        {
            if (i == size - 1)
            {
                if (I2C_write_byte(data[i], false, true))
                    return true;
            }
            else
            {
                if (!I2C_write_byte(data[i], false, false))
                    break; //last byte
            }
        }
    }
    I2C_stop_cond();
    return false;
}

unsigned char I2C_receive(unsigned char address, unsigned char reg[], unsigned char *data, unsigned char reg_size, unsigned char size)
{
    if (I2C_write_byte(address, true, false))
    {
        for (int i = 0; i < reg_size; i++)
        {
            if (!I2C_write_byte(reg[i], false, false))
                break;
        }
        if (I2C_write_byte(address | 0x01, true, false)) // start again, send address, read (LSB signifies R or W)
        {
            for (int j = 0; j < size; j++)
            {
                *data++ = I2C_read_byte(false, false); // read data
            }
            I2C_stop_cond();
            return true;
        }
    }
    I2C_stop_cond();
    return false;
}

unsigned int I2C_receive_adc(unsigned char address, unsigned char *data, unsigned char size)
{
	unsigned int RET = 0;
    if (I2C_write_byte((address | 0x01), true, true))
    {
            for (int j = 0; j < size - 1; j++)
            {
                *data++ = I2C_read_byte(true, false); // read data
            }            *data = I2C_read_byte(false, true); // read data

            RET = (data[1]) << 8;

            RET = ((data[0]) | RET );

            RET = RET >> 2;

           return(RET);
     }
    I2C_stop_cond();
    return false;
}
unsigned int I2C_receive_SRAM(unsigned char address, unsigned char *data, unsigned char size)
{

    if (I2C_write_byte((address | 0x01), true, false))
    {
            for (int j = 0; j < size - 1; j++)
            {
                *data++ = I2C_read_byte(true, false); // read data
            }
            *data = I2C_read_byte(false, true); // read data
            return true;

     }
    I2C_stop_cond();
    return false;
}
unsigned char I2C_transmit_read_SRAM(unsigned char address, unsigned char data[], unsigned char size)
{
    if (I2C_write_byte(address, true, false)) // first byte
    {
        for (int i = 0; i < size; i++)
        {
            if (i == size - 1)
            {
                if (I2C_write_byte(data[i], false, false))
                {
                	/* Do NOthing*/
                }

            }
            else
            {
                if (I2C_write_byte(data[i], false, false))
                	return true; //last byte
            }
        }
    }
	I2C_stop_cond();
    return false;
}
