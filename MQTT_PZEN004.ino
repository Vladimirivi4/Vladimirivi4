/*
   Wiring:
   PZEM 004T v3.0 to NodeMCU
   5v to V3/vin
   RX to D6 (has TX Pin)
   TX to D5 (has RX Pin)
   GND to GND
   Relay to NodeMCU
 GND to GND
   IN1 to D2
   IN2 to D1
   IN3 to D4
   IN4 to D0
   VCC to Vin (not 3v)
*/

#include "settings.h"           // можно объеденить но лень
#include "secret.h"               //можно объеденить но лень
#include <SimpleTimer.h>        //если нет то качнуть с инета
#include <ModbusMaster.h>       //если нет то качнуть с инета
#include <ESP8266WiFi.h>        //если нет то качнуть с инета
#include "ThingSpeak.h"         //если нет то качнуть с инета
#include <SoftwareSerial.h>                                   //(NODEMCU ESP8266)
#include <PubSubClient.h> //********************************************mqtt

char ssid[] = SECRET_SSID;   // из секрета
char pass[] = SECRET_PASS;   // из секрета
int keyIndex = 0;            // your network key Index number (needed only for WEP) 

char msgBuffer[20];  

const char* mqtt_server = "192.168.43.210"; //********************mqtt


WiFiClient  espClient;
PubSubClient client(espClient); //*******************************************mqtt

unsigned long lastMsg = 0;      //*******************************************mqtt
#define MSG_BUFFER_SIZE  (50)  //*******************************************mqtt
char msg[MSG_BUFFER_SIZE];    //*******************************************mqtt
int value = 0;               //*******************************************mqtt


SoftwareSerial pzemSerial(RX_PIN_NODEMCU, TX_PIN_NODEMCU);    //(RX,TX) NodeMCU connect to (TX,RX) of PZEM

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;
int number = 0;
// обьявление посоянных - статических адресов PZEM
static uint8_t pzemSlave1Addr = PZEM_SLAVE_1_ADDRESS; 
static uint8_t pzemSlave2Addr = PZEM_SLAVE_2_ADDRESS;
static uint8_t pzemSlave3Addr = PZEM_SLAVE_3_ADDRESS;

// определяем ручками сколь приборов
ModbusMaster node1;
ModbusMaster node2;
ModbusMaster node3;

//функция таймера из библиотеки SimpleTimer 
SimpleTimer timer;

//объявление типов получаемых данных, почему то выбрали double - ниже конвертировал в float иначе округление до целого
double voltage_usage_1      = 0;
double current_usage_1      = 0;
double active_power_1       = 0;
double active_energy_1      = 0;
double frequency_1          = 0;
double power_factor_1       = 0;

double voltage_usage_2      = 0;
double current_usage_2      = 0;
double active_power_2       = 0;
double active_energy_2      = 0;
double frequency_2          = 0;
double power_factor_2       = 0;

double voltage_usage_3      = 0;
double current_usage_3      = 0;
double active_power_3       = 0;
double active_energy_3      = 0;
double frequency_3          = 0;
double power_factor_3       = 0;

double sum_of_voltage       = 0;
double sum_of_current       = 0;
double sum_of_power         = 0;
double sum_of_active_energy = 0;
double sum_of_frequency     = 0;
double sum_of_power_factor  = 0;


//int underVoltageAlertOnOffState;         // Under voltage one time alert on off state variable
//int phaseFailureAlertOnOffState;         // Phase failure voltage one time alert on off state variable
//bool underVoltageAlertFlag = true;       // Under voltage alert flag set to true 


void setup()
{
  
   /* start Modbus/RS-485 serial communication */
Serial.begin(115200);  //Initialize serial
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo native USB port only
  }
  
  WiFi.mode(WIFI_STA);   //определение режима работывафли на устройствет точка доступа или клиент
  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
 pzemSerial.begin(9600);
  node1.begin(pzemSlave1Addr, pzemSerial);
  node2.begin(pzemSlave2Addr, pzemSerial);
  node3.begin(pzemSlave3Addr, pzemSerial);

 
changeAddress(0XF8, 0x01);                 // раскоментировать для назначения ПЗЕМКЕ адреса
resetEnergy(0x01);                        // раскоментировать для обнуления энергопотребляшки

timer.setInterval(GET_PZEM_DATA_TIME, pzemdevice);                   // функция интервального выполнения задачи в данном случае 20 сек выполняем pzemdevice -см. ниже

} 

//*******************************************mqtt
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//*******************************************mqtt


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "helloooooo world");
     // client.publish("outTopic", voltage_usage_1);
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void pzemdevice()                                                            // Function to get PZEM device 1 data
{
  Serial.println("====================================================");     // PZEM Device 1 data fetching code starts here
  Serial.println("Now checking PZEM Device 1");
  uint8_t result1;

  ESP.wdtDisable();                                                           // Disable watchdog during modbus read or else ESP crashes when no slave connected
  result1 = node1.readInputRegisters(0x0000, 10);
  ESP.wdtEnable(1);                                                           // Enable watchdog during modbus read

 
  if (result1 == node1.ku8MBSuccess)                                          //если адрес пземки совпадает с запрашиваем в настройках то запрашиваем  данные иначе пропускаем и пишем что не нашли "Failed to read PZEM Device 1"
  {
    voltage_usage_1      = (node1.getResponseBuffer(0x00) / 10.0f);
    current_usage_1      = (node1.getResponseBuffer(0x01) / 1000.000f);
    active_power_1       = (node1.getResponseBuffer(0x03) / 10.0f);
    active_energy_1      = (node1.getResponseBuffer(0x05) / 1000.0f);
    frequency_1          = (node1.getResponseBuffer(0x07) / 10.0f);
    power_factor_1       = (node1.getResponseBuffer(0x08) / 100.0f);

 client.publish("VOLTAGE/A", dtostrf((float)voltage_usage_1, 3, 2, msgBuffer));
 client.publish("CURRENT_USAGE/A", dtostrf((float)current_usage_1, 3, 2, msgBuffer));
 client.publish("ACTIVE_POWER/A", dtostrf((float)active_power_1, 3, 2, msgBuffer));
 client.publish("ACTIVE_ENERGY/A", dtostrf((float)active_energy_1, 3, 2, msgBuffer));
 client.publish("FREQUENCY/A", dtostrf((float)frequency_1, 3, 2, msgBuffer));
 client.publish("POWER_FACTOR/A", dtostrf((float)power_factor_1, 3, 2, msgBuffer));

/*
Функция, которая поможет это нам сделать — dtostrf()
Разберем функцию и посмотрим ее использование.
dtostrf(<переменная с плавающей>, <символов всего>, <символов после запятой>, <переменная char>)
после выполнения dtostrf, результат уже можно перевести в char*.
Пример использования:

    char msgBuffer[20];           // убедитесь что стройки размера хватит
    char *pointer_to_created_string;
 
    float testFloat = 123.45;
 
    pointer_to_created_string = dtostrf(testFloat, 6, 2, msgBuffer);
    client.Publish("...topic...", pointer_to_created_string)

    Ну или более компактный вариант:
     char msgBuffer[20];           // длина строки должна быть достаточно большой.
    float testFloat = 123.45;
 
    client.Publish("...topic...", dtostrf(testFloat, 6, 2, msgBuffer));
 */

//для отладки можно коментить
         
    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_1);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_1, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_1);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_1, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_1);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_1);
    Serial.println("====================================================");
  }
  else {
    Serial.println("Failed to read PZEM Device 1");
    Serial.println("PZEM Device 1 Data");
    voltage_usage_1      = 0;
    current_usage_1      = 0;
    active_power_1       = 0;
    active_energy_1      = 0;
    frequency_1          = 0;
    power_factor_1       = 0;
  
    
    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_1);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_1, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_1);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_1, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_1);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_1);

      
    Serial.println("====================================================");
    //для отладки можно коментить
  }
  


/*

{
  Serial.println("====================================================");
  Serial.println("Now checking PZEM Device 2");
  uint8_t result2;

  ESP.wdtDisable();
  result2 = node2.readInputRegisters(0x0000, 10);
  ESP.wdtEnable(1);

  if (result2 == node2.ku8MBSuccess)
  {
    voltage_usage_2      = (node2.getResponseBuffer(0x00) / 10.0f);
    current_usage_2      = (node2.getResponseBuffer(0x01) / 1000.000f);
    active_power_2       = (node2.getResponseBuffer(0x03) / 10.0f);
    active_energy_2      = (node2.getResponseBuffer(0x05) / 1000.0f);
    frequency_2          = (node2.getResponseBuffer(0x07) / 10.0f);
    power_factor_2       = (node2.getResponseBuffer(0x08) / 100.0f);

    
    //ThingSpeak.setField( 3, (float) voltage_usage_2);
    //ThingSpeak.setField( 4, (float) current_usage_2);
 
 //для отладки можно коментить
    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_2);         // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_2, 3);      // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_2);          // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_2, 3);      // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_2);             // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_2);
    Serial.println("====================================================");
  }
    else {
    Serial.println("Failed to read PZEM Device 2");
    Serial.println("PZEM Device 2 Data");
    voltage_usage_2      = 0;
    current_usage_2      = 0;
    active_power_2       = 0;
    active_energy_2      = 0;
    frequency_2          = 0;
    power_factor_2       = 0;
    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_2);         // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_2, 3);      // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_2);          // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_2, 3);      // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_2);             // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_2);
    Serial.println("====================================================");
//для отладки можно коментить
  }



{
  Serial.println("====================================================");     // PZEM Device 1 data fetching code starts here
  Serial.println("Now checking PZEM Device 3");
  uint8_t result3;

  ESP.wdtDisable();                                                           // Disable watchdog during modbus read or else ESP crashes when no slave connected
  result3 = node3.readInputRegisters(0x0000, 10);
  ESP.wdtEnable(1);                                                           // Enable watchdog during modbus read

  if (result3 == node3.ku8MBSuccess)
  {
    voltage_usage_3      = (node3.getResponseBuffer(0x00) / 10.0f);
    current_usage_3      = (node3.getResponseBuffer(0x01) / 1000.000f);
    active_power_3       = (node3.getResponseBuffer(0x03) / 10.0f);
    active_energy_3      = (node3.getResponseBuffer(0x05) / 1000.0f);
    frequency_3          = (node3.getResponseBuffer(0x07) / 10.0f);
    power_factor_3       = (node3.getResponseBuffer(0x08) / 100.0f);

   // ThingSpeak.setField( 5, (float) voltage_usage_3);
    //ThingSpeak.setField( 6, (float) current_usage_3);


//для отладки можно коментить

    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_3);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_3, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_3);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_3, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_3);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_3);
    Serial.println("====================================================");
  }
  else {
    Serial.println("Failed to read PZEM Device 3");
    Serial.println("PZEM Device 3 Data");
    voltage_usage_3      = 0;                                                     // Assigning 0 if it fails to read PZEM device
    current_usage_3      = 0;
    active_power_3       = 0;
    active_energy_3      = 0;
    frequency_3          = 0;
    power_factor_3       = 0;
    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_3);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_3, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_3);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_3, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_3);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_3);
    Serial.println("====================================================");
////для отладки можно коментить

  }
}
}
    
   
    //ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey); //обновляем данные на сайте синг спик
    //Serial.println("Channel update successful.");*/
}

/*void sumofpzem()
{
    Serial.println("Sum of all 3 PZEM devices");
    sum_of_voltage        =   (voltage_usage_1 + voltage_usage_2 + voltage_usage_3);
    sum_of_current        =   (current_usage_1 + current_usage_2 + current_usage_3);
    sum_of_power          =   (active_power_1 + active_power_2 + active_power_3);
    sum_of_active_energy  =   (active_energy_1 + active_energy_2 + active_energy_3); 
    sum_of_frequency      =   (frequency_1 + frequency_2 + frequency_3);
    sum_of_power_factor   =   (power_factor_1 + power_factor_2 + power_factor_3); 
    
    Serial.print("SUM of VOLTAGE:           ");   Serial.println(sum_of_voltage);             // V
    Serial.print("SUM of CURRENT_USAGE:     ");   Serial.println(sum_of_current, 3);          // A
    Serial.print("SUM of ACTIVE_POWER:      ");   Serial.println(sum_of_power);               // W
    Serial.print("SUM of ACTIVE_ENERGY:     ");   Serial.println(sum_of_active_energy, 3);    // kWh
    Serial.print("SUM of FREQUENCY:         ");   Serial.println(sum_of_frequency);           // Hz
    Serial.print("SUM of POWER_FACTOR:      ");   Serial.println(sum_of_power_factor);
    Serial.println("====================================================");
      
  
}
*/
void resetEnergy(uint8_t slaveAddr)                                                // функция сброса энергопотребляшки с пземки после смены можно закоментировать для уменьшения размера 
{
  /* The command to reset the slave's energy is (total 4 bytes):
     Slave address + 0x42 + CRC check high byte + CRC check low byte. */
  uint16_t u16CRC = 0xFFFF;
  static uint8_t resetCommand = 0x42;
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  Serial.println("Resetting Energy");
  pzemSerial.write(slaveAddr);
  pzemSerial.write(resetCommand);
  pzemSerial.write(lowByte(u16CRC));
  pzemSerial.write(highByte(u16CRC));
  delay(1000);
}

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr)                    // функция изменения адреса пземки после смены можно закоментировать для уменьшения размера 
{
  static uint8_t SlaveParameter = 0x06;
  static uint16_t registerAddress = 0x0002;                                       // адресс регистра для измеения
  uint16_t u16CRC = 0xFFFF;
  u16CRC = crc16_update(u16CRC, OldslaveAddr);
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
  Serial.println("Changing Slave Address");
  pzemSerial.write(OldslaveAddr);
  pzemSerial.write(SlaveParameter);
  pzemSerial.write(highByte(registerAddress));
  pzemSerial.write(lowByte(registerAddress));
  pzemSerial.write(highByte(NewslaveAddr));
  pzemSerial.write(lowByte(NewslaveAddr));
  pzemSerial.write(lowByte(u16CRC));
  pzemSerial.write(highByte(u16CRC));
  Serial.println("Changing Slave Address is done"); 
  delay(1000);
}



void loop()
{
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, "hello worlando #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("outTopic", msg);
  
  }
   
  //timer.run();
  pzemdevice(); 

    Serial.println("Выполнено"); //для отладки можно коментить
  delay(5000);

}
  

   
   
 
