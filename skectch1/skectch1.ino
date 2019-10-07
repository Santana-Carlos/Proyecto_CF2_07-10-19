//----------Librerias & Variables Sensor Temp Piel ----------------
#include <OneWire.h>
#include <DallasTemperature.h>

double TEMPSKIN;
//---------------------------------------------------

//----------------Librerias conexión servidor web--------------------------
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
 
const char* ssid = "LAURAS";
const char* pass = "LSFGLJCM";
//-------------------------------------------------------------------------

//---------------------------OneWire(sensor temp piel)---------------------
// El cable de datos se conecta al pin 13 del ESP32
#define ONE_WIRE_BUS 13
// Definir una instancia de OneWire para comunicarse
// con cualquier dispositivo OneWire
OneWire oneWire(ONE_WIRE_BUS);
// Pasar la referencia a OneWire del
// Sensor de temperatura Dallas
DallasTemperature sensors(&oneWire);
//-------------------------------------------------------------------------

//-------------------Variables alertas-----------------------------
int PINLEDG = 12;
int PINLEDB = 25;
int PINBUZZ = 14;
//-----------------------------------------------------------------

//------------------------------Variables sensor respuesta galvanica-------
const int GSR = 26;
int sensorValue = 0;
int gsr_average = 0;
//-------------------------------------------------------------------------

//-------------------Variables Json----------------
float BODY_TEMP = 0;
int BODY_RES = 0;
float AMB_TEMP = 0;
float AMB_HUM = 0;
int DEV_AX = 0;
int DEV_AY = 0;
int DEV_AZ = 0;
int DEV_GX = 0;
int DEV_GY = 0;
int DEV_GZ = 0;
int DEV_MX = 0;
int DEV_MY = 0;
int DEV_MZ = 0;
float DEV_TEMP = 0;
//-------------------------------------------------

//-----------------------------Librerias y Variables IMU---------------------
#include <MPU9250.h>
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
//---------------------------------------------------------------------------------

//-----------------------------Variables sensor Temp & Humedad Ambiental-----------------------------------------------
int DHpin = 27; // input/output pin
byte dat[5];   

byte read_data()
{
  byte i = 0;
  byte result = 0;
  for (i = 0; i < 8; i++) {
      while (digitalRead(DHpin) == LOW); // wait 50us
      delayMicroseconds(30); //The duration of the high level is judged to determine whether the data is '0' or '1'
      if (digitalRead(DHpin) == HIGH)
        result |= (1 << (8 - i)); //High in the former, low in the post
    while (digitalRead(DHpin) == HIGH); //Data '1', waiting for the next bit of reception
    }
  return result;
}
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------Metodos sensor temp & humedad Ambiente----------------------------------------------------
void start_test()
{
  digitalWrite(DHpin, LOW); //Pull down the bus to send the start signal
  delay(30); //The delay is greater than 18 ms so that DHT 11 can detect the start signal
  digitalWrite(DHpin, HIGH);
  delayMicroseconds(40); //Wait for DHT11 to respond
  pinMode(DHpin, INPUT);
  while(digitalRead(DHpin) == HIGH);
  delayMicroseconds(80); //The DHT11 responds by pulling the bus low for 80us;
  
  if(digitalRead(DHpin) == LOW)
    delayMicroseconds(80); //DHT11 pulled up after the bus 80us to start sending data;
  for(int i = 0; i < 5; i++) //Receiving temperature and humidity data, check bits are not considered;
    dat[i] = read_data();
  pinMode(DHpin, OUTPUT);
  digitalWrite(DHpin, HIGH); //After the completion of a release of data bus, waiting for the host to start the next signal
}

void sensorAmb() {
  start_test();
  Serial.print("Humdity = ");
  Serial.print(dat[0], DEC); //Displays the integer bits of humidity;
  Serial.print('.');
  Serial.print(dat[1], DEC); //Displays the decimal places of the humidity;
  Serial.println('%');
  AMB_HUM = (dat[0]+(dat[1]*0.01));
  Serial.print("Temperature = ");
  Serial.print(dat[2], DEC); //Displays the integer bits of temperature;
  Serial.print('.');
  Serial.print(dat[3], DEC); //Displays the decimal places of the temperature;
  Serial.println('C');
  AMB_TEMP = (dat[2]+(dat[3]*0.01));

  byte checksum = dat[0] + dat[1] + dat[2] + dat[3];
  if (dat[4] != checksum) 
    Serial.println("-- Checksum Error!");
  else
    Serial.println("-- OK");
 
  
}
//-----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------Metodo sensor temp piel---------------------------
void sensorTempSkin() {
  TEMPSKIN = sensors.getTempCByIndex(0);
  // llamar a sensors.requestTemperatures() para emitir
 // un pedido de temperatura global a todos los que esten en el bus
 Serial.print("Solicitando temperaturas...");
 sensors.requestTemperatures();  // Enviar el comando para obtener la temperatura
 Serial.println("Hecho");
 Serial.print("La temperatura en el dispositivo 1 es: ");
 Serial.print(TEMPSKIN);
 BODY_TEMP = TEMPSKIN;
 if (TEMPSKIN >= 30.00){
  alerta(1);
 }
 // uso de getTempCByIndex para obtener mas datos
}
//------------------------------------------------------------------------------------


//------------------------------Metodo sensor respuesta galvanica-----
void sensorRespGalv (){
  long sum=0;
  for(int i=0;i<10;i++)           //Average the 10 measurements to remove the glitch
      {
      sensorValue=analogRead(GSR);
      sum += sensorValue;
      delay(5);
      }
   gsr_average = sum/10;
   Serial.println(gsr_average);

   BODY_RES = gsr_average;
}

//--------------------------------------------------------------------

//-------------------------------------Métodos IMU--------------------------------------------------
void giroscopio(){
  // put your main code here, to run repeatedly:
  // read the sensor
  IMU.readSensor();

  //aceleracion
  Serial.print("aceleracion en X es:");
  float ax = IMU.getAccelX_mss();
  Serial.println((int) ax);
  Serial.print("");
  DEV_AX = (int) ax;

  Serial.print("aceleracion en Y es:");
  float ay = IMU.getAccelY_mss();
  Serial.println((int) ay);
  Serial.print("");
  DEV_AY = (int) ay;

  Serial.print("aceleracion en Z es:");
  float az = IMU.getAccelZ_mss();
  Serial.println((int) az);
  Serial.println("");
  DEV_AZ = (int) az;
  int speedP = 10;
  int speedN = speedP * (-1);
  if ((DEV_AX >= speedP) || (DEV_AX <= speedN) || (DEV_AY >= speedP) || (DEV_AY <= speedN) || (DEV_AZ >= speedP) || (DEV_AZ <= speedN)){
  alerta(2);
 }
  
  //Giros
  Serial.print(IMU.getGyroX_rads());
  Serial.print("\t");
  DEV_GX = IMU.getGyroX_rads();
  Serial.print(IMU.getGyroY_rads());
  Serial.print("\t");
  DEV_GY = IMU.getGyroY_rads();
  Serial.print(IMU.getGyroZ_rads());
  Serial.print("\t"); 
  DEV_GZ = IMU.getGyroZ_rads();

  //campo magnetico
  Serial.print((int)IMU.getMagX_uT());
  Serial.print("\t");
  DEV_MX = (int)IMU.getMagX_uT();
  Serial.print((int)IMU.getMagY_uT());
  Serial.print("\t");
  DEV_MY = (int)IMU.getMagY_uT();
  Serial.print((int)IMU.getMagZ_uT());
  Serial.print("\t");
  DEV_MZ = (int)IMU.getMagZ_uT();

  //sensor de teperatura
  Serial.print("la temperatura del circuito es: ");
  Serial.println(IMU.getTemperature_C(),6);
  DEV_TEMP = IMU.getTemperature_C();
}
//------------------------------------------------------------------------------------------

//-------------------------------Metodo Alertas-----------------------------------

void alerta (int T){
  if (T == 1){
    digitalWrite(PINLEDB, HIGH);
    digitalWrite(PINBUZZ, HIGH);
    delay(200);
    digitalWrite(PINLEDB, LOW);
    digitalWrite(PINBUZZ, LOW);
    delay(200);
    digitalWrite(PINLEDB, HIGH);
    digitalWrite(PINBUZZ, HIGH);
    delay(200);
    digitalWrite(PINLEDB, LOW);
    digitalWrite(PINBUZZ, LOW);
    delay(200);
    digitalWrite(PINLEDB, HIGH);
    digitalWrite(PINBUZZ, HIGH);
    delay(200);
    digitalWrite(PINLEDB, LOW);
    digitalWrite(PINBUZZ, LOW);
  }
  
  if (T == 2){
    digitalWrite(PINLEDG, HIGH);
    digitalWrite(PINBUZZ, HIGH);
    delay(200);
    digitalWrite(PINLEDG, LOW);
    digitalWrite(PINBUZZ, LOW);
    delay(200);
    digitalWrite(PINLEDG, HIGH);
    digitalWrite(PINBUZZ, HIGH);
    delay(200);
    digitalWrite(PINLEDG, LOW);
    digitalWrite(PINBUZZ, LOW);
    delay(200);
    digitalWrite(PINLEDG, HIGH);
    digitalWrite(PINBUZZ, HIGH);
    delay(200);
    digitalWrite(PINLEDG, LOW);
    digitalWrite(PINBUZZ, LOW);
  }
}
//--------------------------------------------------------------------------------

//----------------------------------------Métodos WiFi JSon----------------------------------
void conexion (){
  StaticJsonBuffer<1500> JSONbuffer;
  JsonObject& JSONmessage = JSONbuffer.createObject();
  JSONmessage["time"] = "2018-10-23 10:23:54+02";
  JsonObject& body = JSONmessage.createNestedObject("body");
  JsonObject& ambient = JSONmessage.createNestedObject("ambient");
  JsonObject& device = JSONmessage.createNestedObject("device");
  device["UUID"] = "550e8400-e29b-41d4-a716-446655440000";
  JsonObject& temperature = body.createNestedObject("temperature");
  temperature["value"] = BODY_TEMP;
  temperature["type"] = "float";
  temperature["unit"] = "°C";
  JsonObject& resistance = body.createNestedObject("resistance");
  resistance["value"] = BODY_RES;
  resistance["type"] = "int";
  resistance["unit"] = "Ohm";
  //JsonObject& BPM = body.createNestedObject("BPM");
  //JsonObject& Sp02 = body.createNestedObject("Sp02");
  JsonObject& temperatureAmb = ambient.createNestedObject("temperatureAmb");
  temperatureAmb["value"] = AMB_TEMP;
  temperatureAmb["type"] = "float";
  temperatureAmb["unit"] = "°C";
  JsonObject& humidity = ambient.createNestedObject("humidity");
  humidity["value"] = AMB_HUM;
  humidity["type"] = "float";
  humidity["unit"] = "%";
  JsonObject& aceleration = device.createNestedObject("aceleration");
  JsonObject& ax = aceleration.createNestedObject("ax");
  ax["value"] = DEV_AX;
  ax["type"] = "int";
  ax["unit"] = "mg";
  JsonObject& ay = aceleration.createNestedObject("ay");
  ay["value"] = DEV_AY;
  ay["type"] = "int";
  ay["unit"] = "mg";
  JsonObject& az = aceleration.createNestedObject("az");
  az["value"] = DEV_AZ;
  az["type"] = "int";
  az["unit"] = "mg";
  JsonObject& gyro = device.createNestedObject("gyro");
  JsonObject& gx = gyro.createNestedObject("gx");
  gx["value"] = DEV_GX;
  gx["type"] = "int";
  gx["unit"] = "deg/s";
  JsonObject& gy = gyro.createNestedObject("gy");
  gy["value"] = DEV_GY;
  gy["type"] = "int";
  gy["unit"] = "deg/s";
  JsonObject& gz = gyro.createNestedObject("gz");
  gz["value"] = DEV_GZ;
  gz["type"] = "int";
  gz["unit"] = "deg/s";
  JsonObject& magnetometer = device.createNestedObject("magnetometer");
  JsonObject& mx = magnetometer.createNestedObject("mx");
  mx["value"] = DEV_MX;
  mx["type"] = "int";
  mx["unit"] = "mG";
  JsonObject& my = magnetometer.createNestedObject("my");
  my["value"] = DEV_MY;
  my["type"] = "int";
  my["unit"] = "mG";
  JsonObject& mz = magnetometer.createNestedObject("mz");
  mz["value"] = DEV_MZ;
  mz["type"] = "int";
  mz["unit"] = "mG";
  JsonObject& tempIMU = device.createNestedObject("tempIMU");
  tempIMU["value"] = DEV_TEMP;
  tempIMU["type"] = "float";
  tempIMU["unit"] = "°C";
  
  char JSONmessageBuffer[1500];
  JSONmessage.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  JSONmessage.prettyPrintTo(Serial);
 
 if(WiFi.status()== WL_CONNECTED){   //Check WiFi connection status
 
   HTTPClient http;   
 
   http.begin("https://ptsv2.com/t/umabr-1570403313/post");  //Specify destination for HTTP request
   int httpResponseCode = http.POST(JSONmessageBuffer);   //Send the actual POST request
 
   if(httpResponseCode>0){
 
    String response = http.getString();                       //Get the response to the request
 
    Serial.println(httpResponseCode);   //Print return code
    Serial.println(response);           //Print request answer
 
   }else{
 
    Serial.print("Error on sending POST: ");
    Serial.println(httpResponseCode);
 
   }
 
   http.end();  //Free resources
 
 }else{
 
    Serial.println("Error in WiFi connection");   
 
 }
}

void setup(void)
{
 // inicia el puerto serie
 Serial.begin(9600);

 //Conexión WiFi
 WiFi.begin(ssid,pass);
  while (WiFi.status() != WL_CONNECTED) { //Check for the connection
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
 //------------------------
 
 pinMode(DHpin, OUTPUT);//iniciar pin (sensor temp & hum amb)

 pinMode(PINLEDG, OUTPUT);
 pinMode(PINLEDB, OUTPUT);
 pinMode(PINBUZZ, OUTPUT);
 

 digitalWrite(PINLEDB, LOW);
 digitalWrite(PINLEDG, LOW);
 digitalWrite(PINBUZZ, LOW);
 
 Serial.println("Demo de la biblioteca de control del chip de temperatura Dallas");
 // Iniciar la biblioteca
 sensors.begin(); // Establece valor por defecto del chip 9 bit. Si tiene problemas pruebe aumentar

 Wire.begin(); //Iniciar wire (conección Cl2)

 // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}


void loop(void)
{
 sensorTempSkin();
 Serial.println(' ');
 sensorAmb();
 Serial.println(' ');
 sensorRespGalv();
 Serial.println(' ');
 giroscopio();
 Serial.println("-------------------------");
 Serial.println(' ');
 conexion();
 delay(20);
}
