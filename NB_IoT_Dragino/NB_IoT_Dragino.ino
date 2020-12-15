#include <SoftwareSerial.h>
#include <LowPower.h>
#include <AES.h>
#include "./printf.h"
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

// Hardware PINS and delay times
#define PIR 3
#define NB_IOT_POWER 13
#define BG96_BOOTING_TIME 10000
#define SEND_UDP_DATA_DELAY 100
#define SEND_TCP_DATA_DELAY 100
#define AT_RETRY_DELAY 2000
#define RETRY_NR 10
#define WAIT_FOR_REPLY 150000

// AT commands
#define AT "AT"
#define NWSCANMODE "AT+QCFG=\"nwscanmode\",3,1"
#define IOTOPMODE "AT+QCFG=\"iotopmode\",1,1"
#define APN "AT+QICSGP=1,1,\"NBIOT.TELIA.EE\",\"\",\"\",0"
#define CIMI "AT+CIMI"
#define NETWORK_REG "AT+CREG=2"
#define READ_NETWORK_REG_STATUS "AT+CREG?"
#define QUERY_NETWORK_INFO "AT+QNWINFO"
#define READ_OPERATOR_SELECTION "AT+COPS?"
#define ACTIVATE_DPD_CONTEXT "AT+QIACT=1"
#define CHECK_IF_GOT_IP_ADDRESS "AT+QIACT?"
#define OPEN_SOCKET_SERVICE "AT+QIOPEN=1,2,\"UDP SERVICE\",\"127.0.0.1\",0,6805,0"
#define QUERY_SOCKET_SERVICE_STATUS "AT+QISTATE=0,1"
#define CONF_DNS "AT+QIDNSCFG=1,\"92.62.96.27\""
#define SYNC_LOCAL_TIME_WITH_NTP_SERVER "AT+QNTP=1,\"ee.pool.ntp.org\""
#define GET_TIME "AT+QLTS=2"
#define GET_SIGNAL_STRENGTH "AT+QCSQ"
#define GET_HARDWARE_CLOCK "AT+CCLK?"
#define POWER_DOWN "AT+QPOWD=1"
#define MIN_FUNCTIONALITY "AT+CFUN=0"
#define FULL_FUNCTIONALITY "AT+CFUN=1"
#define DEACT_PDP "AT+QIDEACT=1"
#define AIRPLANE_MODE "AT+CFUN=4"
#define PING "AT+QPING=1,\"193.40.246.34\",10,1"
#define CLOSE_TCP "AT+QICLOSE=0" 

AES aes ;
SoftwareSerial mySerial(10, 11); // RX, TX Dragino

String readString;
String writeString;
String IP = "90.30.95.15";  // Server IP
String Port = "12345";        // Server Port
String identifier = "ID";     // ID of the device
bool movement = false;
unsigned long start_time;
unsigned long end_time;
int i = 0;
int k = 0;
int analogPin = A0;
int val = 0;

byte *key = (unsigned char*)"0123456789010qwe"; //Encryption key

//real iv = iv x2 ex: 01234567 = 0123456701234567
unsigned long long int my_iv = 00000000;//36753562

void setup() { // Configure UART, NB-IoT, etc
  pinMode(NB_IOT_POWER,OUTPUT);
  delay(10);
  digitalWrite(NB_IOT_POWER, LOW); 
  mySerial.begin(57600);
  delay(100); 
  pinMode(PIR,INPUT);

  wake_up_module();
  nb_iot_setup();
}
void loop() { // MAIN
  detect_movement();
}

void detect_movement(){
  // Detect movement and send encrypted data to server
  if(digitalRead(PIR) == HIGH){
    send_data(encryptMessage(128, identifier + ',' + getTime() + ',' + getBatteryLevel()));
    delay(11000);
  }
  else{ // Go to power saving mode until movement detected
    attachInterrupt(digitalPinToInterrupt(PIR), isr, HIGH);
    delay(10);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    delay(10);
    detachInterrupt(digitalPinToInterrupt(PIR));
    delay(10);
    send_data(encryptMessage(128, identifier + ',' + getTime() + ',' + getBatteryLevel()));
  }

}

void nb_iot_setup(){ // Configuration
  //checkForError(NWSCANMODE, AT_RETRY_DELAY);
  //checkForError(IOTOPMODE, AT_RETRY_DELAY);
  checkForError(APN, AT_RETRY_DELAY);
  //checkForError(QUERY_NETWORK_INFO, AT_RETRY_DELAY);
  //checkForError(READ_OPERATOR_SELECTION, AT_RETRY_DELAY);
  checkForError(ACTIVATE_DPD_CONTEXT, AT_RETRY_DELAY);
  checkForError(CHECK_IF_GOT_IP_ADDRESS, AT_RETRY_DELAY);
  checkForError(CONF_DNS, AT_RETRY_DELAY);
  checkForError(SYNC_LOCAL_TIME_WITH_NTP_SERVER, AT_RETRY_DELAY);
  get_2nd_reply();
  //checkForError(OPEN_SOCKET_SERVICE, AT_RETRY_DELAY);
  //checkForError(QUERY_SOCKET_SERVICE_STATUS, AT_RETRY_DELAY);
}

String writeUART(String msg){ // Send message to UART (to NB-IoT)
  char c;
  String returnbuf = "";
  if (msg.length() > 0){
    mySerial.println(msg);
    mySerial.flush();
    start_time = millis();
    while (returnbuf == ""){
      delay(10);  
      end_time = millis();
      if(end_time - start_time > WAIT_FOR_REPLY){
        restart_module();
        break;
      }
      while (mySerial.available()  > 0) {
        c = mySerial.read();
        if ((' ' <= c) && (c <= '~')){
          returnbuf += c;
        }
      }    
    }
  }
  return returnbuf;
}
void checkForError(String msg, int delay_value_for_next_command){ // Check for errors, if error, retry, or restart if too many errors
  int i = 0;
  String res = "";
  String res2 = "";
  
  while(i < RETRY_NR){
    res = writeUART(msg);
    if (res.indexOf("ERROR") == -1 and res.indexOf("SEND FAIL") == -1 and res.indexOf("No Service") == -1 and res.indexOf("OK") != -1){
      break;
    }
    else if (res.indexOf("ERROR") != -1 or res.indexOf("SEND FAIL") != -1 or res.indexOf("No Service") != -1){
      i++;
      delay(AT_RETRY_DELAY);
    }
    else{
      res2 = get_2nd_reply();
      if (res2.indexOf("ERROR") == -1 and res2.indexOf("SEND FAIL") == -1 and res2.indexOf("No Service") == -1 and res2.indexOf("OK") != -1){
        break;
      }
    }
  }
  if (i == RETRY_NR){
    restart_module();
  }
  delay(delay_value_for_next_command);
}

void send_data(String msg){ // Send data trhough TCP
  unsigned int data_len = msg.length();
  checkForError("AT+QIOPEN=1,0,\"TCP\"" + String(",") + "\"" + IP + "\"" + "," + Port, SEND_TCP_DATA_DELAY); // TCP
  //Serial.println("TCP");
  String test = get_2nd_reply();
  if (test.indexOf("+QIOPEN: 0,0") != -1){
    writeUART("AT+QISEND=0," + String(data_len));
    delay(SEND_TCP_DATA_DELAY);
    writeUART(msg);
    delay(SEND_TCP_DATA_DELAY);
  }
  writeUART(CLOSE_TCP);
}

void restart_module(){ // Restart module in case of crash
  power_down_module();
  wake_up_module();
  nb_iot_setup();
}

void waiting_for_the_startup(){ // Wait for the NB-IoT app to be ready
  String returnbuf = "";
  char c;
  while(true){
    while (mySerial.available() > 0) {
      c = mySerial.read();
      if ((' ' <= c) && (c <= '~')){
        returnbuf += c;
      }
    }
    if (returnbuf.indexOf("APP RDY") != -1){
      break;
    }
    delay(10);
  }
}

void clean_input_buffer(){ // Clean input buffer
  while (mySerial.available() > 0) {
    char c = mySerial.read();
  }  
}

String get_2nd_reply(){ // Wait for the module for the 2nd reply
  String returnbuf = "";
  char c;
  start_time = millis();
  while(true){
    while (mySerial.available() > 0) {
      c = mySerial.read();
      if ((' ' <= c) && (c <= '~')){
        returnbuf += c;
      }
    }
    if (returnbuf.length() != 0){
      break;
    }
    delay(10);
    end_time = millis();
    if(end_time - start_time > WAIT_FOR_REPLY){
      restart_module();
      break;
    }
  }
  return returnbuf;  
}

String encrypt_16_Bytes (int bits, byte *plain, int plainLength) // Encrypt one block
{
  int padedLength = plainLength + N_BLOCK - plainLength % N_BLOCK;
  String hex = "";
  char hexString[2] = "";
  aes.iv_inc();
  byte iv [N_BLOCK] ;
  byte cipher [padedLength] ;
  aes.set_IV(my_iv);
  aes.get_IV(iv);
  aes.do_aes_encrypt(plain,plainLength,cipher,key,bits,iv);
  for(int i = 0; i < sizeof(cipher); i++){
    sprintf(hexString, "%02X", cipher[i]);
    hex  = hex + hexString;
  }
  return hex;
}

String encryptMessage(int bits, String message) // Encrypt whole message, divide it into blocks
{
  String messageHex = "";
  int str_len = message.length() + 1;
  
  do{
    if (str_len > 16){
      byte plain[16];
      message.getBytes(plain, 16);
      messageHex = messageHex + (encrypt_16_Bytes(bits, plain, sizeof(plain) - 1));
    }
    else{
      byte plain[str_len];
      message.getBytes(plain, str_len);
      messageHex = messageHex + (encrypt_16_Bytes(bits, plain, sizeof(plain) - 1));
    }
    message.remove(0,15);
    str_len -= 15;
  }while(str_len > 0);
    return messageHex;
}

String getTime(){ // Get hardware clock 
  String hours = "";
  int correctedHours = 0;
  char buffer[3] = "";
  clean_input_buffer();  
  String result = writeUART(GET_HARDWARE_CLOCK);
  if (result.length() != 39 or result.indexOf("ERROR") != -1){
    return "TIME_ERR";
  }
  result.remove(0, 25);
  result.remove(8);
  hours = result.charAt(0);
  hours = hours + result.charAt(1);
  correctedHours = hours.toInt() + 3; // Clock 3h behind, update it and correct mistakes
  if (correctedHours == 03 and hours != "00"){
    return "TIME_ERR";
  }
  if (correctedHours == 24){
    correctedHours = 00;
  }
  if (correctedHours == 25){
    correctedHours = 01;
  }
  if (correctedHours == 26){
    correctedHours = 02;
  }
  sprintf(buffer, "%02d", correctedHours);
  result.remove(0, 2);
  result = buffer + result;
  return result;
}

void power_down_to_ready_for_sending(){ // Config
  wake_up_module();
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  checkForError(ACTIVATE_DPD_CONTEXT, AT_RETRY_DELAY);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  checkForError(OPEN_SOCKET_SERVICE, AT_RETRY_DELAY);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  send_data(encryptMessage(128, identifier + getTime()));
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  power_down_module();
}
void wake_up_module(){
  digitalWrite(NB_IOT_POWER, HIGH);
  waiting_for_the_startup();
}
void power_down_module(){
  writeUART("AT+QIDEACT=1");
  delay(500);
  digitalWrite(NB_IOT_POWER, LOW);
  delay(5000);
}

void min_functionality_to_ready_for_sending(){ // Config
  checkForError(FULL_FUNCTIONALITY, AT_RETRY_DELAY);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  checkForError(ACTIVATE_DPD_CONTEXT, AT_RETRY_DELAY);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  checkForError(OPEN_SOCKET_SERVICE, AT_RETRY_DELAY);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  send_data(encryptMessage(128, identifier + getTime()));
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  checkForError(MIN_FUNCTIONALITY, AT_RETRY_DELAY);
}
void airplanemode_to_ready_for_sending(){ // Config
  checkForError(FULL_FUNCTIONALITY, AT_RETRY_DELAY);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  checkForError(ACTIVATE_DPD_CONTEXT, AT_RETRY_DELAY);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  checkForError(OPEN_SOCKET_SERVICE, AT_RETRY_DELAY);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  send_data(encryptMessage(128, identifier + getTime()));
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  checkForError(AIRPLANE_MODE, AT_RETRY_DELAY);
}

void send_every_8_sec(){ // Send data
  int i = 0;
  while(true){
    send_data(encryptMessage(128, identifier + ',' + String(i) + ',' + getTime()));
    Serial.println(i);
    i += 1;
    delay(10000);
  }
}

void send_every_20_min(){ // Send data
  for(int i = 0; i < 11000; i++){
    send_data(encryptMessage(128, identifier + ',' + String(i) + ',' + getTime()));
    delay(1241400);//1000*60*20.69
  }
}
 
void send_every_7_min(){ // Send data
  for(int i = 0; i < 11000; i++){
    send_data(encryptMessage(128, identifier + ',' + String(i) + ',' + getTime()));
    delay(433800);//1000*60*7.23
  }
}

void isr(){ //
  //Serial.println("Movement!");
  //movement = true;
}

String getBatteryLevel(){ // Get battery level from 01 to 99
  val = analogRead(analogPin);  // read the input pin
  delay(5);
  val += analogRead(analogPin);
  val = val/2;
  if (val <= 613){ // Battery empty
    return "01";
  }
  else if(val >= 850){ // Battery full
    return "99";
  }
  else{
    int b_level = (val - 611)/2.41; // Calculate battery level
    if (b_level < 10){
      return ('0' + String(b_level));
    }
    return (String(b_level));
  }
}

void communication(){ //Testing AT commands through PC
  while (mySerial.available()) {
    delay(1);  //delay to allow byte to arrive in input buffer
    char c = mySerial.read();
    if ((' ' <= c) && (c <= '~')){
      readString += c;
    }
  }
  if (readString.length() >0) {
    Serial.println(readString.length());
    Serial.println(readString);
    readString="";
  }
  while (Serial.available()) {
    delay(1);  //delay to allow byte to arrive in input buffer
    char c = Serial.read();
    writeString += c;
  }
  if (writeString.length() >0) {
    mySerial.println(writeString);
    mySerial.flush();
    writeString="";
  }  
}
