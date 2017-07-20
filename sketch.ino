// добавляем необходимые библиотеки
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

SoftwareSerial GPRSSerial(2, 3); // RX, TX
LiquidCrystal lcd(8, 9, 4, 5, 6, 7 );

int valpump = 0;     // хранит значение пина контроля реле насоса
int valjet = 0;     // хранит значение пина контроля реле форсунки
int valcat = 0;		// хранит значение пина контроля уровня катализатора (с ацп)
int lcdcatval = 0;	// хранит пересчитанное значение уровня катализатора для отображения на дисплее
String secondline;	//здесь собираем состояние реле насоса и форсунки для вывода во вторую строку дисплея
String PUMPstr[]={"PUMP ON", "PUMP OFF"};
String JETstr[]={"JET ON", "JET OFF"};
const String siteaddress = "185.86.119.8";   //ip адрес сервера или адрес сайта для отправки данных
const String actionscript = "pumptest.php";  //скрипт на сервере для приема данных

// Нажатые кнопки
int button;
const int BUTTON_NONE   = 0;
const int BUTTON_RIGHT  = 1;
const int BUTTON_UP     = 2;
const int BUTTON_DOWN   = 3;
const int BUTTON_LEFT   = 4;
const int BUTTON_SELECT = 5;
const int LCDBUTTONSPIN = 0; // считываем значения нажатых кнопок с аналогового входа(A0)

//пины, к которым подключены реле и кнопки
const int PUMPRELAY     = 19;	//управление реле насоса
const int JETRELAY      = 18;	//управление реле форсунки
const int PUMPCONTROL   = 17;	//контроль реле насоса
const int JETCONTROL    = 16;	//контроль реле форсунки
const int POWERGPRS     = 12;	//управление питанием модуля SIM900
const int ALARMSIGNAL   = 13;	//вывод на пьезо пищалку
const int CATLEVCONTR   = 15;	//аналоговый вход для контроля уровня катализатора

//организация задержек для 
//   отправки данных на сервер (25 секунд)
unsigned long previousT0Millis = 0;   
const long intervalT0 = 25000;    
//  обновления данных на дисплее (1 секнда)
unsigned long previousT1Millis = 0;    
const long intervalT1 = 1000;    

//вкл сирену при аварии (вкл.-выкл.)
unsigned long alarmison = 0;    

void setup() { // инициализация входных и выходных пинов, переферии
  Serial.begin(19200);
  Serial.println("LCD INIT");
  lcd.begin(16, 2);
  lcd.print("Initialisation");

  Serial.println("ALARM INIT");
  lcd.clear();
  lcd.print("ALARM INIT");
  pinMode(ALARMSIGNAL, OUTPUT);
  delay(300);
  tone(ALARMSIGNAL,1000,300);
  
  Serial.println("PUMPRELAY INIT");
  lcd.clear();
  lcd.print("PUMPRELAY INIT");
  pinMode(PUMPRELAY, OUTPUT);
  digitalWrite(PUMPRELAY, HIGH);
  delay(300);
  
  Serial.println("JETRELAY INIT");
  lcd.clear();
  lcd.print("JETRELAY INIT");
  pinMode(JETRELAY, OUTPUT);
  digitalWrite(JETRELAY, HIGH);
  delay(300);
  
  Serial.println("PUMPCONTROL INIT");
  lcd.clear();
  lcd.print("PUMPCONTROL INIT");
  pinMode(PUMPCONTROL, INPUT_PULLUP);
  delay(300);
  
  Serial.println("JETCONTROL INIT");
  lcd.clear();
  lcd.print("JETCONTROL INIT");
  pinMode(JETCONTROL, INPUT_PULLUP);
  delay(300);
 
  Serial.println("GPRS INIT");
  pinMode(POWERGPRS, OUTPUT); 
  GPRSSerial.begin(19200);               // скорость последовательного порта модуля SIM900
  
  powerongprs();
  delay(300);
  ShowSerialData();
  
  pumpswitch(1);
  jetswitch(1);

  readControls();
  poweronsend();
}

void loop() {
    //задержки для отправки и отрисовки данных
    unsigned long currentMillis = millis();
  if(currentMillis - previousT0Millis >= intervalT0)
  {
    previousT0Millis = currentMillis;
    sendtoserver();  //отправляем данные на сервер
  }
  if(currentMillis - previousT1Millis >= intervalT1)
  {
    previousT1Millis = currentMillis;
    readControls();  //выводим данные на дисплей
  }

  //если авария, то играем сирену
  if (alarmison)
    {
      tone(ALARMSIGNAL,1000);
      delay(300);
      noTone(ALARMSIGNAL);
      tone(ALARMSIGNAL,500);
      delay(300);
    }
  
  ShowSerialData();
  
  //определение нажатых кнопок
  button = getPressedButton();
  switch (button) {
    case BUTTON_RIGHT: 
      //powerongprs();
      alarmison = 0;
      noTone(ALARMSIGNAL);
      break;
    case BUTTON_LEFT: //выкл насос
      pumpswitch(0);
      readControls();
      break;
    case BUTTON_UP: //вкл форсунку
      jetswitch(1);
      readControls();
      break;
    case BUTTON_DOWN: //выкл форсунку
      jetswitch(0);
      readControls();
      break;
    case BUTTON_SELECT: //вкл насос
      pumpswitch(1);
      readControls();
      break;
  }
}

void powerongprs() //включение модуля SIM900
{
  lcd.clear();
  lcd.print("GPRS INIT"); 
  digitalWrite(POWERGPRS,LOW);
  delay(1500);
  digitalWrite(POWERGPRS,HIGH);
  delay(2000);
  digitalWrite(POWERGPRS,LOW);
  delay(10000);
  GPRSSerial.println("AT");       // обязательная запись в модуль SIM900 для установки скорости порта
}

void pumpswitch(int onoff)  //вкл выкл реле насоса
{
  if(onoff)
  {
    Serial.println("PUMPRELAY poweron");
    digitalWrite(PUMPRELAY, LOW);
    delay(300);
  }
  else
  {
    Serial.println("PUMPRELAY poweroff");
    digitalWrite(PUMPRELAY, HIGH);
    delay(300);
  }
}

void jetswitch(int onoff)  //вкл выкл реле форсунки
{
  if(onoff)
  {
    Serial.println("JETRELAY poweron");
    digitalWrite(JETRELAY, LOW);
    delay(300);
  }
  else
  {
    Serial.println("JETRELAY poweroff");
    digitalWrite(JETRELAY, HIGH);
    delay(300);
  }
}

void sendtoserver() //отправка данных на сервер
{
  String tmpstring;
  //формируем строку запроса
  tmpstring = "?module=moduleid4564654654&pump="+String(valpump)+"&jet="+String(valjet)+"&catlevel="+valcat;
  //отправляем по короткой схеме. считаем, что GPRS инициализирован.
  shorthttp(tmpstring);
  //SubmitHttpRequest(tmpstring);
}

void poweronsend() //отправляем сигнал "я включился" на сервер
{
  String tmpstring;
  tmpstring = "?module=moduleid4564654654&pump="+String(valpump)+"&startup=1&jet="+String(valjet)+"&catlevel="+valcat;
  SubmitHttpRequest(tmpstring);
}

void emergencystop(String reason) //отправка сигнала "я сломался" на сервер. в переменной reason можно указать причину
{
  String tmpstring;
  tmpstring = "?module=moduleid4564654654&pump="+String(valpump)+"&jet="+String(valjet)+"&catlevel="+valcat+"&warning="+reason;
  SubmitHttpRequest(tmpstring);
}

void readControls()  //чтение контрольных пинов и вывод на дисплей. 
{
  lcd.clear();
  valcat = analogRead(CATLEVCONTR);
  valpump = digitalRead(PUMPCONTROL);
  valjet = digitalRead(JETCONTROL);
  
  lcdcatval = round(valcat*16/1024);
  if ((valcat<100) and ((valpump<1) or (valjet<1)))  //если мало катализатора, то  
  {
    jetswitch(0); //аварийный останов
    pumpswitch(0);
    alarmison=1;   //и включение сирены
    emergencystop("мало-катализатора");  //отправка сигнала "авария" на сервер с указанием причины
  }
  lcd.setCursor(0, 0);
  for (int i=0; i<=lcdcatval;i++) //вывод уровня катализатора в первую строку дисплея.
    { lcd.write(255);}  //просто заполняем закрашенными квадратами пропорционально значению на пине
  secondline = PUMPstr[valpump] + " " + JETstr[valjet];
  lcd.setCursor(0, 1);
  lcd.print(secondline);
  Serial.println(secondline+" "+valcat);
}

void shorthttp(String msg)  //короткая схема отправки запроса. считаем, что GPRS инициализирован
{
 GPRSSerial.println("AT+HTTPPARA=\"URL\",\""+siteaddress+"/"+actionscript+msg+"\"");// setting the httppara, the second parameter is the website you want to access
 delay(100);
 ShowSerialData();
 GPRSSerial.println("AT+HTTPACTION=0");//submit the request 
 delay(1000);//задержка важна. сервер может думать долго. если зависает, нужно увеличить задержку. ответ сервера надо минимизировать, чтобы не читать много данных.
 ShowSerialData();
}

//длинная схема отправки http запроса
//обязательно выполняется при включении
//здесь инициализируется GPRS
//задержки нужны для правильной инициализации GPRS
void SubmitHttpRequest(String msg)
{
 GPRSSerial.println("AT+CSQ");
 delay(100);
 ShowSerialData();
 GPRSSerial.println("AT+CGATT?");
 delay(100);
 ShowSerialData();
 GPRSSerial.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");//setting the SAPBR, the connection type is using gprs
 delay(100);
 ShowSerialData();
 GPRSSerial.println("AT+SAPBR=3,1,\"APN\",\"internet\"");//установка точки доступа APN. вторым параметром идет адрес сервера APN. для мегафона это internet
 delay(1000);
 ShowSerialData();
 //раскоментировать следующие 2 строки, если часто приходит ошибка 601, 602, 603 или 604
 //GPRSSerial.println("AT+SAPBR=0,1");//setting the SAPBR, for detail you can refer to the AT command mamual
 //delay(2000);
 ShowSerialData();
 GPRSSerial.println("AT+SAPBR=1,1");//setting the SAPBR, for detail you can refer to the AT command mamual
 delay(2000);
 ShowSerialData();
 GPRSSerial.println("AT+HTTPINIT"); //init the HTTP request
 delay(1000); 
 ShowSerialData();
  GPRSSerial.println("AT+HTTPPARA=\"URL\",\""+siteaddress+"/"+actionscript+msg+"\"");// setting the httppara, the second parameter is the website you want to access
 delay(1000);
 ShowSerialData();
 GPRSSerial.println("AT+HTTPACTION=0");//submit the request 
 delay(3000);//the delay is very important, the delay time is base on the return from the website, if the return datas are very large, the time required longer.
 ShowSerialData();
 GPRSSerial.println("AT+HTTPREAD");// read the data from the website you access
 delay(300);
 ShowSerialData();
 GPRSSerial.println("");
 delay(100);
}

void ShowSerialData() //чтение ответа от модуля SIM900. в т.ч. ответа от сервера после http запроса
{
  while(GPRSSerial.available()!=0)
   Serial.write(GPRSSerial.read());
}

int getPressedButton() {  //определяем какая кнопка нажата путем считывания аналогового сигнала. 
	//особенности модуля lcd keypad 16x2
	//кнопки подключены через делитель
  int buttonValue = analogRead(LCDBUTTONSPIN); 
  if (buttonValue < 100) {
    return BUTTON_RIGHT;
  }
  else if (buttonValue < 200) {
    return BUTTON_UP;
  }
  else if (buttonValue < 400) {
    return BUTTON_DOWN;
  }
  else if (buttonValue < 600) {
    return BUTTON_LEFT;
  }
  else if (buttonValue < 800) {
    return BUTTON_SELECT;
  }
  return BUTTON_NONE;
}
