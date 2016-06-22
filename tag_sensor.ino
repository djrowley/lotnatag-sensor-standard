#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <pins_arduino.h>

// The next four lines contain values which can be modified to balance the sensor for use in games


const byte config_max_hits_upper = 21;
const byte config_max_hits_lower = 1;

const unsigned int deathTonePeriodSecs = 60;
const unsigned int deathBleedoutTimerSecs = 900;// seconds

const unsigned int startupTimerMilliseconds = 5000;




// DO NOT CHANGE VALUES UNDER HERE UNLESS YOU KNOW WHAT YOU ARE DOING.

// pin settings

// INPUT
const int buttonPin = 2;
const int signalPin = 3;

// OUTPUT
const int buzzerPin = 5;
const int redLedPin = 6;

// interrupt settings
const int buttonInt = 0;
const int signalInt = 1;

// hit point settings
const int max_hits_address = 0;

byte current_max_hits = 0;
byte current_hits = 0;

unsigned int secondsUntilDeath;

bool isBleedingOut = false;
bool isDead = false;

unsigned long lastDetectionTime = 0;
int detectionTracker = 0;

unsigned long startupEndTime;
unsigned long timeOfBleedoutStart = 0;
unsigned long timeOfDeath = 0;

bool buttonPressed = false;
bool beenShot = false;
String logMessage = "";

bool deathMinuteWarningSet = false;
int deathMinuteWarningCount = 0;


/*
 * This code runs when the device is first turned on.  It sets up all the pins to the correct states,
 * loads the previous hit point settings from memory and enables the button for 'set hit points'
 */
void setup() {
  Serial.begin(9600);
  Serial.println("Sensor initialisation");
  // set variables
  pinMode(buzzerPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(signalPin, INPUT_PULLUP);
  
  //disable led
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  isBleedingOut = false;
  
  // read current hit counter
  loadMaxHitsFromMemory();
  current_hits = current_max_hits;
  
  logMessage = "Initial max hits (from memory): ";
  logMessage = logMessage + current_hits;
  Serial.println(logMessage);
  
  // flash leds and sound buzzer for number of hits
  reportCurrentHits();
  
  // start timer for system startup
  startupEndTime = millis() + startupTimerMilliseconds;
  buttonPressed = false;
  
  // set interrupt
  EIFR = 1;
  EIFR = 2;
  
  // ENable the button for 'decrease max hit points'
  attachInterrupt(buttonInt, buttonInterrupt, FALLING); // button interrupt should be enabled early
  
  //enter timer - if button is not pressed, will end after startupTimerMilliseconds ms.
  // if button is pressed, resets end of timer time to startupTimerMilliseconds ms after button pressed. 
  do
  {
    EIFR = 1;
    
    bool buttonProcessed = false;
    if (buttonPressed)
    {
      decrementMaxHits();
      reportCurrentHits();
      startupEndTime = millis() + startupTimerMilliseconds;
      buttonProcessed = true;
      logMessage = "Max hits changed to ";
      logMessage = logMessage + current_hits;
      Serial.println(logMessage);
      
    }
    else
    {
      delay(100);
    }
    
    if(buttonProcessed)
      buttonPressed = false;
  } while (millis() < startupEndTime);
  // wait until count expires



  // System active siren
  for (int i=0;i<4;i++)
  {
    digitalWrite(redLedPin, HIGH);
    tone(buzzerPin,1046, 400);
    delay(400);
    digitalWrite(redLedPin, LOW);
    tone(buzzerPin,523, 400);
    delay(400);
  }
  logMessage = "Final hit selection: ";
  logMessage = logMessage + current_hits;
  Serial.println(logMessage);
  saveMaxHitsToMemory();
  Serial.println("Saved to memory");
 
  // set sensor interrupt;
  attachInterrupt(signalInt, signalInterrupt, FALLING); // shot detection interrupt only allowed after setup
  
  
  Serial.println("Shot detection online");
  Serial.println("SENSOR READY");
  Serial.println("------------------------------------------------");
}

void loop() {
  EIFR = 1;
  EIFR = 2;
   
  // wait here until an interrupt occurs
  waitUntilInput();
      
  if(beenShot)
  {
    recordShot();
    beenShot = false;
    
    logMessage = "2khz Pulse Count = ";
    logMessage = logMessage + detectionTracker;
    Serial.println(logMessage);
  } 
  else if(buttonPressed)
  {
    if(isBleedingOut)
    {
      reportBleedoutStatus();
    }
    else
    {
      reportCurrentHits();
    }
    buttonPressed = false;
  }
  
    if (isDead)
      {
        logMessage = "DEAD!";
        Serial.println(logMessage);
        
        detachInterrupt(buttonInt); 

        tone(buzzerPin,432);
        do 
        {
          delay(60000);
        } while(true);

      }
  
  if(isBleedingOut)
  {
    // if in initial death warning period (i.e. first minute of death)
    if(secondsUntilDeath > (deathBleedoutTimerSecs - deathTonePeriodSecs))
    { 
      tone(buzzerPin, 432, 2000);
      delay(2000);
      // tone off
      delay(2000);
    }
    // if we've just hit a minute milestone, report it.
    else if(deathMinuteWarningSet)
    {
      logMessage = "Minutes remaining: ";
      logMessage = logMessage + deathMinuteWarningCount;
      Serial.println(logMessage);
      reportBleedoutStatus();
      deathMinuteWarningSet = false;
    }
  }
}

void buttonInterrupt()
{
  //detachInterrupt(buttonInt); 
  
  buttonPressed = true;
}

void signalInterrupt()
{
  //detachInterrupt(signalInt);

  unsigned long now = micros();
  
  // if within the period to be a shot pulse
  if(lastDetectionTime > (now - 1500))
  {
    // increment count
    detectionTracker ++;
    if(detectionTracker > 60)
    {
      beenShot = true;
    }
  }
  else
  {
    // too far away
    detectionTracker = 0;
  }
  
  lastDetectionTime = now;    
}

void waitUntilInput()
{ 
  set_sleep_mode(SLEEP_MODE_IDLE);           // can't use any other as we want to use falling state for interrupt, can only use IDLE at lower awareness levels
  sleep_enable();                            // allow sleep
  
 
  sleep_cpu();

  // Re-enter here after interrupt
  sleep_disable();                           // unset the flag allowing cpu sleep   
}

void reportCurrentHits()
{
  logMessage = "Current hits: ";
  logMessage = logMessage + current_hits;
  Serial.println(logMessage);
  for(int i = 0;i < current_hits; i++)
  {
      tone(buzzerPin,784, 75);
      digitalWrite(redLedPin, HIGH);
      delay(75);
      digitalWrite(redLedPin, LOW);
      delay(75);
      
      if((i+1)%3 == 0 || i == current_hits - 1)
        delay(150);
  }
}

void reportBleedoutStatus()
{ 
    logMessage = "Death in ";
    logMessage = logMessage + secondsUntilDeath;
    Serial.println(logMessage);
    
    if(secondsUntilDeath <= 30)
    {
      // more urgent warning
      for(int j=0; j<5;j++)
      {
         for(int i =0; i<151;i+=10)
         {
           tone(buzzerPin,432+(10*i),15);
           delay(20);
         }
       }
    }
    else
    {
      int minutesRemaining = (secondsUntilDeath / 60);
      if(secondsUntilDeath % 60 > 0)
        minutesRemaining ++;
      for (int i = 0; i< minutesRemaining; i++)
      {
        tone(buzzerPin, 432, 150);
        delay(300);
              
        if((i+1)%3 == 0 || i == minutesRemaining - 1)
          delay(250);
      }
    }

}
int timer1_counter;
void initialiseDeathTimers()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}



// death interrupt timer
ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;   // preload timer
  digitalWrite(redLedPin, digitalRead(redLedPin) ^ 1);
  
  secondsUntilDeath --;
  
  if(secondsUntilDeath <= 0)
  {
    isDead = true;  
    isBleedingOut = false; 
  }
  
  else if(secondsUntilDeath % 60 == 0)
  {
     // set warning for report
     deathMinuteWarningSet = true;
     deathMinuteWarningCount = (secondsUntilDeath / 60);
  }

  
}

void recordShot()
{
    Serial.println("** HIT RECEIVED **");
    current_hits --;
    
    logMessage = "Hits remaining: ";
    logMessage = logMessage + String(current_hits);

    Serial.println(logMessage);
    digitalWrite(redLedPin, HIGH);
    for(int i =0; i<51;i+=5){
        tone(buzzerPin,500+(50*i),18);
        delay(22);
      }
      for(int i =0; i<51;i+=5){
        tone(buzzerPin,500+(50*i),18);
        delay(22);
      }
      for(int i =0; i<51;i+=5){
        tone(buzzerPin,500+(50*i),18);
        delay(22);
      }
      digitalWrite(redLedPin, LOW);
      
    if(current_hits <= 0)
    {
      // Srt up death systems
      isBleedingOut = true;
      detachInterrupt(signalInt);
      
      secondsUntilDeath = deathBleedoutTimerSecs;
    
      Serial.println(" ****** BLEEDING OUT!  ****** ");
      logMessage = "Death in ";
      logMessage = logMessage + secondsUntilDeath;
      Serial.println(logMessage);
      
      initialiseDeathTimers();
    
      tone(buzzerPin, 432, 5000);
    }
    else
    {
      delay(700); // about 1500 overall taking beeping into account
      EIFR = 2; // ignore shots during this time
    }
}


void loadMaxHitsFromMemory()
{
  // read from EEPROM
  current_max_hits = EEPROM.read(max_hits_address);
  
  // if uninitalised, set to max
  if(current_max_hits < config_max_hits_lower || current_max_hits > config_max_hits_upper)
  {  
      current_max_hits = config_max_hits_upper;
  }
}

void saveMaxHitsToMemory()
{
  // write to permanent
  EEPROM.write(max_hits_address, current_max_hits);
}

void decrementMaxHits (){
    // if not at lower bound, decrement
    if(current_max_hits > config_max_hits_lower)
      {  
        current_max_hits --;
      }
      else
      {
        // or reset to max if already at lower
        current_max_hits = config_max_hits_upper;
      }
      current_hits = current_max_hits;
}


