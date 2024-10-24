// Ce script est le script Arduino d'essai pour le prototypage du TCi EVO Conti.
// Si besoin de contact, le système TCi EVO dans son ensemble et le script actuel a été mis au point par Tino de Meyer et Alec Berbudeau, supervisés par David Barnoin.

// La méthode utilisée pour régler les coefficients du correcteur PID est la méthode de Ziegler-Nichols.
// La méthode utilisée pour régler le mode auto du TCi est la méthode du relai (PID autotuning method).

// La machine de test est une NGX carrossée, équipée d'une EV eau froide de 10 bars et 1,5mm de diamètre.
// Le porte-filtre utilisé pour les tests TCi café est un porte-filtre WBC
// Le porte-filtre utilisé pour les tests TCi thé est un porte-filtre thé proto (Q = 1cl/s environ)

// Remarques:
// Concernant le TCi café, il est réglé avec la sonde en dessous du groupe, au même endroit où l'eau froide arrive. 
// Cependant, il serait préférable de le régler plus proche du porte-filtre car même si la distance est courte, le groupe 
// absorbe énormément de chaleurs. Des plans ont été faits pour rajouter un perçage G1/8 après l'EV café pour brancher l'entrée d'eau froide.

// Concernant le TCi thé, on peut assurer environ 3 cycles à 95°C (6s/15s débit/infusion pour 16g de thé).

//----------------------------------------------------------------------------------------------------------------------------------------------------------

// This script is the Arduino test script for prototyping the TCi EVO Conti.
// For contact, the TCi EVO system as a whole and the current script were developed by Tino de Meyer and Alec Berbudeau, supervised by David Barnoin.

// The method used to tune the PID controller coefficients is the Ziegler-Nichols method.
// The method used to tune the auto mode of the TCi is the relay method (PID autotuning method).


// Autotune library
#include <pidautotuner.h>

// Display library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// Variables used for time
unsigned long Now;
unsigned long Start;
double timeChange;

int i = 0;

// Variables used for autotune mode
PIDAutotuner Session;
int tuningCycles = 8;
int thisCycle = 0;
int lowerOutputRange = 175;
int upperOutputRange = 195;

// Tea variable
int Type = 0;

// Variables used for hot water solenoid valve
int Open;
double Probe;
const int HWValve = A3; // hot water solenoid valve output
double tensionHWValve; // hot water solenoid valve tension measurement

// Variables used for Arduino board
const int modeButton = A4; //trigger mode
const int plusButton = A0; //trigger setpoint+
const int minusButton = A2; //trigger setpoint-
int Mode = 3;
int Compteur;
int compteurConsigne;

// Variables used for Temperature probe
double tensionNTC; //Temp probe tension
double resistanceNTC; //Temp probe resistance
double Temp;
double R100 = 3300.0;
double Beta = 3970.0;
const int NTC = A1;
int R = 1000;

// Variables used for PID
int Te = 100; // Sample time 
unsigned long lastTime;
double Input, Output, Setpoint, FSetpoint; // Input, output, setpoint
double errSom, lastErr; //Sum of the errors, last error
double kp, ki, kd;
double kpf, kif, kdf;
double Kpf, Kif, Kdf;
double Kp, Ki, Kd;
int Command; //Output of the cold water solenoid valve
int FCommand;
int CWValve = 9; // Cold water solenoid valve input on the Arduino board
int FValve = 6;
double Percentage; // Percentage of opening

//---------------------------------------------------------------------------------------------------
// Temp probe and hot water solenoid valve functions

double ProbeCTN(const int NTC) {
  double t = 0;
  tensionNTC = (analogRead(NTC) / 1024.0) * 5.0;
  resistanceNTC = (tensionNTC / (5.0 - tensionNTC)) * R;
  t = 1.0 / ((log(resistanceNTC / R100) / Beta) + (1.0 / (100 + 273.15)));
  t = t - 273.15;
  return t;
}

double ProbeHWValve(const int pin) {
  tensionHWValve = (analogRead(pin) / 1024.0);
  return tensionHWValve;
}

//---------------------------------------------------------------------------------------------------
// Display functions for the 4 modes on the Arduino board
// These functions are only for demonstration use

void displayManual(double Percent,double Temp){
  lcd.setCursor(6, 0);
  lcd.print("Mode Manual");   
  lcd.setCursor(3, 2);     
  lcd.print(int((Temp*10)/1.0)/10.0);
  lcd.setCursor(12, 2); 
  lcd.print("Degrees"); 
  lcd.setCursor(3, 3);     
  lcd.print(int((Percent*10)/1.0)/10.0);
  lcd.setCursor(12, 3);     
  lcd.print("Percents");
}

void displayConsigne(double Setpoint){
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Mode Auto");  
  lcd.setCursor(6, 1);     
  lcd.print("Consigne");
  lcd.setCursor(3, 2);     
  lcd.print(int(Setpoint/1.0));
  lcd.setCursor(12, 2);     
  lcd.print("Degrees");
  delay(1000);
}

void displayAuto(double Command,double Temp){
  lcd.setCursor(6, 0); 
  lcd.print("Mode Auto");    
  lcd.setCursor(3, 2);     
  lcd.print(int((Temp*10)/1.0)/10.0);
  lcd.setCursor(12, 2); 
  lcd.print(("Degrees"));
  lcd.setCursor(3, 3);     
  lcd.print((Command/255.0)*100.0);
  lcd.setCursor(12, 3);     
  lcd.print("Percents");

  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.print(Setpoint+1);
  Serial.print(" ");
  Serial.print(Setpoint-1);
  Serial.print(" ");
  Serial.println(Temp);
}

void displayAutotune(double Command, double Cycle){
  
  lcd.setCursor(4, 0); 
  lcd.print("Mode Autotune");    
  lcd.setCursor(4,1);
  if (Session.getCycle() == 0){
    lcd.print("Demarrer EVC");
  }
  
  if (!Session.isFinished() && Session.getCycle() > 0){
    lcd.print("Reglage- EVF");
  }

  lcd.setCursor(3, 2);     
  lcd.print(Cycle);
  lcd.setCursor(12, 2); 
  lcd.print(("cycles"));
  lcd.setCursor(3, 3);     
  lcd.print((Command/255.0)*100.0);
  lcd.setCursor(12, 3);     
  lcd.print("Percents");

  if (Session.isFinished()){
    lcd.setCursor(4,1);
    lcd.print("Couper-- EVC");
  }
}

void displayTea(double Type, double Temp){
  
  lcd.setCursor(6, 0); 
  lcd.print("Mode The");    
  lcd.setCursor(5,1);

  if (Type == 0){
    lcd.print("Black tea");
    lcd.setCursor(3, 2);     
    lcd.print(90);
    lcd.setCursor(12, 2); 
    lcd.print("Degrees");
    lcd.setCursor(3,3);
  }
  
  else {
    lcd.print("Green tea");
    lcd.setCursor(3, 2);     
    lcd.print(70);
    lcd.setCursor(12, 2); 
    lcd.print("Degrees"); 
    lcd.setCursor(3, 3); 
  }
    
  lcd.print(int((Temp*10)/1.0)/10.0);
  lcd.setCursor(12, 3);     
  lcd.print("Degrees");
}

//---------------------------------------------------------------------------------------------------
// Set functions of auto and autotune modes

void setTunings(double Kp, double Ki, double Kd, double Kpf, double Kif, double Kdf) {
  double TeSec = ((double)Te) / 1000;
  kp = Kp;
  ki = Ki * (TeSec);
  kd = Kd / (TeSec);
  kpf = Kpf;
  kif = Kif * (TeSec);
  kdf = Kdf / (TeSec);
}

void setAutotune() {

  Session.setTargetInputValue(Setpoint);
  Session.setLoopInterval(Te * 1000);
  Session.setOutputRange(lowerOutputRange, upperOutputRange); // It is important to be able to set the output range manually
  Session.setZNMode(PIDAutotuner::ZNModeBasicPID); // Set the basic Ziegler-Nichols method for retrieving Kp, Ki, Kd coefficients
  Session.setTuningCycles(tuningCycles); // Set the number of cycles for the relay algorithm

}

//---------------------------------------------------------------------------------------------------
// On/off functions 

int isTriggered(const int button) {
  return !(int(analogRead(button) / 25));
}

int isOpen(const int pin) {

  Open = 0;
  Probe = ProbeHWValve(pin);

  if (ProbeHWValve(pin) >= 0.99) {
    delay(Te/2);
    if (ProbeHWValve(pin) >= 0.99){
      delay(Te/2);
      if (ProbeHWValve(pin) >= 0.99){
        Open = 1;
      }
    }
  }

  return Open;
}

//---------------------------------------------------------------------------------------------------
// Computing functions for the 3 modes: Manual, Auto, and Autotune
// Manual: A percentage of the cold water solenoid valve opening is set (system of the old TCi, not adjustable)
// Auto: A setpoint temperature is imposed, a PID controller is chosen to maintain the setpoint with coefficients Kp, Ki, Kd.
// Autotune: A percentage of opening is imposed, then another lower percentage, over a set number of cycles. The period and amplitude of oscillation are then recovered (relay method to determine kp, ki, kd)

double computeManual() {

  if (isTriggered(minusButton)) {
    if (Compteur > 20) { Compteur = 20; }
    if (Compteur < 0) { Compteur = 0; } 
    else { Compteur = Compteur - 1; }
  }

  if (isTriggered(plusButton)) {
    if (Compteur > 20) { Compteur = 20; }
    if (Compteur < 0) { Compteur = 0; } 
    else { Compteur = Compteur + 1; }
  }

  if(isOpen(HWValve)){
    Percentage = Compteur * 5.0;
    Command = (Percentage / 100.0) * 255;
    analogWrite(CWValve, Command);
  }

  else {
    Percentage = Compteur * 5.0;
    Command = (Percentage / 100.0) * 255;
    analogWrite(CWValve, 0);
  }

  return Percentage;
}

double computeAuto()  //Programme PID
{

  if (isTriggered(plusButton) && Mode == 1) {
    if (Setpoint < 70) { Setpoint = 70; } 
    else if (Setpoint > 110) { Setpoint = 110; } 
    else { Setpoint = Setpoint - 1; }

    displayConsigne(Setpoint);
  }

  else if (isTriggered(plusButton) && Mode == 3) {
    if (Type == 0) { Type++; } 
    else { Type--; } 

    displayTea(Type, Temp);
  }

  Now = millis();
  timeChange = (Now - lastTime);

  if (timeChange >= Te && isOpen(HWValve)) {
    Input = ProbeCTN(NTC);
    double Error = Setpoint - Input;
    errSom += Error;
    double dErr = (Error - lastErr);

    if ((abs(Setpoint - ProbeCTN(NTC)) > 10)){
     errSom = 0;
    }

    Output = kp * Error + ki * errSom + kd * dErr;
    Command = 185 - Output;

    if (Command > 255) { Command = 255; }
    if (Command < 0) { Command = 0; }

    lastErr = Error;
    lastTime = Now;
    analogWrite(CWValve, Command);

  }

  else {
    Command = 0;
    analogWrite(CWValve, Command);
    lastTime = Now;
    errSom = 0;
  }
  
  return Command;
}


double computeAutotune() {
  if (Session.getCycle() == 0 && isOpen(HWValve)) { // If session hasn't started, start session
    Kp = 0;
    Start = micros();
    Session.startTuningLoop(Start);
  }

  if (!(Session.isFinished()) && isOpen(HWValve)) { // If session is in progress and hot water solenoid valve is open, get temp, get output and write it to the cold water valve
    Now = micros();
    Input = ProbeCTN(NTC);
    Output = Session.tunePID(Input, Now);
    Command = Output;

    thisCycle = Session.getCycle();

    analogWrite(CWValve, Command);
    displayAutotune(Command, thisCycle);

    Serial.println(Input);
    Serial.println(Session.getMax());
    Serial.println(Session.getMin());
    Serial.println("");

  }

  if (Session.isFinished() && isOpen(HWValve)){ // If session is over and hot water solenoid valve is open, set cold water valve output to 0 and display "Stop hot water valve"
    Command = 0;
    analogWrite(CWValve, Command);
    displayAutotune(Command, thisCycle);
  }

  // This portion is used to display the Kp, Ki, Kd coefficients only once, it should not appear in the final product
  if (Session.isFinished() && !isOpen(HWValve) && !Kp){ 
    Kp = Session.getKp();
    Ki = Session.getKi();
    Kd = Session.getKd();

    Serial.println(Session.getKp());
    Serial.println(Session.getKi());
    Serial.println(Session.getKd());
    Serial.println("");
  }

  if (Session.isFinished() && !isOpen(HWValve) && Kp){
    Session.startTuningLoop(Start);
    Kp = 0;
  }
  //

  if (!Session.isFinished() && !isOpen(HWValve)) { // If user has cancelled session too soon, set output and cycle to 0
    thisCycle = 0;
    Command = 0;
    analogWrite(CWValve, Command);
    displayAutotune(Command, thisCycle);
  }

  return Command;
}

double computeFlux(){

  FCommand = 255;

  if (!isOpen(HWValve) && ProbeCTN(NTC) > FSetpoint){
    Input = ProbeCTN(NTC);
    double Error = FSetpoint - Input;
    errSom += Error;
    double dErr = (Error - lastErr);

    Output = kpf * Error + kif * errSom + kdf * dErr;
    FCommand = Output;

    if (errSom > 20){
      errSom = 0;  
    
    }

    if (FCommand > 255) { FCommand = 255; }
    if (FCommand < 0) { FCommand = 0; }

    analogWrite(FValve, 255);
  }

  analogWrite(FValve,FCommand);
  Serial.println(FCommand);

  return FCommand;
}

//---------------------------------------------------------------------------------------------------

void manualMode() {
  Temp = ProbeCTN(NTC);
  Percentage = computeManual();

  displayManual(Percentage, Temp);
}

void autoMode() {
  Setpoint = 88;
  setTunings(Kp, Ki, Kd, Kpf, Kif, Kdf);
  Command = computeAuto();
  Temp = ProbeCTN(NTC);

  displayAuto(Command, Temp);
}

void autotuneMode() {
  Setpoint = 88;
  
  setAutotune();
  Command = computeAutotune();

  displayAutotune(Command, thisCycle);
}

void teaMode(){
  if (Type == 0){ 
    Setpoint = 90; 
    FSetpoint = 80; }
  else { 
    Setpoint = 70; 
    FSetpoint = 80; }

  setTunings(Kp, Ki, Kd, Kpf, Kif, Kdf);
  Command = computeAuto();
  FCommand = computeFlux();
  Temp = ProbeCTN(NTC);

  displayTea(Type, Temp);
}

//---------------------------------------------------------------------------------------------------

void setup() {

  pinMode(minusButton, INPUT_PULLUP);
  pinMode(plusButton, INPUT_PULLUP);
  pinMode(modeButton, INPUT_PULLUP);

  pinMode(CWValve, OUTPUT);

  lcd.init();
  lcd.backlight();

  //Coeffs thé
  Kp = 4.9;
  Ki = 2.4;
  Kd = 2.5;

  //Coeffs TCi café
  //Kp = 3.4;
  //Ki = 1.6;
  //Kd = 1.8;

  // Coeffs flux
  Kpf = 10;
  Kif = 0;
  Kdf = 0;

  Serial.begin(9600);

}

//---------------------------------------------------------------------------------------------------

void loop() {
  
  if (isTriggered(minusButton)) {
    switch (Mode) {
      case 0:
        Mode = Mode + 1;
        lcd.clear();
        delay(1000);
        break;
      case 1:
        Mode = Mode + 1;
        lcd.clear();
        delay(1000);
        break;
      case 2:
        Mode = Mode + 1;
        lcd.clear();
        delay(1000);
        break;
      case 3:
        Mode = 0;
        lcd.clear();
        delay(1000);
        break;
    }
  }

  if (Mode == 0) {
    manualMode();
    delay(Te);
  }

  else if (Mode == 1) {
    autoMode();
    delay(Te);
  }

  else if (Mode == 2) {
    autotuneMode();
    delay(Te);
  }

  else if (Mode == 3) {
    teaMode();
    delay(Te);
  }
}
