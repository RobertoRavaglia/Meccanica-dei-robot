#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <math.h>

//Mappatura Pin
#define step1 2
#define step2 3
#define step3 4
#define step4 12
#define dir1 5
#define dir2 6
#define dir3 7
#define dir4 13
#define EndStop1 9
#define EndStop2 10
#define EndStop3 11
#define EndStop4 A3
Servo servo;                          //questa e le due righe setto servono per fare la configurazione del servo
const byte servoPin = A0; // abort
int pos = 0;

//Nota: enable sempre attivo con ponte su CNC shield
/*#define Enable1 8 
  #define Enable2 8
  #define Enable3 8
  #define Enable4 8
*/

//Associamo variabili agli stepper
AccelStepper motor1(1, step1, dir1);                  //1 means an external stepper driver with step and direction pins
AccelStepper motor2(1, step2, dir2);
AccelStepper motor3(1, step3, dir3);
AccelStepper motor4(1, step4, dir4);              

//Definizione parametri SCARA
double h=30;                                          //altezza minima rispetto al piano --> ricalcolare con nuovo progetto

int homingok=0;
int calcok=0;
int postogo[8]= {0,1,2,3,3,2,1,0};

float yarray[4] = {325,      //y   pos.1
                  95,        //y    pos.2
                  150,     //.... 
                  220};
float xarray[4] = {0,
                  -205,
                  130,
                  -115};
float beta[5] = {0,    //il primo valore è la memoria di azzeramento
                        0,
                        0,
                        0,
                        0};    // theta3,angolo EE, definito nullo quando camma di azzeramento è in asse con l'asse dell'avambraccio, positivo per angoli, negativo per angoli 
float zarray[4] = {0,
                    50,
                    0,
                    25};

float theta1array[5];
float theta2array[5];
float theta3array[5];

float theta1steps[4];
float theta2steps[4];
float theta3steps[4];
float theta4steps[4];
                      
void setup()
{
//DEFINIZIONE PARAMETRI SENSORI E MOTORI
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(EndStop1, INPUT_PULLUP);
  pinMode(EndStop2, INPUT_PULLUP);
  pinMode(EndStop3, INPUT_PULLUP);
  pinMode(EndStop4, INPUT_PULLUP);

  //motor1.setPinsInverted(false,false,false);
  motor1.setAcceleration(700);
  motor1.setMaxSpeed(1000);
  //motor1.setSpeed(100);
  motor1.enableOutputs();

  //motor2.setPinsInverted(false,false,false);
  motor2.setAcceleration(1300);
  motor2.setMaxSpeed(1000);
  //motor2.setSpeed(100);
  motor2.enableOutputs();

  //motor3.setPinsInverted(false,false,false);
  motor3.setAcceleration(200);
  motor3.setMaxSpeed(1000);
  //motor3.setSpeed(100);
  motor3.enableOutputs();

  //motor4.setPinsInverted(false,false,false);
  motor4.setAcceleration(200);
  motor4.setMaxSpeed(1000);
  //motor4.setSpeed(100);
  motor4.enableOutputs();

  servo.attach(servoPin);  


}

void loop()
{
  // put your main code here, to run repeatedly:
  
//HOMING
  if (homingok==0)
  {
//homing motor2 (salita/discesa)
    if (digitalRead(EndStop2)==1)                   //1 -> sono già sul proximity
    {
      while (digitalRead(EndStop2)==1)              //libero proximity
      {
        motor2.setSpeed(200);
        motor2.runSpeed();
        delay(50);
      }
    }

    while (digitalRead(EndStop2)==0)              //!= significa DIVERSO, mi muovo finchè non impegno il proximity
    {
      motor2.setSpeed(-400);
      motor2.runSpeed();
    }
    motor2.setCurrentPosition(-2200);           //quando arrivo su endstop setto step attuali
    delay(20);                                    //metto programma in pausa per 20 ms
    
    motor2.moveTo(-2000);                         //setta il target
    motor2.runToPosition();                       //raggiunge il target con velocità e accelerazione fissate
//homing motor1 (spalla) 
    if (digitalRead(EndStop1)==1)                   //1 -> sono già sul proximity
    {
      while (digitalRead(EndStop1)==1)              //libero proximity
      {
      motor1.setSpeed(-100);
      motor1.runSpeed();
      delay(50);
      }
    }

    while (digitalRead(EndStop1)==0)              //!= significa DIVERSO, mi muovo finchè non impegno il proximity
    {
      motor1.setSpeed(200);
      motor1.runSpeed();
    }
    motor1.setCurrentPosition(1160);           //quando arrivo su endstop setto step attuali
    delay(20);                                    //metto programma in pausa per 20 ms

    motor1.moveTo(0);                         //setta il target
    motor1.runToPosition();                       //raggiunge il target con velocità e accelerazione fissate


  
//homing motor4/1 (rotazione EE)
    if (digitalRead(EndStop4)==1)                   //0 -> sono già sul proximity
    {
      while (digitalRead(EndStop4)==1)              //libero proximity
      {
        motor4.setSpeed(-50);
        motor4.runSpeed();
        delay(50);
      }
    }

      while (digitalRead(EndStop4)==0)              //!= significa DIVERSO, mi muovo finchè non impegno il proximity
    {
      motor4.setSpeed(100);
      motor4.runSpeed();
    }
    motor4.setCurrentPosition(165);           //quando arrivo su endstop setto step attuali
    delay(20);                                    //metto programma in pausa per 20 ms

    motor4.moveTo(140);                         //setta il target
    motor4.runToPosition();                       //raggiunge il target con velocità e accelerazione fissate

//homing motor3 (gomito)
    
    if (digitalRead(EndStop3)==1)                   //0 -> sono già sul proximity
    {
      while (digitalRead(EndStop3)==1)              //libero proximity
      {
        motor3.setSpeed(50);
        motor3.runSpeed();
        delay(50);
      } 
    }

    while (digitalRead(EndStop3)==0)              //!= significa DIVERSO, mi muovo finchè non impegno il proximity
    {
      motor3.setSpeed(-100);
      motor3.runSpeed();
    }
    motor3.setCurrentPosition(-50);           //quando arrivo su endstop setto step attuali
    delay(20);                                    //metto programma in pausa per 20 ms
   
    motor4.moveTo(0);
    motor4.runToPosition();
    motor3.moveTo(100);                         //setta il target
    motor3.runToPosition();                       //raggiunge il target con velocità e accelerazione fissate

//homing motor4/2 (rotazione EE)
    motor1.moveTo(600);                         //setta il target
    motor1.runToPosition();     

    if (digitalRead(EndStop4)==1)                   //0 -> sono già sul proximity
    {
      while (digitalRead(EndStop4)==1)              //libero proximity
      {
        motor4.setSpeed(-50);
        motor4.runSpeed();
        delay(50);
      }
    }

      while (digitalRead(EndStop4)==0)              //!= significa DIVERSO, mi muovo finchè non impegno il proximity
    {
      motor4.setSpeed(100);
      motor4.runSpeed();
    }
    motor4.setCurrentPosition(165);           //quando arrivo su endstop setto step attuali
    delay(20);                                    //metto programma in pausa per 20 ms

    motor4.moveTo(0);                         //setta il target
    motor4.runToPosition();                       //raggiunge il target con velocità e accelerazione fissate

//Test Gripper
 /*
    for (pos = 0; pos <= 90; pos += 1)  // goes from 0 degrees to 180 degrees in steps of 1 degree
    {
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    */
    for (pos = 45; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }

//Print
    homingok=1; 
    Serial.print('\n');
    Serial.print("HOMING COMPLETATO, homingok=");
    Serial.print(homingok);
  }
    


//CINEMATICA INVERSA
  if (calcok==0)
  {
  // Dichiarazione delle variabili
  float theta1, theta2, theta3, theta4, theta11, theta22;
  float b = 180;  // Lunghezza del braccio 1 in mm
  float a = 145;  // Lunghezza del braccio 2 in mm
  double d, delta, alpha, gamma;
  theta1array[0] = 0;
  theta2array[0] = 0;
  theta3array[0] = 0;

    for (int i=0; i<4; i++)
    {
      d=sqrt(pow(xarray[i],2)+pow(yarray[i],2));
      
      delta=atan2(yarray[i],xarray[i])*(180/PI);
      
      alpha=acos((pow(b,2)+pow(d,2)-pow(a,2))/(2*b*d))*(180/PI);
      
      gamma=acos((pow(a,2)+pow(b,2)-pow(d,2))/(2*a*b))*(180/PI);


      if (xarray[i]>0)
      {
        theta1=delta+alpha;
        theta2=270-gamma;
        
      }
      else
      {
        theta1=delta-alpha;
        theta2=gamma-90;
      }

      theta3=-theta1+90;

      Serial.print('\n');
      Serial.print(theta1);
      Serial.print('\n');
      Serial.print(theta2);
      Serial.print('\n');
      Serial.print(theta3);
      if (theta1>=180)
        {
          Serial.print('\n');
          Serial.print("Theta1 non raggiungibile");
        }
      if (theta2<=-30 || theta2>=199)
        {
          Serial.print('\n');
          Serial.print("Theta2 non raggiungibile");
        }

      theta1array[i+1]=theta1;
      theta2array[i+1]=theta2;
      theta3array[i+1]=theta3;
    }
     
  // Trasformo angoli in step
    for (int l=0; l<4; l++)
    {
      theta1steps[l]=theta1array[l+1]/0.15;
      theta2steps[l]=theta2array[l+1]/0.9;
      theta3steps[l]=theta3array[l+1]/0.9;
      theta4steps[l]=zarray[l]/0.225;
    }
      calcok=1;
  }


//Ciclo di lavoro
  motor1.setSpeed(600);
  motor3.setSpeed(600);
  motor4.setSpeed(600);
  motor2.setSpeed(600);
  motor1.setAcceleration(1000);
  motor3.setAcceleration(500);
  motor4.setAcceleration(500);
  motor2.setAcceleration(2500);
  for (int p=0; p<=7; p++)
  {
    //raggiungo posizione 
    motor1.moveTo(theta1steps[postogo[p]]);
    motor1.runToPosition();
    
    motor3.moveTo(theta2steps[postogo[p]]);
    motor3.runToPosition();
   
    motor4.moveTo(theta3steps[postogo[p]]);
    motor4.runToPosition();
    
    motor2.moveTo(theta4steps[postogo[p]]);
    motor2.runToPosition();

    if (p % 2==0)
    {
      for (pos = 0; pos <= 90; pos += 1)  // goes from 0 degrees to 180 degrees in steps of 1 degree
      {
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else
    {
      for (pos = 90; pos >= 0; pos -= 1)  // goes from 180 degrees to 0 degrees
      {  
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    
    motor2.moveTo(-600);
    motor2.runToPosition();
    Serial.print("\n");
    Serial.print("Posizione ");
    Serial.print(p);
    Serial.print(" raggiunta");
  }
  /*
    //raggiungo posizione 1 PRELIEVO
      motor1.moveTo(theta1steps[0]);
      motor1.runToPosition();
      motor3.moveTo(theta2steps[0]);
      motor3.runToPosition();
      motor4.moveTo(theta4steps[0]);
      motor4.runToPosition();
      motor2.moveTo(theta3steps[0]);
      motor2.runToPosition();

      //a questo punto devo inserire comando di chiusura pinza
      for (pos = 0; pos <= 180; pos += 1)  // goes from 0 degrees to 180 degrees in steps of 1 degree
      {
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }

      //per evitare di aggiungere una posizione inserisco comando per alzare asse Z
      delay(1000);
      motor2.moveTo(-200);
      motor2.runToPosition();
      delay(2000);

    //raggiungo posizione 2 DEPOSITO
      motor1.moveTo(theta1steps[1]);
      motor1.runToPosition();
      motor3.moveTo(theta2steps[1]);
      motor3.runToPosition();
      motor4.moveTo(theta4steps[1]);
      motor4.runToPosition();
      motor2.moveTo(theta3steps[1]);
      motor2.runToPosition();
      
      //a questo punto devo inserire comando di aprire pinza
      for (pos = 180; pos >= 0; pos -= 1)  // goes from 180 degrees to 0 degrees
      {  
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }

      //per evitare di aggiungere una posizione inserisco comando per alzare asse Z
      delay(1000);
      motor2.moveTo(-200);
      motor2.runToPosition();
      delay(2000);

    //raggiungo posizione 3 PRELIEVO
      motor1.moveTo(theta1steps[2]);
      motor1.runToPosition();
      motor3.moveTo(theta2steps[2]);
      motor3.runToPosition();
      motor4.moveTo(theta4steps[2]);
      motor4.runToPosition();
      motor2.moveTo(theta3steps[2]);
      motor2.runToPosition();
      
      //a questo punto devo inserire comando di chiusura pinza
      for (pos = 0; pos <= 180; pos += 1)  // goes from 0 degrees to 180 degrees in steps of 1 degree
      {
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }

      //per evitare di aggiungere una posizione inserisco comando per alzare asse Z
      delay(1000);
      motor2.moveTo(-200);
      motor2.runToPosition();
      delay(2000);

    //raggiungo posizione 4
      motor1.moveTo(theta1steps[3]);
      motor1.runToPosition();
      motor3.moveTo(theta2steps[3]);
      motor3.runToPosition();
      motor4.moveTo(theta4steps[3]);
      motor4.runToPosition();
      motor2.moveTo(theta3steps[3]);
      motor2.runToPosition();
      //a questo punto devo inserire comando di chiusura pinza

      //per evitare di aggiungere una posizione inserisco comando per alzare asse Z
    delay(1000);
      motor2.moveTo(-200);
      motor2.runToPosition();
    delay(2000);*/




}



