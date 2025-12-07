
#define relay1 24
#define relay2 26
#define relay3 28
#define relay4 30

#define wl_run1 5 
#define wl_run2 4

#define wl_run3 3
#define wl_run4 2

int speed = 210 ;

void setup() {
  // put your setup code here, to run once:

  pinMode (relay1,OUTPUT);
  pinMode (relay2,OUTPUT);
  pinMode (relay3,OUTPUT);
  pinMode (relay4,OUTPUT);

  pinMode(wl_run1 , OUTPUT );
  pinMode(wl_run2 , OUTPUT );

  pinMode(wl_run3 , OUTPUT );
  pinMode(wl_run4 , OUTPUT );

digitalWrite(relay1,HIGH);
digitalWrite(relay2,HIGH);
digitalWrite(relay3,HIGH);
digitalWrite(relay4,HIGH);

}

 int a = 0 ;
void loop() 
{
  // put your main code here, to run repeatedly:

digitalWrite(relay1,HIGH); // relay 24 
digitalWrite(relay2,HIGH);  //relay26
/*digitalWrite(relay2,LOW);
digitalWrite(relay3,LOW); 
digitalWrite(relay2,LOW); */

if (a == 0)
{
  a = 1 ; 
analogWrite(wl_run1 ,speed);  //wheel num1
//analogWrite(wl_run2 ,speed);  //wheel num2
//analogWrite(wl_run3 ,speed);  //wheel num3
analogWrite(wl_run4 ,speed);  //wheel num4

delay (6000 );


digitalWrite(relay1,LOW); // relay 24 
digitalWrite(relay2,LOW);  //relay26


analogWrite(wl_run1 ,speed);  //wheel num1
//analogWrite(wl_run2 ,speed);  //wheel num2
//analogWrite(wl_run3 ,speed);  //wheel num3
analogWrite(wl_run4 ,speed);  //wheel num4

delay(1000);

analogWrite(wl_run1 ,0);  //wheel num1
//analogWrite(wl_run2 ,speed);  //wheel num2
//analogWrite(wl_run3 ,speed);  //wheel num3
analogWrite(wl_run4 ,0);  //wheel num4



}




}
