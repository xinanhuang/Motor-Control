// motor operate by finding the shortest directional path to return to target position (acu_pos)
// not idea for legged system ( always rotate back the way you came), or if the motor is gear down (need to account for the output shaft rotation)

#include <MagAlpha.h>
#include <ESP32Encoder.h>


#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          2             //SPI CS pin



MagAlpha magAlpha;
ESP32Encoder encoder;
ESP32Encoder encoder2;


const byte interruptPin = 25;
const int ledPin = 14;  




// for testing
unsigned int timebb = 0;
unsigned int timeee = 0;


// for recording time between each velocity loop ( current set to 50us, or 20khz)
unsigned long timeb = 0;
unsigned long timee = 0;
long time_diff = 0;

int angle = 0;      // current angle (raw, 0 - 1024)
int angle_prev = 0; // previous angle
int vel  = 0;       // velocity error

int angle_dif =0;   // current angle value to target angle value 
int err = 0;        // pd error total, write to analogwrite()


// setting PWM properties
const int freq = 25000;
const int ledChannel = 0;
const int resolution = 12;  // up to 4095 pwm vlaue


// pid value p - position, d - velocity
int multipler = 5;
int p = 10*multipler;
int i_m = 2;
int i_d = 100;
int d = 10000*multipler;

int err_i = 0;


int max_val = 4095;    // max pwm on vlaue



// we use overflow characteristic for circular value in the future
// overflow solves everything
// controll will be simpler when its geared down and finding the shortest rout back isn't necessary

int acu_pos = 951;      // store actuator position
int upper_acu_pos = 0;  // upper bound (acu_pos + 1), overflow to 0 when its 1024
int lower_acu_pos = 0;  // lower bound (acu_pos - 1), underflow to 1023 when its -1

int base_param = 0;   // lowest pwm on value (up to max bit value)



bool wait_for_idx = true;  // wait until human operated opitcal encoder is calibrated (generating index pulse)



//// index pin interrupt to reset human operated optical encoder ////
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  if (encoder.getCount() > 500)
  {
    encoder.setCount(0);
  }
  else
  {
    encoder.setCount(1999);
  }
  wait_for_idx = false;
  portEXIT_CRITICAL_ISR(&mux);
}




//// used to update target position (acu_pos is a global variable to no need to pass in and return)
void position_update()
{
  acu_pos = (int32_t)encoder.getCount()*1024/2000;
  
  upper_acu_pos = acu_pos + 1;
  lower_acu_pos = acu_pos - 1;

  if (upper_acu_pos > 1023)
  {
    upper_acu_pos = upper_acu_pos - 1023;
  }

  if (lower_acu_pos < 0)
  {
    lower_acu_pos = lower_acu_pos + 1023;
  }

}



//// arduino ide setup loop
void setup() {

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 780); 

  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  digitalWrite(27, LOW);
  digitalWrite(26, LOW);

  //Serial.begin(UART_BAUDRATE);


    
  // set up ma702 embedded in the dc motor (ma 730 would be ideal)
  magAlpha.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, SPI_CS_PIN);

  // set up opticlencoder
  ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachHalfQuad(17, 16);
  encoder.setCount(0);   // set a random starting count value after attaching
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING); // detect falling edges 


  while(wait_for_idx == true)
  {
     position_update();
  }
  
  
  angle= magAlpha.readAngleRaw16()>>6;  // read ma702 output and ditch the least signifcant 6 bit. ditch least significant 4 bit for ma730 (ideally)
  timeb = micros();
  angle_prev = angle;

  
}





void loop() {

   timee = micros();
   timeee = millis();


//    time_diff  = timee-timeb;
//   if (timeee - timebb > 1)
//   {
//       acu_pos += 1;
//       if (acu_pos >1024)
//       {
//        acu_pos = acu_pos-1024;
//       }
//       position_update();
//       timebb = timeee;
//
//   }

  
    //Read the angle (10-bit raw angle value)
    position_update();
    angle= magAlpha.readAngleRaw16()>>6;
    time_diff  = timee-timeb;


   // if greater tahn 50 microseconds, update velocity
   if (time_diff > 50)
   {
       // might wanna add an averging filter
       timeb = timee;
       vel = (angle - angle_prev)*d/time_diff;
       if (vel > max_val)
       {
          vel = max_val;
       }
    
//        Serial.print("err:");
//        Serial.print(err);
//        Serial.print(" vel:");
//        Serial.println(vel);
       
        angle_prev = angle;
   }

   

    // if target position is between (512-1023)
    if ((acu_pos <= 1023) && (acu_pos>= 512))
    {
       angle_dif = acu_pos - angle;         

        // stationary
        if (angle ==  acu_pos)
        {
//          digitalWrite(26, LOW);
//          digitalWrite(27, LOW);
//          ledcWrite(ledChannel, 0);   
            
        }



        else{
              // clockwise
             if (angle_dif >0 && angle_dif < 512)
             {
                err_i += angle_dif-1;
                err = (angle_dif-1)*p-vel+err_i*i_m/i_d;
             }
      
      
             // counter clockwise
             else if (angle_dif >512 )
             {
                err_i += angle_dif-1022;
                err = (angle_dif-1022)*p-vel+err_i*i_m/i_d;
             }
             else if (angle_dif < 0)
             {
                err_i += angle_dif+1;
                err = (angle_dif+1)*p-vel+err_i*i_m/i_d;
             }

             
             // update pwm on time
             if (err > 0)
             {
                if (err >= max_val-1)
                {
                  err = max_val;
                }
                digitalWrite(26, LOW);
                digitalWrite(27, HIGH);
                ledcWrite(ledChannel, err+base_param);   // take out the backlash operation
             }

             else if (err < 0)
             {
                if (err <= -max_val+1)
                {
                  err = -max_val;
                }
                digitalWrite(27, LOW);
                digitalWrite(26, HIGH);
                ledcWrite(ledChannel, base_param-err);   // take out the backlash operation
      
             }
        
         }

    }








    // if target position is between (0-511)
    else if ((acu_pos >= 0) && (acu_pos< 512))
    {


       angle_dif = angle - acu_pos;  
        // stationary
        if (angle >=  lower_acu_pos && angle <= upper_acu_pos)
        {
//          digitalWrite(26, LOW);
//          digitalWrite(27, LOW);
//          ledcWrite(ledChannel, 0);   
        }



        else{
          
              // clockwise
             if (angle_dif >0 && angle_dif < 512)
             {
                err_i += angle_dif-1;
                err = (angle_dif-1)*p+vel+err_i*i_m/i_d;             
             }
      
      
             // counter clockwise
             else if (angle_dif >512 )
             {
                err_i += angle_dif-1022;
                err = (angle_dif-1022)*p+vel+err_i*i_m/i_d;
             }
             else if (angle_dif < 0)
             {
                err_i += angle_dif+1;
                err = (angle_dif+1)*p+vel+err_i*i_m/i_d;
             }


             // update pwm on time
             if (err > 0)
             {
                if (err >= max_val-1)
                {
                  err = max_val;
                }
                digitalWrite(27, LOW);
                digitalWrite(26, HIGH);
                ledcWrite(ledChannel, err+base_param);   // take out the backlash operation
             }

             else if (err < 0)
             {       
                if (err <= -max_val+1)
                {
                  err = -max_val;
                }  
                digitalWrite(26, LOW);
                digitalWrite(27, HIGH);
                ledcWrite(ledChannel, base_param-err);   // take out the backlash operation
      
             }
        
         }
         


//          Serial.print("angle: ");
//          Serial.print(angle);
//          Serial.print(", dif: ");
//          Serial.print(angle_dif);
//          Serial.print(", pwm_duty: ");
//          Serial.println( err*err*p/p_d);
  
 
    }

    if (err_i >= max_val-1)
    {
      err_i = max_val;
    }
    else if (err_i <= -max_val+1)
    {
      err_i = -max_val;
    }
    
  


}
