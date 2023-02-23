#include <esp_now.h>
#include <WiFi.h>

#include <Arduino.h>

#include <stdio.h>
#include <iostream>
#include "AccelStepper.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <queue>
#include <stack>

//#include <boost>
//#include <boost/circular_buffer.hpp>

//#include "cooling.h" //to be devveloped
//#include "touch.h" //to be developed`
//#incude ""

//definition of the pin and static configuration values
#define STEP_GPIO GPIO_NUM_4
#define MOTOR_ENABLE GPIO_NUM_21 
#define RST GPIO_NUM_22
#define SLP GPIO_NUM_16
#define DIR_PIN GPIO_NUM_0
#define stick_threshold 90

const float MAX_ACC = 4500;
const float MAX_SPEED = 17500;

static int stepper_direction;
static float speed_normalized;
bool esp_now_verbose = false;

//static std::stack<float> q;
//static int q;  
static float c_command_val;
static float abs_stick;



//static boost::circular_buffer<float> cq(2);

//what was this even for? to send multiple or scale the step rate if microstepping?

static const int step_normalize(int input_steps, int m0, int m1, int m2) {
  //define stepping combinations
  int step_factor;
  if (m0 == 0 && m1 ==0 && m2 == 0) {
    step_factor = 1; //no microstepping, scaling to 1
  }
  else if (m0 == 1 && m1 == 0 && m2 == 0) {
    step_factor = 2; //I think this is half microstepping but not entirely sure
  }
  else if (m0 == 1 && m1 == 1 && m2 == 0) {
    step_factor = 4; //I think this is 1/4 microstepping but not entirely sure
  }
  else if (m0 == 1 && m1 == 1 && m2 == 1) {
    step_factor = 32; //I think this is 1/32 microstepping but not entirely sure
  }
  else{
    printf("Invalid mmicrostepping parameters");
    step_factor = 0;
  }

  return step_factor;
}

static AccelStepper stepper0(AccelStepper::DRIVER);

// Structure example to receive data
// Must match the sender structure
typedef struct message_object
  {
    int xStick_left = 0;
    int yStick_left = 0;

    int r_trigger = 0;
    int r_bumper = 0;

    int l_trigger = 0;
    int l_bumper = 0;

  } struct_sendmsg;

struct_sendmsg myData;



// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  
  if(esp_now_verbose == true){
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("X Axis: ");
    Serial.println(myData.xStick_left);
    Serial.print("Y Axis: ");
    Serial.println(myData.yStick_left);
  }

  c_command_val = myData.xStick_left;
  //q.push(myData.xStick_left);

  //cq.push_back()
  //vTaskDelay(150 / portTICK_PERIOD_MS);
}

void stepper_initialization() {

  gpio_reset_pin(STEP_GPIO);
  gpio_reset_pin(MOTOR_ENABLE);
  gpio_reset_pin(RST);
  gpio_reset_pin(SLP);
  gpio_reset_pin(DIR_PIN);
  gpio_set_direction(STEP_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR_ENABLE, GPIO_MODE_OUTPUT);
  gpio_set_direction(RST, GPIO_MODE_OUTPUT);
  gpio_set_direction(SLP, GPIO_MODE_OUTPUT);
  gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

  gpio_set_level(MOTOR_ENABLE, 0);
  gpio_set_level(RST, 1);
  gpio_set_level(SLP, 1);

  stepper0.setAcceleration(MAX_ACC);
  stepper0.setMaxSpeed(MAX_SPEED);
  Serial.println("Stepper <index> Initialized");
}


int dir_check()
{
  if (c_command_val < 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }

}

void command_speed()
{
  Serial.println("Waking motor");
  stepper0.enableOutputs(); //begin the motor to resume use

  while (abs(c_command_val) > stick_threshold)
  {
    stepper_direction = dir_check();
    stepper0._direction = stepper_direction; //assign direction to stepper, queried via object memory, not within a step function
    speed_normalized = ((abs(c_command_val) - stick_threshold) / (512.0 - stick_threshold)) * MAX_SPEED;
    

    //Serial.print("ESP_raw: ");
    //Serial.println(c_command_val);
    //Serial.print("Normalized_speed: ");
    //Serial.println(speed_normalized);
    stepper0.setSpeed(speed_normalized);
    stepper0.runSpeed();
    //vTaskDelay(150 / portTICK_PERIOD_MS);
  }
  // Disable the motor once inputs are not 
  stepper0.setSpeed(0); //stop the motor
  stepper0.runSpeed(); //run the motor at zero speed
  stepper0.disableOutputs(); //cut motor power after stop to save power, heat
  //Serial.println("Motor stopped due to stick inactivity");
}


void motor_control_loop() {
  if(&myData !=  NULL)
    {
      //unsure of absolute directionality, determine the actual directionality and include in notation here

      if (abs(c_command_val) > stick_threshold)
      {
        command_speed();
      }

      // else do nothing, wait for the next polling period to check for a change in commanded speeds / directions

    } //TERMINATE if(&myData !=  NULL)

    //vTaskDelay(150 / portTICK_PERIOD_MS); //polling period for the motor speeds. Should provide for some debounce. Ensure this is nonblocking
    //must use a task delay, to do anything else is frankly stupid (or I am for thinking so???)... wondering if this will cause callback issues if a blocking wait function
}



void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW, must be done after wifi init
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else {
    Serial.println("SUCCESS initializing ESP-NOW");
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  stepper_initialization();

  // >>>>>>>>>>>>>>>>> MAKE TASK PINNED TO CORE WORK PROPERLY, HANDLE ON THE NON-WIRELESS CORE   <<<<<<<<<<<<<<
  //xTaskCreatePinnedToCore(motor_control_loop(), "main_loop", 10000, 1, 1, xCoreID=2);

  //xTaskCreateStaticPinnedToCore(); figure out how to make this task properly... also, which core? which one normally handles the wireless??? core0?
}

//code below may be problematic, may need to set a normal operation thread for one of these and specifically set the
// stepper execution functions to their own thread


void loop()
{
  motor_control_loop(); // begin the motor task
  //no contents, control loop handled in setup
  // DO not remove this loop, it will break the program
  
}