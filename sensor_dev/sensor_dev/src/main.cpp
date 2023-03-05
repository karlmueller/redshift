#include <Arduino.h>
#include "freertos/FreeRTOS.h"


#define HALL_PIN GPIO_NUM_5

#define TOUCH_PIN2 GPIO_NUM_2
#define TOUCH_PIN3 GPIO_NUM_15
#define TOUCH_PIN4 GPIO_NUM_13
#define TOUCH_PIN5 GPIO_NUM_12
#define TOUCH_PIN6 GPIO_NUM_14
#define TOUCH_PIN7 GPIO_NUM_27
#define TOUCH_PIN8 GPIO_NUM_33
#define TOUCH_PIN9 GPIO_NUM_32

void read_cont(void) {

    // Serial.print(digitalRead(HALL_PIN));
    // Serial.print("Hall:: ");
    Serial.println( digitalRead(HALL_PIN));
    // Serial.println(hallRead());
    
    Serial.print("Touch ::  ");
    Serial.print(touchRead(TOUCH_PIN2));
    Serial.print("  ::  ");
    Serial.print(touchRead(TOUCH_PIN3));
    Serial.print("  ::  ");
    Serial.print(touchRead(TOUCH_PIN4));
    Serial.print("  ::  ");
    Serial.print(touchRead(TOUCH_PIN5));
    Serial.print("  ::  ");
    Serial.print(touchRead(TOUCH_PIN6));
    Serial.print("  ::  ");
    Serial.print(touchRead(TOUCH_PIN7));
    Serial.print("  ::  ");
    Serial.print(touchRead(TOUCH_PIN8));
    Serial.print("  ::  ");
    Serial.println(touchRead(TOUCH_PIN9));

}


void setup() {
    gpio_reset_pin(HALL_PIN);
    gpio_set_direction(HALL_PIN, GPIO_MODE_INPUT);

    //pinMode(HALL_PIN, INPUT);
    Serial.begin(115200);
    
    while (1)
    {
        read_cont();
        vTaskDelay(100/portTICK_PERIOD_MS);

    }


}
void loop() 
{


}
