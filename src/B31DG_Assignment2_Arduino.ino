#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

QueueHandle_t eventQueue;
SemaphoreHandle_t frequencySemaphore;
String stackStr;

#define signalPin 23 // Pin for outputting signal
#define task2Pin 18 // Input pin for Task 2
#define task3Pin 19 // Input pin for Task 3
#define analogPin 4 // Analog input pin
#define errorLedPin 21 // Unique pin for error indication LED
#define buttonPin 15 // Assumed pin for button
#define controlLedPin 16 // Pin for control LED

// Global variables for Task2/3/5
volatile unsigned long lastRiseTimeTask2 = 0;
volatile unsigned long lastRiseTimeTask3 = 0;
volatile int frequencyTask2 = 0;
volatile int frequencyTask3 = 0;
volatile int freqISRtask2 = 0;
volatile int freqISRtask3 = 0;

// To calculate the delay between the time the push button is pressed and the time the LED is toggled
unsigned long buttonPressedTime = 0;
unsigned long ledToggledTime = 0;

void printRemainingStack(int taskID, int remainingStack){
    stackStr = "The minimum remaining stack space of Task " + String(taskID) + " is " + String(remainingStack) + " bytes.";
    Serial.println(stackStr);
}

// Task1
void signalGenerate(void * parameter) {
    // Get the task handle for use with uxTaskGetStackHighWaterMark()
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const unsigned long HighTime1 = 180; // 180 microseconds high
    const unsigned long LowTime1 = 40; // 40 microseconds low
    const unsigned long HighTime2 = 530; // 530 microseconds high
    while(1){
      // Capture the current tick count to use as a reference for timing
      TickType_t xLastWakeTime = xTaskGetTickCount();

      // Set the signal pin to HIGH, then delay for 180 microseconds
      digitalWrite(signalPin, HIGH);
      delayMicroseconds(HighTime1);
      
      // Set the signal pin to LOW, then delay for 40 microseconds
      digitalWrite(signalPin, LOW);
      delayMicroseconds(LowTime1);
      
      // Set the signal pin to HIGH, then delay for 530 microseconds
      digitalWrite(signalPin, HIGH);
      delayMicroseconds(HighTime2);

      // Set the signal pin to LOW
      // then wait until 4 milliseconds have passed since xLastWakeTime
      // to ensure a consistent 4ms cycle regardless of execution time
      digitalWrite(signalPin, LOW);

      // Check the remaining stack space
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
      //printRemainingStack(1, uxHighWaterMark);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
    }
}

// ISR for Task 2
void IRAM_ATTR onRiseTask2() {
    unsigned long currentTime = micros();
    unsigned long pulseWidth = currentTime - lastRiseTimeTask2;
    lastRiseTimeTask2 = currentTime;
    
    // Validate pulse width
    if (pulseWidth >= 1000 && pulseWidth <= 3000) {
        freqISRtask2 = 1000000 / pulseWidth;
    }
}

// ISR for Task 3
void IRAM_ATTR onRiseTask3() {
    unsigned long currentTime = micros();
    unsigned long pulseWidth = currentTime - lastRiseTimeTask3;
    lastRiseTimeTask3 = currentTime;
    
    if (pulseWidth >= 1000 && pulseWidth <= 2000) {
        freqISRtask3 = 1000000 / pulseWidth;
    }
}

// Task function for Task 2
void frequencyMeasureTask2(void * parameter) {
    // Get the task handle for use with uxTaskGetStackHighWaterMark()
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    while(1) {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        if(xSemaphoreTake(frequencySemaphore, portMAX_DELAY)){
            // Attach an interrupt to the task2Pin to handle rising edge signals and measure frequency
            attachInterrupt(digitalPinToInterrupt(task2Pin), onRiseTask2, RISING);
            // Update the shared frequency variable from the ISR's calculated value
            frequencyTask2 = freqISRtask2;
            // Detach the interrupt to prevent further triggers until next cycle
            detachInterrupt(digitalPinToInterrupt(task2Pin));
            // Release the semaphore to allow other tasks to access the frequency variables
            xSemaphoreGive(frequencySemaphore);
        }
        // Check the remaining stack space
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
        //printRemainingStack(2, uxHighWaterMark);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

// Task 3
void frequencyMeasureTask3(void * parameter) {
    // Get the task handle for use with uxTaskGetStackHighWaterMark()
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    while(1) {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        if(xSemaphoreTake(frequencySemaphore, portMAX_DELAY)){
            // Attach an interrupt on task3Pin to capture rising edge and calculate frequency
            attachInterrupt(digitalPinToInterrupt(task3Pin), onRiseTask3, RISING);
            // Safely update the frequency value from ISR
            frequencyTask3 = freqISRtask3;
            // Remove the interrupt handler to reset for the next cycle
            detachInterrupt(digitalPinToInterrupt(task3Pin));
            // Release the semaphore allowing other tasks to use the shared data
            xSemaphoreGive(frequencySemaphore);
        }
        // Check the remaining stack space
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
        //printRemainingStack(3, uxHighWaterMark);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(8));
    }
}

// Task4
void sampleAnalogAndCheckError(void * parameter) {

    int readings[10] = {0}; // Array to store last 10 readings
    int analogValue = 0;
    int readIndex = 0; // Current index in the array
    int total = 0; // Sum of readings
    int average = 0; // Average reading
    int maxAnalogValue = 4095; // Max ADC reading for ESP32

    // Get the task handle for use with uxTaskGetStackHighWaterMark()
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    // Initialize reading array
    for (int thisReading = 0; thisReading < 10; thisReading++) {
        readings[thisReading] = 0;
    }

    while(1) {
        //TickType_t startTime = xTaskGetTickCount();

        TickType_t xLastWakeTime = xTaskGetTickCount();
        // Read from analog input
        analogValue = analogRead(analogPin);
        // Subtract oldest reading from total
        total = total - readings[readIndex];
        // Read new value and add to total
        readings[readIndex] = analogValue;
        total = total + readings[readIndex];
        // Move to next reading position
        readIndex = readIndex + 1;

        // If at end of array, loop back to start
        if (readIndex >= 10) {
            readIndex = 0;
        }

        // Calculate average
        average = total / 10;
        // If average exceeds half max reading, light LED as error indicator
        if (average > maxAnalogValue / 2) {
            digitalWrite(errorLedPin, HIGH);
        } else {
            digitalWrite(errorLedPin, LOW);
        }
        // Check the remaining stack space
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
        //printRemainingStack(4, uxHighWaterMark);

        // Wait until next sampling cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
        //TickType_t endTime = xTaskGetTickCount();
        //unsigned long taskDuration = (unsigned long) endTime - startTime;
        //Serial.printf("Task duration: %u ms.\n", taskDuration);
    }
}

// Scales frequency to a 0-99 range, used in Task5
int scaleFrequency(int measuredFreq, int minFreq, int maxFreq) {
    if (measuredFreq <= minFreq) {
        return 0;
    } else if (measuredFreq >= maxFreq) {
        return 99;
    } else {
        // Scale frequency
        return (measuredFreq - minFreq) * 99 / (maxFreq - minFreq);
    }
}

// Task5
void logFrequency(void * parameter) {
    const int minFreqTask2 = 333; // Minimum frequency for Task 2
    const int minFreqTask3 = 500; // Minimum frequency for Task 3
    const int maxFreq = 1000; // Maximum frequency for Tasks 2 and 3

    // Get the task handle for use with uxTaskGetStackHighWaterMark()
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    while(1) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        // Safely read shared variables
        if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
            // Scale frequencies
            int scaledFreqTask2 = scaleFrequency(frequencyTask2, minFreqTask2, maxFreq);
            int scaledFreqTask3 = scaleFrequency(frequencyTask3, minFreqTask3, maxFreq);

            // Log to serial port
            Serial.printf("%d,%d\n", scaledFreqTask2, scaledFreqTask3);

            // Release semaphore
            xSemaphoreGive(frequencySemaphore);
        }
        // Check the remaining stack space
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
        //printRemainingStack(5, uxHighWaterMark);
        // Delay 200ms before next log
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
    }
}

// Task7_button
void toggleLedWithButton(void * parameter) {
  
    int buttonState = 1; // Current button state
    int lastButtonState = 1; // Previous button state
    unsigned long lastDebounceTime = 0; // Time of last state change
    unsigned long debounceDelay = 50; // Debounce threshold time
    const TickType_t xDelay = pdMS_TO_TICKS(debounceDelay); // Convert debounce delay from milliseconds to ticks
    // Get the task handle for use with uxTaskGetStackHighWaterMark()
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    while(1) {
        int reading = digitalRead(buttonPin); // Read the current state of the button

        // Reset debounce timer if button state changes
        if (reading != lastButtonState) {
            lastDebounceTime = xTaskGetTickCount(); // Capture the current tick count
            //buttonPressedTime = micros();
        }
        
        // Check if state has changed after the debounce period
        if (xTaskGetTickCount() - lastDebounceTime >= xDelay) {
            // Change LED state if button state remains changed after debounce time
            if (reading != buttonState) {
                buttonState = reading; // Update the button state
                if (buttonState == LOW){ // Check if button is pressed
                  int event = 1;
                  xQueueSend(eventQueue, &event, 0); // Send an event to the queue
                }
            }
        }

        lastButtonState = reading; // Update the last button state for next loop iteration
        // Check the remaining stack space
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
        //printRemainingStack(6, uxHighWaterMark);
        // Proper RTOS delay to yield control to other tasks
        vTaskDelay(xDelay);
    }
}

// Task7_led
void ControlLed(void * parameter) {
    // Get the task handle for use with uxTaskGetStackHighWaterMark()
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    int event;
    while(1) {
        // Wait for an event from the queue. Blocks until an event is received.
        if (xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdPASS) {
            // Event received, toggle the LED.
            if(event == 1){
              digitalWrite(controlLedPin, !digitalRead(controlLedPin));
              //ledToggledTime = micros();
              //unsigned long delayTime = ledToggledTime - buttonPressedTime;
              //Serial.printf("Response time: %u us.\n", delayTime);
            }
        }
        // Check the remaining stack space
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
        //printRemainingStack(7, uxHighWaterMark);
    }
}

// Task8
void CPU_work(int time) {
    volatile long iterations_per_millisecond = 80000; // Iterations per ms for 80MHz CPU
    long cycles = time * iterations_per_millisecond;

    for(long i = 0; i < cycles; i++) {
        __asm__("nop"); // No operation, simulates work
    }
}

void periodicCpuWorkTask(void * parameter) {
    // Get the task handle for use with uxTaskGetStackHighWaterMark()
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    while(1) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        // Call CPU_work function every 20ms
        CPU_work(2);
        // Check the remaining stack space
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
        //printRemainingStack(8, uxHighWaterMark);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

void setup() {
    Serial.begin(9600);

    // Initialize pins
    pinMode(signalPin, OUTPUT);
    pinMode(task2Pin, INPUT);
    pinMode(task3Pin, INPUT);
    pinMode(analogPin, INPUT);
    pinMode(errorLedPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(controlLedPin, OUTPUT);

    // Create semaphore and event queue
    frequencySemaphore = xSemaphoreCreateMutex();
    eventQueue = xQueueCreate(10, sizeof(int));

    // Create tasks
    xTaskCreate(signalGenerate, "Task1", 1230, NULL, 5, NULL); //Task1
    xTaskCreate(frequencyMeasureTask2, "Task2", 1650, NULL, 3, NULL); //Task2
    xTaskCreate(frequencyMeasureTask3, "Task3", 1630, NULL, 4, NULL); //Task3
    xTaskCreate(sampleAnalogAndCheckError, "Task4", 1660, NULL, 3, NULL); //Task4
    xTaskCreate(logFrequency, "Task5", 1750, NULL, 1, NULL); //Task5
    xTaskCreate(toggleLedWithButton, "Task7_Button", 1050, NULL, 2, NULL); //Task7_Button
    xTaskCreate(ControlLed, "Task7_LED", 1420, NULL, 2, NULL); //Task7_LED
    xTaskCreate(periodicCpuWorkTask, "Task8", 1520, NULL, 3, NULL); //Task8
}

void loop() {
}
