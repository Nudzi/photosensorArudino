#include <Arduino_FreeRTOS.h>

// constants won't change
const int LIGHT_SENSOR_PIN = A0; // Arduino pin connected to light sensor's  pin
const int interruptPin     = 2;  // Arduino pin connected to LED's pin
int LED_PIN          = 5;  // Arduino pin connected to LED's pin
const int ANALOG_THRESHOLD = 500;
bool volatile flagTest = 0;

#include <timers.h>
#include <task.h>
#define mainONE_SHOT_TIMER_PERIOD pdMS_TO_TICKS( 3333 )u
#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( 500 )
//create reference hanlders for one-shot and auto-relaod timers
TimerHandle_t xAutoReloadTimerLight, xAutoReloadTimerDark;
BaseType_t xTimer1Started, xTimer2Started;

void Task1( void *pvParameters );
void Task2( void *pvParameters );
void Taskprint( void *pvParameters );

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT); // set arduino pin to output mode
  pinMode(interruptPin, INPUT_PULLUP); // set arduino pin to output mode
  attachInterrupt(digitalPinToInterrupt(interruptPin), toggle, FALLING);

  xTaskCreate(
    Task1
    , (const portCHAR *)"Task1"
    , 128
    , NULL
    , 1
    , NULL );

  xTaskCreate(
    Task2
    , (const portCHAR *)"Task2"
    , 128
    , NULL
    , 1
    , NULL );

  xAutoReloadTimerLight = xTimerCreate(
                            "Light",
                            mainAUTO_RELOAD_TIMER_PERIOD,
                            pdTRUE,
                            0,
                            prvAutoReloadTimerCallbackLight );

  xAutoReloadTimerDark = xTimerCreate(
                           "Dark",
                           mainAUTO_RELOAD_TIMER_PERIOD,
                           pdTRUE,
                           0,
                           prvAutoReloadTimerCallbackDark );

  if ( ( xAutoReloadTimerLight != NULL && xAutoReloadTimerDark != NULL ) )
  {
    //    BaseType_t xTimerStart( TimerHandle_t xTimer, TickType_t xTicksToWait );
    xTimer1Started = xTimerStart( xAutoReloadTimerLight, 0 );
    xTimer2Started = xTimerStart( xAutoReloadTimerDark, 0 );
    if ( ( xTimer1Started == pdPASS && xTimer2Started == pdPASS ) )
    {
      vTaskStartScheduler();
    }
  }
}

void vDemoFunction( void )
{
  /* This function suspends the scheduler.  When it is called from vTask1 the
    scheduler is already suspended, so this call creates a nesting depth of 2. */
  vTaskSuspendAll();

  if (flagTest) {
    Serial.println("Restarting...");
    xTimerStop(prvAutoReloadTimerCallbackLight, 0);
    xTimerStop(prvAutoReloadTimerCallbackDark, 0);

    xTimer1Started = xTimerStart( xAutoReloadTimerLight, 0 );
    xTimer2Started = xTimerStart( xAutoReloadTimerDark, 0 );

    flagTest = 0;
  }

  /* As calls to vTaskSuspendAll() are nested, resuming the scheduler here will
    not cause the scheduler to re-enter the active state. */
  xTaskResumeAll();
}

static void toggle() {
  Serial.println("Button pressed...");
  flagTest = 1;
}

void loop() {
}

static void prvAutoReloadTimerCallbackLight( TimerHandle_t xTimer )
{
  int analogValue = analogRead(A0);
  if (analogValue > ANALOG_THRESHOLD) {
    TickType_t xTimeNow;
    xTimeNow = xTaskGetTickCount();
    Serial.print("Light timer callback executing ");
    Serial.println(xTimeNow / 31);
  }

}
static void prvAutoReloadTimerCallbackDark( TimerHandle_t xTimer )
{
  int analogValue = analogRead(A0);
  if (analogValue < ANALOG_THRESHOLD) {
    TickType_t xTimeNow;
    xTimeNow = xTaskGetTickCount();
    Serial.print("Drak timer callback executing ");
    Serial.println( xTimeNow / 31 );
  }
}

void Task1(void *pvParamaters) {
  while (1)
  {
    xAutoReloadTimerDark = NULL;
    int analogValue = analogRead(A0);
    if (flagTest == 1) {
      vTaskSuspendAll();
      vDemoFunction();

    }
    if ( xTaskResumeAll() == pdTRUE ) {
      if (analogValue < ANALOG_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH);
      }
    }
  }
}

void Task2(void *pvParamaters) {
  while (1)
  {
    if ( xTaskResumeAll() == pdTRUE ) {
      xAutoReloadTimerLight = NULL;
      int analogValue = analogRead(A0);
      if (analogValue > ANALOG_THRESHOLD) {
        digitalWrite(LED_PIN, LOW);
      }
    }
  }
}
