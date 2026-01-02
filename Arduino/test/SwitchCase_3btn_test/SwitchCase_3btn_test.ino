const int BTN_PIN1 = 2;   // CASE
const int BTN_PIN2 = 3;   // UP
const int BTN_PIN3 = 4;   // DOWN
const int LED_PIN1 = 9;  // LED1
const int LED_PIN2 = 10;  // LED2
const int LED_PIN3 = 11;  // LED3

int led1_value = 100;
int led2_value = 100;
int led3_value = 100;

int case_selected = 0;
bool btn1_pressed_last = false;
bool btn1_pressed = false;
bool btn1_positive_flank = false;

bool btn2_pressed_last = false;
bool btn2_pressed = false;
bool btn2_positive_flank = false;

bool btn3_pressed_last = false;
bool btn3_pressed = false;
bool btn3_positive_flank = false;

int increaseValue(int Value){
  Value = Value + 25;
  Value = min(255, Value);
  return Value;
}

int decreaseValue(int Value){
  Value = Value - 25;
  Value = max(0, Value);
  return Value;
}

void LED_CTRL(int PIN, int Value){
  analogWrite(PIN, Value);
}

bool detectPositiveFlank(bool current, bool &last) {
  bool flank = (current == LOW && last == HIGH);
  last = current;
  return flank;
}

void setup() {
  pinMode(BTN_PIN1, INPUT_PULLUP);
  pinMode(BTN_PIN2, INPUT_PULLUP);
  pinMode(BTN_PIN3, INPUT_PULLUP);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  btn1_pressed = digitalRead(BTN_PIN1);
  btn2_pressed = digitalRead(BTN_PIN2);
  btn3_pressed = digitalRead(BTN_PIN3);

  if (detectPositiveFlank(btn1_pressed, btn1_pressed_last)) {
    case_selected = (case_selected + 1) % 3;
  }


  // Switch-case logic
  switch (case_selected){
    case 0:
      if (detectPositiveFlank(btn2_pressed, btn2_pressed_last)){
        led1_value = increaseValue(led1_value);
        }
      if (detectPositiveFlank(btn3_pressed, btn3_pressed_last)){
        led1_value = decreaseValue(led1_value);
        }
      LED_CTRL(LED_PIN1, led1_value);
      //LED_CTRL(LED_PIN2, 0);
      //LED_CTRL(LED_PIN3, 0);
      break;
    
    case 1:
      if (detectPositiveFlank(btn2_pressed, btn2_pressed_last)){
        led2_value = increaseValue(led2_value);
        }
      if (detectPositiveFlank(btn3_pressed, btn3_pressed_last)){
        led2_value = decreaseValue(led2_value);
        }
      //LED_CTRL(LED_PIN1, 0);
      LED_CTRL(LED_PIN2, led2_value);
      //LED_CTRL(LED_PIN3, 0);
      break;


    case 2:
      if (detectPositiveFlank(btn2_pressed, btn2_pressed_last)){
        led3_value = increaseValue(led3_value);
        }
      if (detectPositiveFlank(btn3_pressed, btn3_pressed_last)){
        led3_value = decreaseValue(led3_value);
        }
      //LED_CTRL(LED_PIN1, 0);
      //LED_CTRL(LED_PIN2, 0);
      LED_CTRL(LED_PIN3, led3_value);
      break;

  }

  delay(10);
}
