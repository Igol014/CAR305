#include <Servo.h>

// Конфигурация пинов для драйвера L298N
#define IN1 5  // Левый мотор (вперёд)
#define IN2 6   // Левый мотор (назад)
#define IN3 3   // Правый мотор (вперёд)
#define IN4 11   // Правый мотор (назад)

// Конфигурация сервопривода
#define SERVO_PIN 9
Servo myServo;
int servoAngle = 90;  // Текущий угол сервопривода (0-180)

// Конфигурация энкодеров
#define ENCODER_LEFT_A 2   // Канал A левого энкодера (INT0)
#define ENCODER_LEFT_B 7   // Канал B левого энкодера
#define ENCODER_RIGHT_A 12  // Канал A правого энкодера (INT1)
#define ENCODER_RIGHT_B 8  // Канал B правого энкодера

// Переменные состояния
int motorSpeed = 0;      // Общая скорость (-255 до 255)
int leftSpeed = 0;       // Скорость левого мотора
int rightSpeed = 0;      // Скорость правого мотора
volatile long leftEncoderCount = 0;   // Счетчик левого энкодера
volatile long rightEncoderCount = 0;  // Счетчик правого энкодера
unsigned long lastEncoderPrint = 0;   // Время последнего вывода энкодеров

void setup() {
  // Инициализация моторов
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
  
  // Инициализация сервопривода
  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);
  
  // Инициализация энкодеров
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  
  // Настройка прерываний для энкодеров
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateRightEncoder, CHANGE);
  
  // Инициализация последовательного порта
  Serial.begin(115200);
  Serial.println("System Ready");
}

void loop() {
  // Обработка команд с последовательного порта
  if (Serial.available()) {
    handleCommand(Serial.read());
  }
  
  // Периодическая отправка данных о состоянии
  if (millis() - lastEncoderPrint > 100) {
    Serial.print("SERVO:");
    Serial.print(servoAngle);
    Serial.print("|MOTORS:");
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.print(rightSpeed);
    Serial.print("|ENCODERS:");
    Serial.print(leftEncoderCount);
    Serial.print(",");
    Serial.println(rightEncoderCount);
    
    lastEncoderPrint = millis();
  }
  
  // Управление моторами на основе значений энкодеров
  controlMotors();
}

void handleCommand(char cmd) {
  switch(cmd) {
    case 'W': motorSpeed = min(255, motorSpeed + 25); break;   // Вперед
    case 'S': motorSpeed = max(-255, motorSpeed - 25); break;  // Назад
    case 'X': motorSpeed = 0; break;                          // Стоп
    case 'A': servoAngle = max(0, servoAngle - 10); break;     // Серво влево
    case 'D': servoAngle = min(180, servoAngle + 10); break;   // Серво вправо
  }

  // Обновляем сервопривод
  myServo.write(servoAngle);
  
  // Устанавливаем скорости для моторов
  leftSpeed = -motorSpeed;  // Инверсия для левого мотора
  rightSpeed = motorSpeed;  // Правый мотор без изменений
  
  setMotors(leftSpeed, rightSpeed);
  Serial.print("MOTORS:");
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.println(rightSpeed);
}

void setMotors(int left, int right) {
  // Левый мотор (инвертированный)
  if(left > 0) {
    analogWrite(IN1, left);  // Вперед
    analogWrite(IN2, 0);
  } else if(left < 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(left)); // Назад
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0); // Стоп
  }
  
  // Правый мотор (обычное управление)
  if(right > 0) {
    analogWrite(IN3, right); // Вперед
    analogWrite(IN4, 0);
  } else if(right < 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, abs(right)); // Назад
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0); // Стоп
  }
}

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

// Обработчики прерываний для энкодеров
void updateLeftEncoder() {
  if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void updateRightEncoder() {
  if (digitalRead(ENCODER_RIGHT_A) == digitalRead(ENCODER_RIGHT_B)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

// Функция управления моторами на основе значений энкодеров
void controlMotors() {
  // Здесь можно добавить логику для управления моторами на основе значений энкодеров
  // Например, можно использовать PID-контроллер для поддержания заданной скорости
  // или корректировки направления движения на основе разницы между значениями энкодеров.
  
  long leftError = leftEncoderCount;  // Получаем текущее значение левого энкодера
  long rightError = rightEncoderCount; // Получаем текущее значение правого энкодера
  

  
  // Убедитесь, что скорости не выходят за пределы
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Устанавливаем новые скорости моторов
  setMotors(leftSpeed, rightSpeed);
}