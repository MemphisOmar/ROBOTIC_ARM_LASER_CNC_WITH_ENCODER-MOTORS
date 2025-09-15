#include <Arduino.h>
#include <math.h>
#include <PID_v1.h> // Incluimos la librería PID
// --- Parámetro para tolerancia de error para avanzar al siguiente punto (más permisivo para continuidad) ---
const long ENCODER_TOLERANCE_M1_CONTINUOUS = 15; // Ajusta según lo fluido que quieras el trazo
const long ENCODER_TOLERANCE_M2_CONTINUOUS = 15;
// --- Constantes del Brazo SCARA (en mm) ---
const double L1 = 228.0; // Longitud del primer eslabón (base a codo)
const double L2 = 136.5; // Longitud del segundo eslabón (codo a muñeca/efector final)

// --- Pines Motor 1 (J1 - Base) ---
const int encoder1PinA = 2;
const int encoder1PinB = 3;
const int motor1IN1 = 8;
const int motor1IN2 = 9;
const int motor1PWM = 10;
const int limitSwitch1Pin = 45;
const float PULSOS_POR_GRADO_M1 = 3050.0 / 180.0;
const float OFFSET_DEGREES_M1_AFTER_HOMING = 102.0;
const long OFFSET_PULSES_M1_AFTER_HOMING = round(OFFSET_DEGREES_M1_AFTER_HOMING * PULSOS_POR_GRADO_M1);
const float MIN_ANGLE_J1 = -90.0;
const float MAX_ANGLE_J1 = 266.0;

volatile long currentPosition1Pulses = 0;
long motor1TargetPulses = 0;
bool motor1Moving = false;
const long ENCODER_TOLERANCE_M1 = 5;

// Variables PID para Motor 1
double Setpoint1, Input1, Output1;
double Kp1 = 3.5, Ki1 = 0.60, Kd1 = 0.15;
PID motor1PID(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

// Limites de salida del PID para Motor 1
const int minMotor1PWM = 30;
const int maxMotor1PWM = 50;

// --- Pines Motor 2 (J2 - Codo) ---
const int encoder2PinA = 21;
const int encoder2PinB = 20;
const int motor2IN1 = 11;
const int motor2IN2 = 12;
const int motor2PWM = 13;
const int limitSwitch2Pin = 46;
const float PULSOS_POR_GRADO_M2 = 8150.0 / 180.0;
const float OFFSET_DEGREES_M2_AFTER_HOMING = 159.0;
const long OFFSET_PULSES_M2_AFTER_HOMING = round(OFFSET_DEGREES_M2_AFTER_HOMING * PULSOS_POR_GRADO_M2);
const float MIN_ANGLE_J2 = -150.0;
const float MAX_ANGLE_J2 = 150.0;

volatile long currentPosition2Pulses = 0;
long motor2TargetPulses = 0;
bool motor2Moving = false;
const long ENCODER_TOLERANCE_M2 = 5;

// Variables PID para Motor 2
double Setpoint2, Input2, Output2;
double Kp2 = 19.5, Ki2 = 0.0932, Kd2 = 0.95;
PID motor2PID(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

// Limites de salida del PID para Motor 2
const int minMotor2PWM = 180;
const int maxMotor2PWM = 250;

// --- Variables Generales del Sistema ---
bool homingCompleteM1 = false;
bool homingCompleteM2 = false;
bool allHomingDone = false; // Bandera para saber si ambos homes están completos

// --- Pines y Constantes del Láser ---
const int laserPin = 6;      // Pin donde está conectado el láser (PWM)
const int laserPower =100;   // PWM máximo para el láser (0-255)
// Un umbral más alto puede ser necesario para algunos láseres para que se vean.
// Si 70 no funciona, intenta con 100, 150 o incluso 255 para probar.
// const int laserPower = 150; // ¡PRUEBA CON UN VALOR MÁS ALTO SI ES NECESARIO!

// --- Variables para Dibujo de Trayectoria ---
int currentPointIndex = 0;
bool drawingEnabled = false;
bool isDrawingSquare = false;
bool isDrawingTriangle = false;

// --- NUEVAS CONSTANTES DE LÍMITES DE WORKSPACE EN COORDENADAS DEL ROBOT (actuales) ---
const double WORKSPACE_X_MIN_ROBOT = 98.0;
const double WORKSPACE_X_MAX_ROBOT = 309.0;
const double WORKSPACE_Y_MIN_ROBOT = -143.5;
const double WORKSPACE_Y_MAX_ROBOT = 160.0;

// --- CONSTANTES DEL MARCO DE REFERENCIA DE LA HOJA (CALCULADAS) ---
const double PAPER_ORIGIN_X_ROBOT = 98.0;
const double PAPER_ORIGIN_Y_ROBOT = -143.5;
const double PAPER_ROTATION_RAD = atan2(6.8, 211.0);
const double PAPER_WIDTH_MM = 218.9;
const double PAPER_HEIGHT_MM = 270.4;

// --- Variables para el envío de telemetría ---
unsigned long lastTelemetryTime = 0;
const long TELEMETRY_INTERVAL = 1000;

// --- Arrays para los puntos de las figuras (inicialización en setup o en las funciones de dibujo) ---
double* currentShapePointsX = nullptr;
double* currentShapePointsY = nullptr;
int numShapePoints = 0;

// --- Función general para frenar un motor ---
void brakeMotor(int motorId) {
  if (motorId == 1) {
    digitalWrite(motor1IN1, LOW);
    digitalWrite(motor1IN2, LOW);
    analogWrite(motor1PWM, 0);
  } else if (motorId == 2) {
    digitalWrite(motor2IN1, LOW);
    digitalWrite(motor2IN2, LOW);
    analogWrite(motor2PWM, 0);
  }
}

// --- Rutinas de Interrupción para los Encoders ---
void updateEncoder1() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    currentPosition1Pulses++;
  } else {
    currentPosition1Pulses--;
  }
}

void updateEncoder2() {
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    currentPosition2Pulses++;
  } else {
    currentPosition2Pulses--;
  }
}

// --- Función de Cinemática Directa (calcula X, Y a partir de ángulos en grados) ---
void calculateForwardKinematics(double theta1_deg, double theta2_deg, double &x, double &y) {
  double theta1_rad = radians(theta1_deg);
  double theta2_rad = radians(theta2_deg);

  x = L1 * cos(theta1_rad) + L2 * cos(theta1_rad + theta2_rad);
  y = L1 * sin(theta1_rad) + L2 * sin(theta1_rad + theta2_rad);
}

// --- Función de Cinemática Inversa (calcula ángulos en grados) ---
bool calculateInverseKinematics(double x, double y, double &theta1, double &theta2) {
  double dist_sq = x * x + y * y;
  double dist = sqrt(dist_sq);

  if (dist > (L1 + L2) || dist < abs(L1 - L2)) {
    Serial.println("Rango inval."); // La definición no debe pasar de 12 caracteres.
    return false;
  }

  double cos_theta2 = (dist_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (cos_theta2 > 1.0 || cos_theta2 < -1.0) {
    Serial.println("Angulo inval."); // La definición no debe pasar de 12 caracteres.
    return false;
  }
  theta2 = atan2(-sqrt(1 - cos_theta2 * cos_theta2), cos_theta2);

  double alpha = atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
  theta1 = atan2(y, x) - alpha;

  theta1 = degrees(theta1);
  theta2 = degrees(theta2);

  if (theta1 < MIN_ANGLE_J1 || theta1 > MAX_ANGLE_J1) {
    Serial.println("J1 fuera lim"); // La definición no debe pasar de 12 caracteres.
    return false;
  }
  if (theta2 < MIN_ANGLE_J2 || theta2 > MAX_ANGLE_J2) {
    Serial.println("J2 fuera lim"); // La definición no debe pasar de 12 caracteres.
    return false;
  }

  return true;
}

// --- NUEVA FUNCIÓN: Transforma coordenadas de la hoja a coordenadas del robot ---
bool transformPaperToRobotCoords(double x_paper, double y_paper, double &x_robot, double &y_robot) {
    if (x_paper < 0 || x_paper > PAPER_WIDTH_MM || y_paper < 0 || y_paper > PAPER_HEIGHT_MM) {
        Serial.println("Hoja fuera."); // La definición no debe pasar de 12 caracteres.
        return false;
    }

    double rotated_x = x_paper * cos(PAPER_ROTATION_RAD) - y_paper * sin(PAPER_ROTATION_RAD);
    double rotated_y = x_paper * sin(PAPER_ROTATION_RAD) + y_paper * cos(PAPER_ROTATION_RAD);

    x_robot = rotated_x + PAPER_ORIGIN_X_ROBOT;
    y_robot = rotated_y + PAPER_ORIGIN_Y_ROBOT;

    return true;
}

// --- NUEVA FUNCIÓN: Transforma coordenadas del robot a coordenadas de la hoja ---
bool transformRobotToPaperCoords(double x_robot, double y_robot, double &x_paper, double &y_paper) {
    // Trasladar al origen de la hoja
    double dx = x_robot - PAPER_ORIGIN_X_ROBOT;
    double dy = y_robot - PAPER_ORIGIN_Y_ROBOT;
    // Rotar en sentido contrario al ángulo de la hoja
    double cos_a = cos(-PAPER_ROTATION_RAD);
    double sin_a = sin(-PAPER_ROTATION_RAD);
    x_paper = dx * cos_a - dy * sin_a;
    y_paper = dx * sin_a + dy * cos_a;
    return true;
}

// Función para verificar si un punto (X,Y) está dentro del workspace definido (en coordenadas del robot)
bool isPointInRobotWorkspace(double x_robot, double y_robot) {
  if (x_robot >= WORKSPACE_X_MIN_ROBOT && x_robot <= WORKSPACE_X_MAX_ROBOT &&
      y_robot >= WORKSPACE_Y_MIN_ROBOT && y_robot <= WORKSPACE_Y_MAX_ROBOT) {
    return true;
  } else {
    Serial.println("Fuera robot."); // La definición no debe pasar de 12 caracteres.
    return false;
  }
}

// --- Función para iniciar el movimiento de un motor a una posición ABSOLUTA (no bloqueante) ---
void startMotorMoveAbsolute(int motorId, float targetDegrees) {
  if (motorId == 1) {
    long targetPulsesAbsolute = round(targetDegrees * PULSOS_POR_GRADO_M1);
    motor1TargetPulses = targetPulsesAbsolute;
    Setpoint1 = targetPulsesAbsolute; // Establecer el setpoint para el PID del Motor 1
    motor1Moving = true;

    Serial.print("M1: Mover a "); Serial.print(targetDegrees); Serial.print("°.");
    Serial.print(" Obj: "); Serial.println(targetPulsesAbsolute);

  } else if (motorId == 2) {
    long targetPulsesAbsolute = round(targetDegrees * PULSOS_POR_GRADO_M2);
    motor2TargetPulses = targetPulsesAbsolute;
    Setpoint2 = targetPulsesAbsolute; // Establecer el setpoint para el PID del Motor 2
    motor2Moving = true;

    Serial.print("M2: Mover a "); Serial.print(targetDegrees); Serial.print("°.");
    Serial.print(" Obj: "); Serial.println(targetPulsesAbsolute);
  } else {
    return;
  }
}

// --- Función para realizar el homing del Motor 1 ---
void performHomingM1() {
  Serial.println("Home M1..."); // La definición no debe pasar de 12 caracteres.
  homingCompleteM1 = false;

  // Mover el motor hacia el limit switch
  digitalWrite(motor1IN1, LOW);
  digitalWrite(motor1IN2, HIGH);
  analogWrite(motor1PWM, 50); // Velocidad fija más baja para homing

  while (digitalRead(limitSwitch1Pin) == HIGH) { // Moverse mientras NO esté presionado
    delay(1);
  }

  brakeMotor(1);
  Serial.println("M1: Limit OK."); // La definición no debe pasar de 12 caracteres.
  currentPosition1Pulses = 0; // Resetear la posición después de tocar el límite

  // Mover el offset después de tocar el límite
  long tempTargetPulses = OFFSET_PULSES_M1_AFTER_HOMING;
  Setpoint1 = tempTargetPulses;
  motor1Moving = true; // Activar el PID para el movimiento de offset

  digitalWrite(motor1IN1, HIGH);
  digitalWrite(motor1IN2, LOW);

  while (motor1Moving) { // El PID se encargará de llevarlo al offset
    Input1 = currentPosition1Pulses;
    motor1PID.Compute();
    if (Output1 > 0) {
      digitalWrite(motor1IN1, HIGH);
      digitalWrite(motor1IN2, LOW);
      analogWrite(motor1PWM, abs(Output1));
    } else if (Output1 < 0) {
      digitalWrite(motor1IN1, LOW);
      digitalWrite(motor1IN2, HIGH);
      analogWrite(motor1PWM, abs(Output1));
    } else {
      brakeMotor(1);
    }
    if (abs(Setpoint1 - Input1) < ENCODER_TOLERANCE_M1) {
      brakeMotor(1);
      motor1Moving = false;
    }
    delay(1);
  }

  currentPosition1Pulses = 0; // Re-resetear a 0 después del offset para que esta sea la nueva posición 0 de operación
  homingCompleteM1 = true;
  motor1PID.SetTunings(Kp1, Ki1, Kd1); // Restaurar tunings
  motor1PID.SetMode(AUTOMATIC);
  motor1PID.SetOutputLimits(-maxMotor1PWM, maxMotor1PWM);
  motor1PID.SetSampleTime(10);

  Serial.println("Home M1 listo."); // La definición no debe pasar de 12 caracteres.
}

// --- Función para realizar el homing del Motor 2 ---
void performHomingM2() {
  Serial.println("Home M2..."); // La definición no debe pasar de 12 caracteres.
  homingCompleteM2 = false;

  digitalWrite(motor2IN1, LOW);
  digitalWrite(motor2IN2, HIGH);
  analogWrite(motor2PWM, 220);

  while (digitalRead(limitSwitch2Pin) == HIGH) { // Moverse mientras NO esté presionado
    delay(1);
    Serial.println(digitalRead(limitSwitch2Pin));
  }

  brakeMotor(2);
  Serial.println("M2: Limit OK."); // La definición no debe pasar de 12 caracteres.
  currentPosition2Pulses = 0; // Resetear la posición después de tocar el límite

  // Mover el offset después de tocar el límite
  long tempTargetPulses = OFFSET_PULSES_M2_AFTER_HOMING;
  Setpoint2 = tempTargetPulses;
  motor2Moving = true; // Activar el PID para el movimiento de offset

  digitalWrite(motor2IN1, HIGH);
  digitalWrite(motor2IN2, LOW);

  while (motor2Moving) { // El PID se encargará de llevarlo al offset
    Input2 = currentPosition2Pulses;
    motor2PID.Compute();
    if (Output2 > 0) {
      digitalWrite(motor2IN1, HIGH);
      digitalWrite(motor2IN2, LOW);
      analogWrite(motor2PWM, abs(Output2));
    } else if (Output2 < 0) {
      digitalWrite(motor2IN1, LOW);
      digitalWrite(motor2IN2, HIGH);
      analogWrite(motor2PWM, abs(Output2));
    } else {
      brakeMotor(2);
    }
    if (abs(Setpoint2 - Input2) < ENCODER_TOLERANCE_M2) {
      brakeMotor(2);
      motor2Moving = false;
    }
    delay(1);
  }

  currentPosition2Pulses = 0; // Re-resetear a 0 después del offset para que esta sea la nueva posición 0 de operación
  homingCompleteM2 = true;
  motor2PID.SetTunings(Kp2, Ki2, Kd2); // Restaurar tunings
  motor2PID.SetMode(AUTOMATIC);
  motor2PID.SetOutputLimits(-maxMotor2PWM, maxMotor2PWM);
  motor2PID.SetSampleTime(10);

  Serial.println("Home M2 listo."); // La definición no debe pasar de 12 caracteres.
}

// --- Funciones de control del láser ---
void laserOn() {
  analogWrite(laserPin, laserPower);
}

void laserOff() {
  analogWrite(laserPin, 0);
}

// --- Función para dibujar un cuadrado ---
void drawSquare() {
    Serial.println("Dibujar cuad."); // La definición no debe pasar de 12 caracteres.
    drawingEnabled = true;
    isDrawingSquare = true;
    isDrawingTriangle = false;
    currentPointIndex = 0;

    // Define los puntos del cuadrado con origen en (150, 100) en coordenadas de la hoja
    // Tamaño del lado del cuadrado: 50mm
    static double squarePtsX[] = { 150.0, 200.0, 200.0, 150.0, 150.0 };
    static double squarePtsY[] = { 100.0, 100.0, 150.0, 150.0, 100.0 };
    
    currentShapePointsX = squarePtsX;
    currentShapePointsY = squarePtsY;
    numShapePoints = 5;
    analogWrite(laserPin, 0);
    laserOff(); // Asegurarse de que el láser esté apagado al inicio del dibujo
}

// --- Función para dibujar un triángulo ---
void drawTriangle() {
    Serial.println("Dibujar trian."); // La definición no debe pasar de 12 caracteres.
    drawingEnabled = true;
    isDrawingTriangle = true;
    isDrawingSquare = false;
    currentPointIndex = 0;

    // Define los puntos del triángulo con origen en (150, 100) en coordenadas de la hoja
    // Triángulo equilátero con lado de 60mm aprox.
    static double trianglePtsX[] = { 150.0, 210.0, 180.0, 150.0 };
    static double trianglePtsY[] = { 100.0, 100.0, 151.96, 100.0 }; // Altura = sqrt(3)/2 * lado = 0.866 * 60 = 51.96
    
    currentShapePointsX = trianglePtsX;
    currentShapePointsY = trianglePtsY;
    numShapePoints = 4;
    analogWrite(laserPin, 0);
    laserOff(); // Asegurarse de que el láser esté apagado al inicio del dibujo
}

volatile bool ultimoComandoXYH = false; // --- NUEVO: Variable global para el último comando XYH ---
volatile bool esperandoXYH = false; // NUEVO: indica si estamos esperando terminar un movimiento XYH
volatile double ultimoXYH_X = 0, ultimoXYH_Y = 0; // Para debug

// --- Buffer para puntos XYH recibidos ---
#define MAX_XYH_POINTS 400  // Límite seguro para Arduino Mega
float xyh_points_x[MAX_XYH_POINTS];
float xyh_points_y[MAX_XYH_POINTS];
double trajX[MAX_XYH_POINTS]; // Buffer temporal para conversión a double
double trajY[MAX_XYH_POINTS];
int xyh_points_count = 0;
int xyh_points_index = 0;
bool xyh_sequence_active = false;

// --- NUEVO: Bandera para pedir más puntos a la PC ---
bool need_more_xyh_points = false;
// --- NUEVO: Bandera para saber si el dibujo completo ya terminó ---
bool xyh_dibujo_completo = false;

void setup() {
  // --- Configurar pines Motor 1 ---
  pinMode(motor1IN1, OUTPUT);
  pinMode(motor1IN2, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(limitSwitch1Pin, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), updateEncoder1, CHANGE);

  // --- Configurar pines Motor 2 ---
  pinMode(motor2IN1, OUTPUT);
  pinMode(motor2IN2, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(limitSwitch2Pin, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), updateEncoder2, CHANGE);

  // --- Configurar pin del Láser ---
  pinMode(laserPin, OUTPUT);
  laserOff(); // Asegurarse de que el láser esté apagado al inicio

  // --- Iniciar comunicación serial ---
  Serial.begin(115200);
  Serial.println("Control J1/J2"); // La definición no debe pasar de 12 caracteres.
  Serial.println("Robot SCARA"); // La definición no debe pasar de 12 caracteres.

  // --- Configurar PID para Motor 1 y 2 ---
  motor1PID.SetMode(AUTOMATIC);
  motor1PID.SetOutputLimits(-maxMotor1PWM, maxMotor1PWM);
  motor1PID.SetSampleTime(10);

  motor2PID.SetMode(AUTOMATIC);
  motor2PID.SetOutputLimits(-maxMotor2PWM, maxMotor2PWM);
  motor2PID.SetSampleTime(10);

  // --- Realizar homing secuencial ---
  performHomingM1();
  performHomingM2();
  allHomingDone = true;

  Serial.println("Homes OK. Listo."); // La definición no debe pasar de 12 caracteres.
  Serial.println("Cmd: [M_ID],[grados] o 'draw' o 'XYH,x,y' o 'cuadrado' o 'triangulo'"); // La definición no debe pasar de 12 caracteres.
}

// --- Nueva función: dibujar trayectoria recibida desde la interfaz (tipo drawSquare pero dinámica) ---
void drawTrajectoryFromInterface() {
    Serial.println("Dibujar tray.");
    drawingEnabled = true;
    isDrawingSquare = false;
    isDrawingTriangle = false;
    currentPointIndex = 0;
    analogWrite(laserPin, 0);
    laserOff();
    // --- CORRECCIÓN: Copia los puntos float a double para el buffer de dibujo ---
    for (int i = 0; i < xyh_points_count && i < MAX_XYH_POINTS; i++) {
        trajX[i] = (double)xyh_points_x[i];
        trajY[i] = (double)xyh_points_y[i];
    }
    currentShapePointsX = trajX;
    currentShapePointsY = trajY;
    numShapePoints = xyh_points_count;
    // --- CORRECCIÓN: Reinicia el índice de avance SOLO si es el primer bloque ---
    // (No reiniciar currentPointIndex aquí si ya se está dibujando)
    // currentPointIndex = 0; // <-- ya está arriba, no repetir aquí
}

void loop() {
  // --- Procesar comandos seriales ---
  if (Serial.available() > 0 && allHomingDone) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("XYH_START")) {
      xyh_points_count = 0;
      xyh_points_index = 0;
      xyh_sequence_active = true;
      esperandoXYH = false;
      need_more_xyh_points = false;
      xyh_dibujo_completo = false;
      return;
    }
    if (input.equalsIgnoreCase("XYH_END")) {
      xyh_sequence_active = false;
      if (xyh_points_count > 0) {
        drawTrajectoryFromInterface();
      }
      // No marcar como completo aquí, esperar a que la PC indique que ya no hay más puntos
      return;
    }
    if (input.equalsIgnoreCase("XYH_MORE")) {
      // --- CORRECCIÓN: SIEMPRE reinicia el buffer y el índice para recibir el siguiente bloque ---
      xyh_points_count = 0;
      xyh_points_index = 0;
      xyh_sequence_active = true;
      esperandoXYH = false;
      need_more_xyh_points = false;
      xyh_dibujo_completo = false;
      return;
    }
    if (input.equalsIgnoreCase("XYH_DONE")) {
      // La PC indica que ya no hay más puntos, termina el dibujo al agotar el buffer
      xyh_dibujo_completo = true;
      return;
    }
    if (xyh_sequence_active) {
      int firstSeparator = input.indexOf(',');
      if (firstSeparator != -1) {
        String commandType = input.substring(0, firstSeparator);
        if (commandType.equalsIgnoreCase("XYH")) {
          int secondSeparator = input.indexOf(',', firstSeparator + 1);
          if (secondSeparator != -1 && xyh_points_count < MAX_XYH_POINTS) {
            String xStr = input.substring(firstSeparator + 1, secondSeparator);
            String yStr = input.substring(secondSeparator + 1);
            xyh_points_x[xyh_points_count] = xStr.toFloat();
            xyh_points_y[xyh_points_count] = yStr.toFloat();
            xyh_points_count++;
            // --- CORRECCIÓN: Si llegan puntos, desactiva la bandera de petición ---
            need_more_xyh_points = false;
          }
          // Si el buffer está lleno, pide más puntos solo una vez
          else if (xyh_points_count >= MAX_XYH_POINTS) {
            if (!need_more_xyh_points) {
              Serial.println("NEED_XYH");
              need_more_xyh_points = true;
            }
          }
        }
      }
      return;
    }

    // --- SOLO ejecuta comandos manuales si NO está esperando XYH ---
    if (!esperandoXYH) {
      // --- Ejecución automática de puntos XYH desde buffer ---
      if (esperandoXYH && xyh_points_count > 0 && xyh_points_index < xyh_points_count) {
        if (!motor1Moving && !motor2Moving) {
          double targetX_paper = xyh_points_x[xyh_points_index];
          double targetY_paper = xyh_points_y[xyh_points_index];
          double targetX_robot, targetY_robot;
          double targetTheta1, targetTheta2;

          if (!transformPaperToRobotCoords(targetX_paper, targetY_paper, targetX_robot, targetY_robot)) {
            Serial.println("XYH fuera lim");
            xyh_points_index++;
            delay(5);
            return;
          } else if (!isPointInRobotWorkspace(targetX_robot, targetY_robot)) {
            Serial.println("XYH fuera ws");
            xyh_points_index++;
            delay(5);
            return;
          } else if (calculateInverseKinematics(targetX_robot, targetY_robot, targetTheta1, targetTheta2)) {
            // Lógica igual a drawSquare/drawTriangle: láser solo se enciende después del primer punto
            if (xyh_points_index == 0) {
              laserOff();
            } else {
              laserOn();
            }
            startMotorMoveAbsolute(1, targetTheta1);
            startMotorMoveAbsolute(2, targetTheta2);
          } else {
            Serial.println("IK Fallo");
            xyh_points_index++;
            delay(5);
          }
        }
        return;
      }

      // --- Modo normal: permite comandos manuales y de figuras ---
      if (input.equalsIgnoreCase("cuadrado")) {
        drawSquare();
      } else if (input.equalsIgnoreCase("triangulo")) {
        drawTriangle();
      }
      else if (input.equalsIgnoreCase("draw")) {
        Serial.println("No impl. 'draw'"); // La definición no debe pasar de 12 caracteres.
      }
      else {
        // Si se envía un comando de movimiento manual (ej: 1,90 o XYH,x,y), asegurar el láser apagado.
        if (drawingEnabled) // Si estábamos dibujando y se interrumpe con un comando manual.
        {
            drawingEnabled = false;
            isDrawingSquare = false;
            isDrawingTriangle = false;
            laserOff();
            Serial.println("Dibujo inter."); // La definición no debe pasar de 12 caracteres.
        }
        int firstSeparator = input.indexOf(',');
        if (firstSeparator != -1) {
          String commandType = input.substring(0, firstSeparator);

          // SOLO permite comandos XYH para dibujos automáticos
          if (commandType.equalsIgnoreCase("XYH")) {
              int secondSeparator = input.indexOf(',', firstSeparator + 1);
              if (secondSeparator != -1) {
                  String xStr = input.substring(firstSeparator + 1, secondSeparator);
                  String yStr = input.substring(secondSeparator + 1);

                  double targetX_paper = xStr.toFloat();
                  double targetY_paper = yStr.toFloat();
                  double targetX_robot, targetY_robot;
                  double targetTheta1, targetTheta2;

                  if (!transformPaperToRobotCoords(targetX_paper, targetY_paper, targetX_robot, targetY_robot)) {
                      Serial.println("OKP");
                      return;
                  } else if (!isPointInRobotWorkspace(targetX_robot, targetY_robot)) {
                      Serial.println("OKP");
                      return;
                  } else if (!motor1Moving && !motor2Moving) {
                      if (calculateInverseKinematics(targetX_robot, targetY_robot, targetTheta1, targetTheta2)) {
                          startMotorMoveAbsolute(1, targetTheta1);
                          startMotorMoveAbsolute(2, targetTheta2);
                          ultimoComandoXYH = true;
                          esperandoXYH = true;
                          ultimoXYH_X = targetX_paper;
                          ultimoXYH_Y = targetY_paper;
                      } else {
                          Serial.println("IK Fallo");
                          Serial.println("OKP");
                      }
                  } else {
                      Serial.println("OKP");
                  }
              } else {
                  Serial.println("OKP");
              }
              return;
          }

          // Si no es XYH, ignora comandos en grados durante modo dibujo automático
          String degreesStr = input.substring(firstSeparator + 1);
          int motorId = commandType.toInt();
          float degrees = degreesStr.toFloat();

          if (motorId == 1 && !motor1Moving) {
            startMotorMoveAbsolute(1, degrees);
          } else if (motorId == 2 && !motor2Moving) {
            startMotorMoveAbsolute(2, degrees);
          } else {
            Serial.println("M-ID o ocupado"); // La definición no debe pasar de 12 caracteres.
          }
        } else {
          Serial.println("Cmd inválido"); // La definición no debe pasar de 12 caracteres.
        }
      }
    }
  }

  // --- Lógica de control para Motor 1 (CON PID) ---
  if (motor1Moving) {
    Input1 = currentPosition1Pulses;
    motor1PID.Compute();

    int pwm = (int)Output1;
    int minPWM = minMotor1PWM;
    pwm = applyDeadzonePWM(pwm, minPWM);

    if (pwm > 0) {
      digitalWrite(motor1IN1, HIGH);
      digitalWrite(motor1IN2, LOW);
      analogWrite(motor1PWM, pwm);
    } else if (pwm < 0) {
      digitalWrite(motor1IN1, LOW);
      digitalWrite(motor1IN2, HIGH);
      analogWrite(motor1PWM, -pwm);
    } else {
      brakeMotor(1);
    }

    // Usa tolerancia mayor si está en modo dibujo automático para mayor continuidad
    long tol = drawingEnabled ? ENCODER_TOLERANCE_M1_CONTINUOUS : ENCODER_TOLERANCE_M1;
    if (abs(Setpoint1 - Input1) < tol) {
      brakeMotor(1);
      motor1Moving = false;
      Serial.print("M1: Llegó ");
      Serial.print(currentPosition1Pulses / PULSOS_POR_GRADO_M1, 2);
      Serial.println("°");
    }
  }

  // --- Lógica de control para Motor 2 (CON PID) ---
  if (motor2Moving) {
    Input2 = currentPosition2Pulses;
    motor2PID.Compute();

    int pwm = (int)Output2;
    int minPWM = minMotor2PWM;
    pwm = applyDeadzonePWM(pwm, minPWM);

    if (pwm > 0) {
      digitalWrite(motor2IN1, HIGH);
      digitalWrite(motor2IN2, LOW);
      analogWrite(motor2PWM, pwm);
    } else if (pwm < 0) {
      digitalWrite(motor2IN1, LOW);
      digitalWrite(motor2IN2, HIGH);
      analogWrite(motor2PWM, -pwm);
    } else {
      brakeMotor(2);
    }

    // Usa tolerancia mayor si está en modo dibujo automático para mayor continuidad
    long tol = drawingEnabled ? ENCODER_TOLERANCE_M2_CONTINUOUS : ENCODER_TOLERANCE_M2;
    if (abs(Setpoint2 - Input2) < tol) {
      brakeMotor(2);
      motor2Moving = false;
      Serial.print("M2: Llegó ");
      Serial.print(currentPosition2Pulses / PULSOS_POR_GRADO_M2, 2);
      Serial.println("°");
      if (!motor1Moving && !motor2Moving && (ultimoComandoXYH || esperandoXYH)) {
        Serial.println("OKP");
        ultimoComandoXYH = false;
        esperandoXYH = false;
      }
    }
  }

  // --- Lógica de Dibujo de Trayectoria (generalizada para cuadrado/triángulo) ---
  if (drawingEnabled) {
    if (!motor1Moving && !motor2Moving) {
      if (currentPointIndex < numShapePoints) {
        // Si es el primer punto, no enciendas el láser todavía.
        // Mueve el robot al primer punto con el láser apagado.
        // El láser se encenderá JUSTO ANTES de mover al segundo punto,
        // garantizando que solo grabe durante el trazado.
        if (currentPointIndex > 0) { // Enciende el láser si no es el primer punto
            laserOn();
        } else { // Si es el primer punto, asegúrate de que el láser esté apagado mientras se posiciona.
            laserOff();
        }

        double targetX_paper = currentShapePointsX[currentPointIndex];
        double targetY_paper = currentShapePointsY[currentPointIndex];
        double targetX_robot, targetY_robot;
        double targetTheta1, targetTheta2;

        Serial.print("Punto Hoja "); // La definición no debe pasar de 12 caracteres.
        Serial.print(currentPointIndex);
        Serial.print(": ");
        Serial.print(targetX_paper);
        Serial.print(",");
        Serial.println(targetY_paper);

        if (!transformPaperToRobotCoords(targetX_paper, targetY_paper, targetX_robot, targetY_robot)) {
            drawingEnabled = false;
            laserOff(); // Apagar láser si hay error
            return;
        }

        if (!isPointInRobotWorkspace(targetX_robot, targetY_robot)) {
            drawingEnabled = false;
            laserOff(); // Apagar láser si hay error
            return;
        }

        if (calculateInverseKinematics(targetX_robot, targetY_robot, targetTheta1, targetTheta2)) {
          startMotorMoveAbsolute(1, targetTheta1);
          startMotorMoveAbsolute(2, targetTheta2);
          currentPointIndex++;
        } else {
          Serial.println("Pto inaccesible"); // La definición no debe pasar de 12 caracteres.
          drawingEnabled = false;
          laserOff(); // Apagar láser si hay error
        }
      } else {
        // --- CORRECCIÓN: Solo pide más puntos si el buffer está vacío y no han llegado nuevos ---
        if (!xyh_dibujo_completo && xyh_points_count == 0) {
          if (!need_more_xyh_points) {
            delay(100); // Espera 100 ms antes de pedir más puntos (para asegurar que sea el último mensaje)
            Serial.println("NEED_XYH");
            need_more_xyh_points = true;
          }
          return;
        } else if (xyh_dibujo_completo) {
          drawingEnabled = false;
          isDrawingSquare = false;
          isDrawingTriangle = false;
          laserOff();
          Serial.println("Dibujo final.");
          currentPointIndex = 0;
        } else if (xyh_points_count > 0) {
          // --- CORRECCIÓN: Cuando llegan nuevos puntos, reinicia el buffer de avance para el nuevo bloque ---
          for (int i = 0; i < xyh_points_count && i < MAX_XYH_POINTS; i++) {
              trajX[i] = (double)xyh_points_x[i];
              trajY[i] = (double)xyh_points_y[i];
          }
          currentShapePointsX = trajX;
          currentShapePointsY = trajY;
          numShapePoints = xyh_points_count;
          currentPointIndex = 0; // <-- AVANZA AL SIGUIENTE BLOQUE
        }
      }
    } else {
      laserOff();
    }

    // --- NUEVO: Si se está dibujando y quedan pocos puntos, pedir más a la PC ---
    if (drawingEnabled && (currentPointIndex >= numShapePoints) && !need_more_xyh_points) {
      // Si se llegó al final del buffer y la PC puede enviar más
      Serial.println("NEED_XYH");
      need_more_xyh_points = true;
      // El Arduino esperará a que la PC envíe "XYH_MORE" y nuevos puntos
    }
  }

  // --- Enviar telemetría periódicamente ---
  if (allHomingDone && millis() - lastTelemetryTime > TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();
    double current_j1_deg = (double)currentPosition1Pulses / PULSOS_POR_GRADO_M1;
    double current_j2_deg = (double)currentPosition2Pulses / PULSOS_POR_GRADO_M2;
    double current_x, current_y;
    double current_x_paper, current_y_paper;

    calculateForwardKinematics(current_j1_deg, current_j2_deg, current_x, current_y);
    transformRobotToPaperCoords(current_x, current_y, current_x_paper, current_y_paper);

    Serial.print("DATA,"); // La definición no debe pasar de 12 caracteres.
    Serial.print(current_j1_deg, 2);
    Serial.print(",");
    Serial.print(current_j2_deg, 2);
    Serial.print(",");
    Serial.print(current_x_paper, 2); // X hoja
    Serial.print(",");
    Serial.println(current_y_paper, 2); // Y hoja
  }

  // --- Avance automático de puntos XYH tras movimiento ---
  if (esperandoXYH && xyh_points_count > 0 && xyh_points_index < xyh_points_count) {
    if (!motor1Moving && !motor2Moving) {
      xyh_points_index++;
      delay(5);
      // No OKBLOQUE, solo termina cuando termina la lista
      if (xyh_points_index >= xyh_points_count) {
        esperandoXYH = false;
        laserOff();
        xyh_points_count = 0;
        xyh_points_index = 0;
      }
    }
  }
}

// --- Nueva función para aplicar PWM mínimo efectivo (zona muerta) ---
int applyDeadzonePWM(int pwm, int minPWM) {
  if (pwm > 0) {
    return (pwm < minPWM) ? minPWM : pwm;
  } else if (pwm < 0) {
    return (pwm > -minPWM) ? -minPWM : pwm;
  }
  return 0;
}

