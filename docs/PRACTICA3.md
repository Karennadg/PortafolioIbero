# 📚 Práctica 3: Control de velocidad de motor DC con ESP32

## 1) Resumen

- **Equipo / Autor(es):** _Tomás Toledo y Karen Itzel_  
- **Curso / Asignatura:** _Introducción a la mecatronica_  
- **Fecha:** _19/09/2025_  
- **Descripción breve:** En esta práctica se desarrolló un programa en el ESP32 para controlar la velocidad de un motor de corriente directa (DC) utilizando modulación por ancho de pulso (PWM). Se configuraron los pines de salida digital y el canal PWM del microcontrolador para variar gradualmente la velocidad del motor en ambas direcciones de giro.
---
## 2) Objetivos
- **General:** _Comprender el control de velocidad de un motor DC mediante señales PWM generadas por el ESP32.
Específicos:
Configurar los pines de salida del ESP32 para controlar la dirección de giro de un motor.
Observar el cambio de velocidad en ambas direcciones del giro del motor.
---

## 3) Alcance y Exclusiones
**Incluye:**

-Programación en Arduino IDE utilizando el ESP32.

-Control de un motor DC mediante salidas digitales y PWM.

-Variación progresiva de la velocidad del motor en ambas direcciones.

**No incluye:**

-Control PID o sistemas de regulación automática.

-Integración con módulos de comunicación.

---
## 4) Codigo

'''
CCC
// Que avance en una dirección
#define in1 32
#define in2 33
#define pwm 25

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  ledcAttachChannel(pwm, 1000, 8, 0); // Configuración del canal PWM
}

void loop() {
  // Aceleración
  for (int vel = 0; vel < 256; vel++) {
    ledcWrite(pwm, vel);
    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
    delay(10);
  }

  // Desaceleración
  for (int vel = 256; vel > 0; vel--) {
    ledcWrite(pwm, vel);
    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
    delay(10);
  }
}
'''


## 4) Resultados

_El motor incrementa gradualmente su velocidad desde 0 hasta el valor máximo (255) y luego disminuye hasta detenerse. El sentido de giro se mantiene constante, controlado por las señales digitales de los pines in1 y in2._



_Al final logramos ver el comportamiento controlado del motor_


[Video armado] (https://youtu.be/vTdIwA4jg94)

---
## 5) conclusiones
_Se logró comprender el uso del PWM en el ESP32 para el control de velocidad de un motor DC. El programa permitió visualizar cómo la variación del ciclo de trabajo modifica la potencia entregada al motor, cambiando así su velocidad. Esta práctica constituye la base para el control más avanzado de motores, incluyendo inversión de giro, control de torque y sistemas automáticos_
