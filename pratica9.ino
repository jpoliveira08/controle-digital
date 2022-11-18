#include <TimerOne.h>

// Tempos para setpoint
unsigned long tempo0 = 0, tempo1 = 60, tempo2 = 120, tempo3 = 180, delta_t = 0;
// Variaveis do sistema de controle
int medido = 0, duty = 0;
float vsensor = 0, sensor_filt = 0, nivel = 0, setpoint = 20, erro =0, Kp = 1, Ki = 0, outPI = 0, outP = 0, outI = 0;
float Imax = 10, Imin = 0, outMax = 10, outMin = 0, u_control = 0;

void setup()
{
    pinMode (3, OUTPUT);
    pinMode (9, OUTPUT);
    Timer1.initialize(20000); // valor em us
    Timer1.attachInterrupt(amostragem1);
    // attachInterrupt ( function , period ) period in us
    Timer1.pwm(9, 0); // pino 9
    digitalWrite(3, LOW);
    Serial.begin(9600);
    Serial.println("CLEARDATA");
    Serial.println("Setpoint, Sensor");
}

void loop()
{
    Timer1.setPwmDuty(9, duty);

    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(nivel);

    delta_t = (millis() - tempo0) / 1000;
    if (delta_t <= tempo1) {
        setpoint = 10;
    } else if (delta_t <= tempo2) {
        setpoint = 15;
    } else if (delta_t <= tempo3) {
        setpoint = 20;
    } else {
        tempo0 = millis();
    }
}

void amostragem1()
{
    medido = analogRead(A0);
    vsensor = float(medido * 5) / 1023;
    // sensor_filt =
    // nivel =
    erro = float (setpoint - nivel);

    // ====== On - Off
    // if (sensor > setpoint) {
    //    u_control = 0
    //} else {
    //    u_control = 1023;
    //}

    // ====== Histerese
    // if ( erro > 1) {
    //    u_control = 1023;
    //}
    // else if ( erro < -1) {
    //    u_control = 0;
    //}

    // ====== Proporcional - Integral
    // outP = Kp * erro;
    // outI += (Ki * Ts * erro);
    // if ( outI > Imax ) {
    //     outI = Imax;
    // }
    // else if ( outI < Imin ) {
    //     outI = Imin;
    //     outPI = outP + outI;
    //     u_control =( outPI / outMax ) * 1023;
    // }

    // ====== Limites duty int
    duty = int (u_control) ;
    if ( duty > 1023) {
        duty = 1023;
    } else if ( duty < 0) {
        duty = 0;
    }
}