//
// Created by kimdj on 22. 10. 15.
//
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#define ENCODER_A 17
#define ENCODER_B 27
#define ENC2REDGEAR 216
#define MOTOR1 19
#define MOTOR2 26

int encA;
int encB;
int encoderPosition = 0;
float redGearPosition = 0;

void funENCODER_A()
{
    encA = digitalRead(ENCODER_A);
    encB = digitalRead(ENCODER_B);
    if (encA == HIGH)
    {
        if (enB == LOW)
        {
            encoderPosition++;
            printf("A:rising B:%d\n", encB);
        }
        else
        {
            encoderPosition--;
            printf("A:rising B:%d\n", encB);
        }
    }
    else
    {
        if (enB == LOW)
        {
            encoderPosition--;
            printf("A:falling B:%d\n", encB);
        }
        else
        {
            encoderPosition++;
            printf("A:falling B:%d\n", encB);
        }
    }

    redGearPosition = (float)encoderPosition / ENC2REDGEAR;
    printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition);

}


void funENCODER_B()
{
    encA = digitalRead(ENCODER_A);
    encB = digitalRead(ENCODER_B);
    if (encB == HIGH)
    {
        if (enA == LOW)
        {
            encoderPosition--;
            printf("A:%d B:rising\n", encA);
        }
        else
        {
            encoderPosition++;
            printf("A:%d B:rising\n", encA);
        }
    }
    else
    {
        if (enA == LOW)
        {
            encoderPosition++;
            printf("A:%d B:falling\n", encA);
        }
        else
        {
            encoderPosition++;
            printf("A:%d B:falling\n", encA);
        }
    }

    redGearPosition = (float)encoderPosition / ENC2REDGEAR;
    printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition);

}

float read_position()
{
    return redGearPosition;
}

void set_amplifier(float actu_signal) // integer signal?
{
    if (actu_signal > 0)
    {
        softPwmWrite(MOTOR1, actu_signal);
        softPwmWrite(MOTOR2, 0);
    }
    else
    {
        softPwmWrite(MOTOR1, 0);
        softPWmWrite(MOTOR2, -actu_signal);
    }
}


int main() {

    wiringPiSetupGpio();
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);

    wiringPiISR(ENCODER_A, INT_EDGE_BOTH, funENCODER_A);
    wiringPiISR(ENCODER_B, INT_EDGE_BOTH, funENCODER_B);

    // tuning gains Pgain, Igain, Dgain
    int Pgain = 100;
    int Igain = 10;
    int Dgain = 1;

    float T = 1e-3; // sampling time (in sec)

    float itae = 0.0; // accumulate itae while time ellpase starting from zero

    int reference=0;
    int n=0; // number of iterations
    printf("Enter number of iterations: ");
    scanf("%d", &n); // input number of iterations by console

    for (int i = 0; i < n; i ++) {
        printf("Enter reference motor position: ");
        scanf("%d", &reference);
        pid_control(&reference, &Pgain, &Igain, &Dgain, &T, &itae);
        printf("measured itae(iteration: %d): %f\n", i+1, itae);
    }
    printf("PerformanceITAE: %f\n", itae);
    return 0;
}

void pid_control(int* reference, int* Pgain, int* Igain, int* Dgain, float* T, float* itae) { // get variables by reference
    float m, m1=0, e, e1=0, e2=0; // declare variable for ms and errors
    float G1, G2, G3; // coefficients for e_n, e_(n-1), e_(n-2)
    float c;
    // setting coefficients for e_n, e_(n-1), e_(n-2)
    G1 = ((float)(*Pgain) + (float)(*Igain) *(*T) + (float)(*Dgain) / (*T));
    G2 = -((float)(*Pgain) + 2 * (float)(*Dgain) / (*T));
    G3 = ((float)(*Dgain))/(*T);

    unsigned int startTime = millis(); // measure time for T = T
    while (1) {
        unsigned int innerStartTime = millis();
        c = read_position(motor1);
        e = (float)(*reference) - c;
        m = m1 + G1 * e + G2 * e1 + G3 * e2; // update actuating signal
        set_amplifier(m);
        e2 = e1; //
        e1 = e; //
        m1 = m; //

        unsigned int endTime = millis();
        while ((endTime - innerStartTime)< (*T) * 1000) { // stop process until T seconds arrive
            endTime = millis();
        } // delay
        *itae += (endTime - startTime) * abs(e) * (*T);

        if ((e-e1) < 1e-10 && (e-e2) < 1e-10) // break while loop if steady state error occured
            break;
    }
}

float ITAE

