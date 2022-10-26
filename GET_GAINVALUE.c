#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>

#define ENCODER_A 17
#define ENCODER_B 27
#define ENC2REDGEAR 216
#define MOTOR1 19
#define MOTOR2 26
#define PULSE 21

FILE* fpt;

int encA;
int encB;
int encoderPosition = 0;
float redGearPosition = 0;

//###################### configuration ######################
int tuning_iteration = 10;
int tuning_reference = 5;

char mode = 'P';
float Pgain = 650;
float Igain = 2;
float Dgain = 4.5;
float delta = 1;
char file_name[] = "P_P100_I1_D1e-1_d1";
//##################### configuration ends ####################

void funENCODER_A()
{
	encA = digitalRead(ENCODER_A);
	encB = digitalRead(ENCODER_B);
	if (encA == HIGH)
	{
		if (encB == LOW)
			encoderPosition++;
		else
			encoderPosition--;
	}
	else
	{
		if (encB == LOW)
			encoderPosition--;
		else
			encoderPosition++;
	}

	redGearPosition = ((float)encoderPosition) / ENC2REDGEAR;
//	printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition); // uncomment this to see motor position
}


void funENCODER_B()
{
	encA = digitalRead(ENCODER_A);
	encB = digitalRead(ENCODER_B);
	if (encB == HIGH)
	{
		if (encA == LOW)
			encoderPosition--;
		else
			encoderPosition++;
	}
	else
	{
		if (encA == LOW)
			encoderPosition++;

		else
			encoderPosition--;
	}
	redGearPosition = ((float)encoderPosition) / ENC2REDGEAR;
//	printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition); // uncomment this to see motor position in realtime

}

void set_amplifer(float m, float error);
double PID_Control(int reference, float Pgain, float Igain, float Dgain);
void PID_Tuning(int reference, char mode, int iteration, float delta ,float Pgain, float Igain, float Dgain);
int main()
{
	wiringPiSetupGpio();
	pinMode(ENCODER_A, INPUT);
	pinMode(ENCODER_B, INPUT);

	int test_iteration = 0;
	int test_reference = 0;
    float itae = 0.0;

	softPwmCreate(MOTOR1, 0, 100);
	softPwmCreate(MOTOR2, 0, 100);

	wiringPiISR(ENCODER_A, INT_EDGE_BOTH, funENCODER_A);
	wiringPiISR(ENCODER_B, INT_EDGE_BOTH, funENCODER_B);

    int pulse = 0; // pulse variable

//#################     tuning code     ########################
//    printf("mode: %c\n", mode);
//    printf("Pgain: %f, Igain: %f, Dgain: %f\n", Pgain, Igain, Dgain);
//    printf("delta %c: %f\n", mode, delta);
//    printf("reference: %f\n", tuning_reference);
//    printf("iteration: %f\n", tuning_iteration);
//    fpt = fopen(file_name, "w+");
//    fprintf(fpt, "ITAE,Pgain,Igain,Dgain\n");
//    PID_Tuning(tuning_reference, mode, tuning_iteration, delta, Pgain, Igain, Dgain);
//
//    fclose(fpt);

//#################     project code     ########################
    printf("Enter number of iterations");
    scanf("%d", &test_iteration);

    int references[10] = {0,};

    for (int i = 0; i < test_iteration; i++) {
        printf("Enter reference number %d: ", i+1);
        scanf("%d", &references[i]);
    }

    for (int i = 0; i < test_iteration; i++) {
        while(1)
        {
            pulse = digitalRead(PULSE);
            if (pulse == HIGH) {
                itae += PID_Control(references[i], Pgain, Igain, Dgain); // cumulate itae for each iteration
                printf("itae(iteration: %d): %f\n", i, itae);
                break;
            }
        }
    }
    printf("total performance(itae): %f\n", itae);

//################     project code ends     #####################

    softPwmWrite(MOTOR1, 0);
    softPwmWrite(MOTOR2, 0);
    return 0;
//################    stop motor1, 2     #########################
}



double PID_Control(int reference, float Pgain, float Igain, float Dgain)
{
	float m = 0;
	float m1 = 0;
	float error = 0;
	float curr_pos = 0;
	float error_1 = 0, error_2 = 0;
	float T = 0.01;
    double itae = 0;
	float G1 = (Pgain + Igain * T + Dgain / T);
	float G2 = -(Pgain + 2 * Dgain / T);
	float G3 = Dgain / T;

	unsigned int startTime = millis();
	while (1)
	{
		unsigned int inner_startTime = millis();

		curr_pos = redGearPosition;
		error = (float)reference - curr_pos;

		m = m1 + G1 * error + G2 * error_1 + G3 * error_2;

		//printf("Error Value: %f\n", error);
		//printf("Actuating signal Value: %f\n", m);

		set_amplifer(m, error);

		error_2 = error_1;
		error_1 = error;
		m1 = m;

		unsigned int inner_endTime = millis();

		while (inner_endTime - inner_startTime < T * 1000)
		{
			inner_endTime = millis();
		}
		itae += (inner_endTime - startTime) / 1000 * fabs(error) * T;

		unsigned int endtime = millis();
		if (endtime - startTime >= 4000) // perform pid control for 7 seconds
		{
			softPwmWrite(MOTOR1, 0);
			softPwmWrite(MOTOR2, 0);
//			printf("PID CONTROL FINISH\n");
            printf("itae: %lf\n", itae);
			break;
		}

	}
    return itae;
}

void PID_Tuning(int reference, char mode, int iteration, float delta ,float Pgain, float Igain, float Dgain) {
    float min_itae = 1e+20; // infinity
    float min_Pgain = 0.0;
    float min_Igain = 0.0;
    float min_Dgain = 0.0;

    switch(mode){
        case 'P' :
            printf("Tuning P_gain\n");
            for (int i = 0; i < iteration; i++) {
                double itae = PID_Control(reference, Pgain, Igain, Dgain);
                fprintf(fpt, "%f, %f, %f, %f\n",itae,Pgain,Igain,Dgain);
                if (itae < min_itae) {
                    min_itae = itae;
                    min_Pgain = Pgain;
                    min_Igain = Igain;
                    min_Dgain = Dgain;
                }
                Pgain += delta;
                encoderPosition = 0;
                redGearPosition = 0.0;
            }
            printf("==================================================\n");
            printf("minimum itae: %lf (Pgain: %f, Igain: %f, Dgain: %f\n", min_itae, min_Pgain, min_Igain, min_Dgain);
            break;
        case 'I' :
            printf("Tuning I_gain\n");
            for (int i = 0; i < iteration; i++) {
                double itae = PID_Control(reference, Pgain, Igain, Dgain);
                fprintf(fpt, "%f, %f, %f, %f\n",itae,Pgain,Igain,Dgain);
                if (itae < min_itae) {
                    min_itae = itae;
                    min_Pgain = Pgain;
                    min_Igain = Igain;
                    min_Dgain = Dgain;
                }
                Igain += delta;
                encoderPosition = 0;
                redGearPosition = 0.0;
            }
            printf("==================================================\n");
            printf("minimum itae: %lf (Pgain: %f, Igain: %f, Dgain: %f\n", min_itae, min_Pgain, min_Igain, min_Dgain);
            break;
        case 'D' :
            printf("Tuning D_gain\n");
            for (int i = 0; i < iteration; i++) {
                double itae = PID_Control(reference, Pgain, Igain, Dgain);
                fprintf(fpt, "%f, %f, %f, %f\n",itae,Pgain,Igain,Dgain);
                if (itae < min_itae) {
                    min_itae = itae;
                    min_Pgain = Pgain;
                    min_Igain = Igain;
                    min_Dgain = Dgain;
                }
                Dgain += delta;
                encoderPosition = 0;
                redGearPosition = 0.0;
            }
            printf("==================================================\n");
            printf("minimum itae: %lf (Pgain: %f, Igain: %f, Dgain: %f\n", min_itae, min_Pgain, min_Igain, min_Dgain);
            break;
        default :
            printf("mode should be in between {P, I, D}.\n");
    }
}

void set_amplifer(float m, float error)
{
	if (error > 0)
	{
		softPwmWrite(MOTOR1, m);
		softPwmWrite(MOTOR2, 0);
	}
	else
	{
		softPwmWrite(MOTOR1, 0);
		softPwmWrite(MOTOR2, -m);
	}
}

