#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>

#define ENCODER_A 17
#define ENCODER_B 27
#define ENC2REDGEAR 216
#define MOTOR1 19
#define MOTOR2 26
#define PULSE 21

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
//	printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition); // uncomment this to see motor position

}

void set_amplifer(float m, float error);
double PID_Control(int reference, float Pgain, float Igain, float Dgain);
void PID_Tuning(int reference, char mode, int iteration, float diff ,float Pgain, float Igain, float Dgain);
int main()
{
	wiringPiSetupGpio();
	pinMode(ENCODER_A, INPUT);
	pinMode(ENCODER_B, INPUT);

	int iterations = 0;
	int reference = 0;
	double itae = 0;

	float ITAE[100];
	float ERROR[100];

	float Pgain;
	float Igain;
	float Dgain;

    char mode; // tuning mode selection
    float diff; // tuning difference between gains
	int number_case=0;

	softPwmCreate(MOTOR1, 0, 100);
	softPwmCreate(MOTOR2, 0, 100);

	wiringPiISR(ENCODER_A, INT_EDGE_BOTH, funENCODER_A);
	wiringPiISR(ENCODER_B, INT_EDGE_BOTH, funENCODER_B);

//#################     tuning code     ########################
    printf("Enter mode for tuning: ");
    scanf("%c", &mode);
    printf("Enter Pgain, Igain, Dgain");
    scanf("%f %f %f", &Pgain, &Igain, &Dgain);
    printf("Enter number of iterations:");
    scanf("%d", &iterations);
    printf("Enter difference between %c gains", mode);
    scanf("%f", &diff);
    PID_Tuning(reference, mode, iterations, diff, Pgain, Igain, Dgain);

//#################     project code     ########################
//    printf("Enter the reference position\n");
//    scanf("%d", &reference);
//    PID_Control(reference, Pgain, Igain, Dgain);
//    printf("measured itae(iteration: %d): %f\n", i + 1, itae); // iteration term i

    return 0;
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
		if (endtime - startTime >= 7000) // perform pid control for 7 seconds
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

void PID_Tuning(int reference, char mode, int iteration, float diff ,float Pgain, float Igain, float Dgain) {
    float min_itae = 1e+20; // infinity
    float min_Pgain = 0.0;
    float min_Igain = 0.0;
    float min_Dgain = 0.0;
    switch(mode){
        case 'P' :
            printf("Tuning P_gain\n");
            for (int i = 0; i < iteration; i++) {
                double itae = PID_Control(reference, Pgain, Igain, Dgain);
                // csv.write(Pgain, Igain, Dgain, itae);
                if (itae < min_itae) {
                    min_itae = itae;
                    min_Pgain = Pgain;
                    min_Igain = Igain;
                    min_Dgain = Dgain;
                }
                Pgain += diff;
            }
            printf("==================================================\n");
            printf("minimum itae: %lf (Pgain: %f, Dgain: %f, Igain: %f\n");
            break;
        case 'I' :
            printf("Tuning I_gain\n");
            break;
        case 'D' :
            printf("Tuning D_gain\n");
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

