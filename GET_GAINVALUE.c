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
	printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition);
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
	printf("funcEncoder_A Result: encPos:%d gearPos:%f\n", encoderPosition, redGearPosition);

}

void set_amplifer(float m, float error);
void PID_Control(int reference, int Pgain, int Igain, int Dgain, double* itae, float ERROR[], float ITAE[]);


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

	int number_case=0;

	softPwmCreate(MOTOR1, 0, 100);
	softPwmCreate(MOTOR2, 0, 100);

	wiringPiISR(ENCODER_A, INT_EDGE_BOTH, funENCODER_A);
	wiringPiISR(ENCODER_B, INT_EDGE_BOTH, funENCODER_B);

	printf("Enter the number of iterations\n");
	scanf("%d", &iterations);

	for (int D = 0; D <= 20; D++)
	{
		Dgain = D;
		for (int I = 0; I < 20; I++)
		{
			Igain = I * 0.1;
			for (int P = 1; P <= 20; P++)
			{
				Pgain = P * 50;

				for (int i = 0; i < iterations; i++)
				{
					
					printf("Enter the reference position\n");
					scanf("%d", &reference);
					PID_Control(reference, Pgain, Igain, Dgain, &itae,ERROR,number_case);
					printf("measured itae(iteration: %d): %f\n", i + 1, itae);
					number_case++;
				}

			}
		}

	}

	for (int i = 0; i < (sizeof(ERROR) / sizeof(float)); i++)
	{
		printf("%f\t%f", ERROR[i], ITAE[i]);
	}	

	return 0;
}



void PID_Control(int reference, int Pgain, int Igain, int Dgain, double* itae,float ERROR[],float ITAE[],int number_case)
{

	float m = 0;
	float m1 = 0;
	float error = 0;
	float curr_pos = 0;
	float error_1 = 0, error_2 = 0;
	float T = 0.01;

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
		*itae += (inner_endTime - startTime) / 1000 * fabs(error) * T;

		unsigned int endtime = millis();
		if (endtime - startTime >= 7000)
		{
			softPwmWrite(MOTOR1, 0);
			softPwmWrite(MOTOR2, 0);
			printf("PID CONTROL FINISH\n");
			ERROR[number_case] = error;
			ITAE[number_case] = *itae;
			break;
		}

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

