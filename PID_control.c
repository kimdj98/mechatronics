#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>

#define ENCODER_A 17
#define ENCODER_B 27
#define ENC2REDGEAR 216
#define MOTOR1 19
#define MOTOR2 26

int encA;
int encB;
int encoderPosition = 0;
float redGearPosition = 0;

void funENCODER_A() //execute this function if there is a digital change in sensor A
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


void funENCODER_B() //execute this function if there is a digital change in sensor B
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

<<<<<<< HEAD
float get_position()
{
	return redGearPosition;
}

void MOVING_MOTOR(float actu_signal) // integer signal?
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
=======
void set_amplifer(float m, float error); //prototype
void PID_Control(int reference, int Pgain, int Igain, int Dgain, double* itae); //prototype
>>>>>>> ca33d4c31db0955a3fc999646c90194b91960757


int main()
{
	wiringPiSetupGpio();
	pinMode(ENCODER_A, INPUT);
	pinMode(ENCODER_B, INPUT);

	int iterations = 0;
	int reference = 0;
	double itae = 0;

	float Pgain = 100; //define gain values
	float Igain = 0.1;
	float Dgain = 1;

	softPwmCreate(MOTOR1, 0, 100);
	softPwmCreate(MOTOR2, 0, 100);

	wiringPiISR(ENCODER_A, INT_EDGE_BOTH, funENCODER_A); //interrupt
	wiringPiISR(ENCODER_B, INT_EDGE_BOTH, funENCODER_B);

	printf("Enter the number of iterations\n");
	scanf("%d", &iterations);

	for (int i = 0; i < iterations; i++)
	{
		printf("Enter the reference position\n");
		scanf("%d", &reference);
		PID_Control(reference, Pgain, Igain, Dgain, &itae);
		printf("measured itae(iteration: %d): %f\n", i + 1, itae);
	}

	return 0;
}



void PID_Control(int reference, int Pgain, int Igain, int Dgain, double* itae)
{

	float m = 0;
	float m1 = 0;
	float error = 0;
	float curr_pos = 0;
	float error_1 = 0, error_2 = 0; 
	float T = 0.01; // set sampling time as 0.01s

	float G1 = (Pgain + Igain * T + Dgain / T); // discretize PID equation in time space
	float G2 = -(Pgain + 2 * Dgain / T);
	float G3 = Dgain / T;

	unsigned int startTime = millis(); // start recording time to keep of track 't' at each sampling time unit


	while (1)
	{
		unsigned int inner_startTime = millis();

		curr_pos = redGearPosition;
		error = (float)reference - curr_pos; // calculate error

		m = m1 + G1 * error + G2 * error_1 + G3 * error_2; // calculate actuating signal

		printf("Error Value: %f\n", error); 
		printf("Actuating signal Value: %f\n", m);

		set_amplifer(m, error); // move the DC motor in actuating signal value

		error_2 = error_1; // update the errors
		error_1 = error;
		m1 = m;

		unsigned int inner_endTime = millis();

		while (inner_endTime - inner_startTime < T * 1000) // delay to make exact 0.01s sampling time
		{
			inner_endTime = millis();
		}
		*itae += (inner_endTime - startTime) / 1000 * fabs(error) * T; // calculated itae

		unsigned int endtime = millis();
		if (endtime - startTime >= 5000) // if 5 seconds pass, finish PID control
		{
			softPwmWrite(MOTOR1, 0);
			softPwmWrite(MOTOR2, 0);
			printf("PID CONTROL FINISH\n");
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

