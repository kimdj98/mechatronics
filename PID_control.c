#include <stdio.h>
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


int main()
{
	wiringPiSetupGpio();
	pinMode(ENCODER_A, INPUT);
	pinMode(ENCODER_B, INPUT);

	wiringPiISR(ENCODER_A, INT_EDGE_BOTH, funENCODER_A);
	wiringPiISR(ENCODER_B, INT_EDGE_BOTH, funENCODER_B);	

	return 0;
}

