#define DEBUG Serial

#define FL_CENTRE_WHITE_DIFF 10
#define FL_NEAR_BLACK_DIFF 10
#define FL_FAR_BLACK_DIFF 10

#define LEFT_MOTOR_OFFSET 0
#define RIGHT_MOTOR_OFFSET 0

#define TURN_TIME_DELAY 300  //
//-----------------------------------------------------
typedef struct
{
	uint8_t a, b, pwm;
	int16_t offset;
} motor;

typedef struct 
{
	uint8_t r2, r1, m0, l1, l2;
} lightPin;

typedef struct
{
	bool r2;
	bool r1;
	bool m0;
	bool l1;
	bool l2;
} lightValue;

typedef struct {
    uint8_t signalA;
    uint8_t signalB;
    uint32_t code;
} coderPin;

//-----------------------------------------------------
const motor motorL={11,12,13,LEFT_MOTOR_OFFSET};
const motor motorR={10,8,9,RIGHT_MOTOR_OFFSET};
const lightPin lightFront={3,4,5,6,7};
lightValue light;
uint8_t lineNum = 0;
int motorSpeed = 0;  //全局速度控制
uint8_t step = 0;
bool isStepDone = false;
//-----------------------------------------------------
void lightInit(const lightPin light)
{
	pinMode(light.l2,INPUT);
	pinMode(light.l1,INPUT);
	pinMode(light.m0,INPUT);
	pinMode(light.r1,INPUT);
	pinMode(light.r2,INPUT);
}

void lightUpdate(lightValue lightX)
{
	light.l2 = digitalRead(lightX.l2);
	light.l1 = digitalRead(lightX.l1);
	light.m0 = digitalRead(lightX.m0);
	light.r1 = digitalRead(lightX.r1);
	light.r2 = digitalRead(lightX.r2);

#ifdef DEBUG
	DEBUG.print(lightX.l2);
	DEBUG.print(' ');
	DEBUG.print(lightX.l1);
	DEBUG.print(' ');
	DEBUG.print(lightX.m0);
	DEBUG.print(' ');
	DEBUG.print(lightX.r1);
	DEBUG.print(' ');
	DEBUG.print(lightX.r2);
	DEBUG.println(' ');
#endif
}

void motorInit(motor motorX)
{
	pinMode(motorX.a,   OUTPUT);
	pinMode(motorX.b,   OUTPUT);
	pinMode(motorX.pwm, OUTPUT);

	digitalWrite(motorX.a, LOW);
	digitalWrite(motorX.b, LOW);
}

void motorSet(const motor motorX, int power)
{
	if(power == 0)
	{
		digitalWrite(motorX.a, LOW);
		digitalWrite(motorX.b, LOW);    
	}
	else if(power > 0)
	{
	    digitalWrite(motorX.a, HIGH);
	    digitalWrite(motorX.b, LOW);    
	}
	else
	{
	    digitalWrite(motorX.a, LOW);
	    digitalWrite(motorX.b, HIGH);      
	}
	analogWrite(motorX.pwm, abs(power)*10 + motorX.offset);
}

void motorStop(const motor motorX)
{
	digitalWrite(motorX.a,HIGH);
	digitalWrite(motorX.b,HIGH);  
}

void moveStop()
{
	motorStop(motorL);
	motorStop(motorR);
}

void movePower(int powerLeft, int powerRight)
{
	motorSet(motorL, powerLeft + motorSpeed);
	motorSet(motorR, powerRight + motorSpeed);
}

void moveLeft()
{
	motorSet(motorL, -100);
	motorSet(motorR,  100);
	delay(TURN_TIME_DELAY);
	moveStop();
}

void moveRight()
{
	motorSet(motorL,  100);
	motorSet(motorR, -100);
	delay(TURN_TIME_DELAY);
	moveStop();
}

void moveForward(int power)
{
	power += motorSpeed;
	motorSet(motorL, power);
	motorSet(motorR, power);
}

bool compareLight(uint8_t in)
{
	uint8_t s = (light.l2*16 + light.l1*8 + light.m0*4 + light.r1*2 + light.r2);
	if(in == s)
		return true;
	else
		return false;
}

bool checkLine()
{
	delay(10);
	if(compareLight(0b11111))
	{
		delay(10);
		if(compareLight(0b11111))
			lineNum++;
	}
}

void checkSwitch()  //策略选择器
{
	uint8_t s = (light.l2*16 + light.l1*8 + light.m0*4 + light.r1*2 + light.r2);
	switch(s)
	{
		case 0b00000: moveForward(80);  break;
		case 0b11111: checkLine(); break;
		case 0b01110: moveForward(80);  break;
		case 0b00100: moveForward(80); break;

		case 0b00111: movePower( 50,-30); break;
		case 0b00011: movePower( 50,-30); break;
		case 0b00101: movePower( 70,  0); break;
		case 0b00001: movePower( 70,  0); break;
		case 0b00110: movePower( 80, 30); break;
		case 0b00010: movePower( 80, 30); break;

		case 0b11100: movePower(-30, 50); break;
		case 0b11000: movePower(-30, 50); break;
		case 0b10100: movePower(  0, 70); break;
		case 0b10000: movePower(  0, 70); break;
		case 0b01100: movePower( 30, 80); break;
		case 0b01000: movePower( 30, 80); break;
	}
}
//-----------------------------------------------------
void setup()
{
	Serial.begin(9600);
	lightInit(lightFront);
	motorInit(motorL);
	motorInit(motorR);
}

void loop()
{
	lightUpdate(light);
	checkSwitch();
	switch(step)  //流程步 变量触发
	{
		case 0:
		{
			if(!isStepDone)
			{

			}  //
			step++;
			break;
		}
		default: moveStop();
	}
}



