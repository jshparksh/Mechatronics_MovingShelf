#include <HCMotor.h>
#include <LiquidCrystal.h>
#include <Keypad.h>

/* 모터드라이버 연결핀 */
#define DIR_PIN0 14
#define CLK_PIN0 15

#define DIR_PIN1 18
#define CLK_PIN1 19

#define DIR_PIN2 22
#define CLK_PIN2 23

#define DIR_PIN3 26
#define CLK_PIN3 27


/*encoder rotate*/
#define QNUM 50
#define SSERROR 0

float queue[QNUM] {0,};

/*Rotate num*/
float a = 0;
float b = 0;
float c = 0;
float d = 0;

float s = 0;

int a_rotate = 0;
int b_rotate = 0;
int c_rotate = 0;
int d_rotate = 0;

/*#rotation of motor 1 to move one x*/
int mx = 64; // 62 아니면 63
/*#rotation of motor 2 and 3 to move one y*/
int my = 35; 

/*coordinate*/
char coordinate[2] = {'0', '0'};
int i=0;

/* keypad for number */
const byte rows0 = 4;
const byte cols0 = 3;
byte rowPins0[rows0] = {7, 6, 5, 4};
byte colPins0[cols0] = {3, 2, 1};

char keys0[rows0][cols0] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}, };

/* keypad for alphabet */
const byte rows1 = 4;
const byte cols1 = 1;
byte rowPins1[rows1] = {7, 6, 5, 4};
byte colPins1[cols1] = {0};

char keys1[rows1][cols1] = {
  {'A'},
  {'B'},
  {'C'},
  {'D'},  };

/* LCD and Keypad */
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
Keypad keypad0 = Keypad(makeKeymap(keys0), rowPins0, colPins0, rows0, cols0);
Keypad keypad1 = Keypad(makeKeymap(keys1), rowPins1, colPins1, rows1, cols1);

float P_control,  D_control, PID_control;
float I_control = 0;
float error, prev_error=0;
float kp = 1; //25.2;
float ki = 0; //243.0;
float kd = 0.005;
float samplingtime = 0.004 ;

float PIDControl(float dest, float current)
{
  error = dest - current;
  P_control = kp*error;
  I_control += ki*error*samplingtime;
  D_control = kd*(error - prev_error);
  PID_control = P_control + I_control + D_control;
  prev_error = error;
  if (PID_control > 5)
  {
    return 5;
  }
  return PID_control;
}

/*encoder rotate*/
float getSpeed(int analogPin)
{
  float currentvalue = 0;
  for(int cnt = 0; cnt < QNUM-1; cnt++)
  {
    queue[cnt] = queue[cnt + 1];
    currentvalue += queue[cnt];
  }
  queue[QNUM-1] = (float)analogRead(analogPin) / (float)QNUM;
  currentvalue += queue[QNUM-1];
  
  return currentvalue;
}

/*Encoder data part*/
int Rotate(int m)
{
  if(m == 0)
  {
    if(600 < getSpeed(A0))
    {
      a = getSpeed(A0);
    }
    if(a - getSpeed(A0) > 360)
    {
      a_rotate += 1;
      a = 0;
    }
    return a_rotate;
  }
  
  if(m == 1)
  {
    if(500 < getSpeed(A1))
    {
      b = getSpeed(A1);
    }
    if(b - getSpeed(A1) > 280)
    {
      b_rotate += 1;
      b = 0;
    }
    return b_rotate;
  }
  
  if(m == 2)
  {
    if(500 < getSpeed(A3))
    {
      c = getSpeed(A3);
    }
    if(c - getSpeed(A3) > 280)
    {
      c_rotate += 1;
      c = 0;
    }
    return c_rotate;
  }
  
  if(m == 3)
  {
    if(500 < getSpeed(A3))
    {
      d = getSpeed(A3);
    }
    if(d - getSpeed(A3) > 260)
    {
      d_rotate += 1;
      d = 0;
    }
    return d_rotate;
  }
}

/* HCMotor 라이브러리 인스턴스 생성 */
HCMotor HCMotor;

/*Speed function*/
float Speed(int m, float s) // 0 <= s <= 5
{
  //speed_min = HCMotor.DutyCycle(m, 300);
  //speed_max = HCMotor.DutyCycle(m, 10);
  s = map(s, 0, 5, 20, 10);
  HCMotor.DutyCycle(m, s);
}

/*Direction function*/
float Dir(int m, char d)
{
  if(d == 'F')
  {
    HCMotor.Direction(m, FORWARD);
  }
  else if(d == 'R')
  {
    HCMotor.Direction(m, REVERSE);
  }
} 

/*Stop function*/
float Stop(int m)
{
  HCMotor.DutyCycle(m,0);
}

float Moveto()
{
  if(coordinate[0] == 51 && coordinate[1] == 49)
  {
    return;
  }
  
  else if(coordinate[0] != 51 && coordinate[1] != 49)
  {
    Speed(1, 5);
    Dir(1, 'R');
    while(1)
    {
      if(Rotate(1) > (51 - coordinate[0]) * mx - 5)
      {
        break;
      }
    }
    while(1)
    {
      Speed(1, PIDControl((51 - coordinate[0]) * mx, Rotate(1)));
      if (((51 - coordinate[0]) * mx) - Rotate(1) == 0)
      {
        Stop(1);
        b_rotate = 0;
        break;
      }
    }
    if(coordinate[1] == 53)
    {
      Speed(2, 5);
      Dir(2, 'R');
      Speed(3, 5);
      Dir(3, 'R');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 2 - 5 ||
        Rotate(3) > (coordinate[1] - 49) * my - 2 - 5)
        {          
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my - 2, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my - 2, Rotate(2)));
        if ((coordinate[1] - 49)* my - 2 - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
    else if(coordinate[1] == 52)
    {
      Speed(2, 5);
      Dir(2, 'R');
      Speed(3, 5);
      Dir(3, 'R');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 1 - 5 || Rotate(3) > (coordinate[1] - 49) * my - 1 - 5)
        {
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my - 1, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my - 1, Rotate(2)));
        if ((coordinate[1] - 49)* my - 1 - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
    else
    {
      Speed(2, 5);
      Dir(2, 'R');
      Speed(3, 5);
      Dir(3, 'R');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 5 || Rotate(3) > (coordinate[1] - 49) * my - 5)
        {          
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my, Rotate(2)));
        if ((coordinate[1] - 49)* my - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
  }
  else if(coordinate[0] == 51 && coordinate[1] != 49)
  {
    if(coordinate[1] == 53)
    {
      Speed(2, 5);
      Dir(2, 'R');
      Speed(3, 5);
      Dir(3, 'R');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 2 - 5 || Rotate(3) > (coordinate[1] - 49) * my - 2 - 5)
        {          
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my - 2, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my - 2, Rotate(2)));
        if ((coordinate[1] - 49)* my - 2 - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
    else if(coordinate[1] == 52)
    {
      Speed(2, 5);
      Dir(2, 'R');
      Speed(3, 5);
      Dir(3, 'R');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 1 - 5 || Rotate(3) > (coordinate[1] - 49) * my - 1 - 5)
        {
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my - 1, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my - 1, Rotate(2)));
        if ((coordinate[1] - 49)* my - 1 - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
    else
    {
      Speed(2, 5);
      Dir(2, 'R');
      Speed(3, 5);
      Dir(3, 'R');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 5 || Rotate(3) > (coordinate[1] - 49) * my - 5)
        {          
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my, Rotate(2)));
        if ((coordinate[1] - 49)* my - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
  }
  else if(coordinate[0] != 51 && coordinate[1] == 49)
  {
    Speed(1, 5);
    Dir(1, 'R');
    while(1)
    {
      if(Rotate(1) > (51 - coordinate[0]) * mx - 5)
      {
        break;
      }
    }
    while(1)
    {
      Speed(1, PIDControl((51 - coordinate[0]) * mx, Rotate(1)));
      if (((51 - coordinate[0]) * mx) - Rotate(1) == 0)
      {
        Stop(1);
        b_rotate = 0;
        break;
      }
    }
  }
}

float Moveback()
{
  if(coordinate[0] == 51 && coordinate[1] == 49)
  {
    return;
  }
  
  else if(coordinate[0] != 51 && coordinate[1] != 49)
  {
    Speed(1, 5);
    Dir(1, 'F');
    while(1)
    {
      if(Rotate(1) > (51 - coordinate[0]) * mx - 5)
      {
        break;
      }
    }
    while(1)
    {
      Speed(1, PIDControl((51 - coordinate[0]) * mx, Rotate(1)));
      if (((51 - coordinate[0]) * mx) - Rotate(1) == 0)
      {
        Stop(1);
        b_rotate = 0;
        break;
      }
    }
    if(coordinate[1] == 53)
    {
      Speed(2, 5);
      Dir(2, 'F');
      Speed(3, 5);
      Dir(3, 'F');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 2 - 5 || Rotate(3) > (coordinate[1] - 49) * my - 2 - 5)
        {          
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my - 2, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my - 2, Rotate(2)));
        if ((coordinate[1] - 49)* my - 2 - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
    else if(coordinate[1] == 52)
    {
      Speed(2, 5);
      Dir(2, 'F');
      Speed(3, 5);
      Dir(3, 'F');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 1 - 5 || Rotate(3) > (coordinate[1] - 49) * my - 1 - 5)
        {
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my - 1, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my - 1, Rotate(2)));
        if ((coordinate[1] - 49)* my - 1 - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
    else
    {
      Speed(2, 5);
      Dir(2, 'F');
      Speed(3, 5);
      Dir(3, 'F');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 5 || Rotate(3) > (coordinate[1] - 49) * my - 5)
        {          
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my, Rotate(2)));
        if ((coordinate[1] - 49)* my - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
  }
  else if(coordinate[0] == 51 && coordinate[1] != 49)
  {
    if(coordinate[1] == 53)
    {
      Speed(2, 5);
      Dir(2, 'F');
      Speed(3, 5);
      Dir(3, 'F');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 2 - 5 || Rotate(3) > (coordinate[1] - 49) * my - 2 - 5)
        {          
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my - 2, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my - 2, Rotate(2)));
        if ((coordinate[1] - 49)* my - 2 - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
    else if(coordinate[1] == 52)
    {
      Speed(2, 5);
      Dir(2, 'F');
      Speed(3, 5);
      Dir(3, 'F');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 1 - 5 || Rotate(3) > (coordinate[1] - 49) * my - 1 - 5)
        {
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my - 1, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my - 1, Rotate(2)));
        if ((coordinate[1] - 49)* my - 1 - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
    else
    {
      Speed(2, 5);
      Dir(2, 'F');
      Speed(3, 5);
      Dir(3, 'F');
      while(1)
      {
        if(Rotate(2) > (coordinate[1] - 49)* my - 5 || Rotate(3) > (coordinate[1] - 49) * my - 5)
        {          
          break;
        }
      }
      while(1)
      {
        Speed(2, PIDControl((coordinate[1] - 49)* my, Rotate(2)));
        Speed(3, PIDControl((coordinate[1] - 49)* my, Rotate(2)));
        if ((coordinate[1] - 49)* my - Rotate(2) == 0)
        {
          Stop(2);
          Stop(3);
          c_rotate = 0;
          d_rotate = 0;
          break;
        }
      }
    }
  }
  else if(coordinate[0] != 51 && coordinate[1] == 49)
  {
    Speed(1, 5);
    Dir(1, 'F');
    while(1)
    {
      if(Rotate(1) > (51 - coordinate[0]) * mx - 5)
      {
        break;
      }
    }
    while(1)
    {
      Speed(1, PIDControl((51 - coordinate[0]) * mx, Rotate(1)));
      if (((51 - coordinate[0]) * mx) - Rotate(1) == 0)
      {
        Stop(1);
        b_rotate = 0;
        break;
      }
    }
  }
}


float Pull()
{
  Dir(0, 'F');
  Speed(0, 5);
  while(1)
  {
    if(Rotate(0) == 8)
    {
      Stop(0);
      a_rotate = 0;
      break;
    }
  }
  
  Dir(3, 'R');
  Speed(3, 5);
  Dir(2, 'R');
  Speed(2, 5);
  while(1)
  {
    if(Rotate(2) == 5)
    {
      Stop(2);
      Stop(3);
      c_rotate = 0;
      d_rotate = 0;
      break;
    }
  }
  
  Dir(0, 'R');
  Speed(0, 5);
  while(1)
  {
    if(Rotate(0) == 145)
    {
      Stop(0);
      a_rotate = 0;
      break;
    }
  }
}

float Push()
{
  Dir(0, 'F');
  Speed(0, 5);
  while(1)
  {
    if(Rotate(0) == 145)
    {
      Stop(0);
      a_rotate = 0;
      break;
    }
  }
  
  Dir(3, 'F');
  Speed(3, 5);
  Dir(2, 'F');
  Speed(2, 5);
  while(1)
  {
    if(Rotate(2) == 5)
    {
      Stop(2);
      Stop(3);
      c_rotate = 0;
      d_rotate = 0;
      break;
    }
  }

  Dir(0, 'R');
  Speed(0, 5);
  while(1)
  {
    if(Rotate(0) == 8)
    {
      Stop(0);
      a_rotate = 0;
      break;
    }
  }
}

void setup() 
{
  /* 라이브러리 초기화 */
  HCMotor.Init();

  /* 모터0을 스텝모터로 설정하고 연결된 핀을 지정 */
  HCMotor.attach(0, STEPPER, CLK_PIN0, DIR_PIN0);  // 서랍 꺼내기
  HCMotor.attach(1, STEPPER, CLK_PIN1, DIR_PIN1);  // 좌우
  HCMotor.attach(2, STEPPER, CLK_PIN2, DIR_PIN2);  // 위아래
  HCMotor.attach(3, STEPPER, CLK_PIN3, DIR_PIN3);  // 위아래
  
  HCMotor.Steps(0,CONTINUOUS);
  HCMotor.Steps(1,CONTINUOUS);
  HCMotor.Steps(2,CONTINUOUS);
  HCMotor.Steps(3,CONTINUOUS);

  /*lcd start*/
  lcd.begin(16, 2);
  lcd.print("(x,y):");
}


void loop() 
{
  /*(1,1) ~ (3, 5), (3, 1)이 원점*/
  while(1)
  {
    char key = keypad0.getKey();
    char letter = keypad1.getKey();
    if (key != NO_KEY || letter != NO_KEY)
    {

      if (key >= '1' && key <= '9')
      {
        coordinate[i] = key;
      }
      else if (letter == 'D')
      {
        i=1; //리스트 한 칸 뒤로
      }
      else if (key == '*')
      {
        i=0; // i 리셋
        break;
      }
    }
  }

  while(1)
  {
    char key = keypad0.getKey();
    char letter = keypad1.getKey();
    if (key != NO_KEY || letter != NO_KEY)
    {
      if (key == '#')
      {
        lcd.setCursor(0,1);
        lcd.print("                        ");
        break;
      }
      else if (letter == 'A')
      {
        lcd.setCursor(0,1);
        lcd.print("Moving to");
        lcd.setCursor(11,1);
        lcd.print("(");
        lcd.setCursor(12,1);
        lcd.print(coordinate[0]);
        lcd.setCursor(13,1);
        lcd.print(",");
        lcd.setCursor(14,1);
        lcd.print(coordinate[1]);
        lcd.setCursor(15,1);
        lcd.print(")");
        Moveto();
        lcd.setCursor(0,1);
        lcd.print("                        ");
      }
      else if (letter == 'B')
      {
        lcd.setCursor(0,1);
        lcd.print("Taking out");
        Pull();
        lcd.setCursor(0,1);
        lcd.print("                        ");
      }
      else if (letter == 'C')
      {
        lcd.setCursor(0,1);
        lcd.print("Moving to Orient");
        Moveback();
        lcd.setCursor(0,1);
        lcd.print("                        ");
      }
      else if (letter == 'D')
      {
        lcd.setCursor(0,1);
        lcd.print("Putting in");
        Push();
        lcd.setCursor(0,1);
        lcd.print("                        ");
      }
    }
  }
}
