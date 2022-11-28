#include <TinyStepper_28BYJ_48.h>
//https://github.com/Stan-Reifel/TinyStepper_28BYJ_48
#include <Servo.h>
#define STEPS_PER_TURN  (2048)  //步进电机一周步长 2048步转360度
#define SPOOL_DIAMETER  (35)    //线轴直径mm
#define SPOOL_CIRC      (SPOOL_DIAMETER * 3.1416)  //线轴周长 35*3.14=109.956
#define TPS             (SPOOL_CIRC / STEPS_PER_TURN)  //步进电机步距

#define step_delay      1   //步进电机每步的等候时间 （微妙）
#define TPD             300   //转弯等待时间（毫秒），由于惯性笔会继续运动，暂定等待笔静止再运动。


//两个电机的旋转方向  1正转  -1反转  
//调节进出方向可垂直反转图像
#define M1_REEL_OUT     1     //放出线
#define M1_REEL_IN      -1      //卷入线
#define M2_REEL_OUT     -1      //放出线
#define M2_REEL_IN      1     //卷入线


static long laststep1, laststep2; //当前线长度 记录笔位置
#define X_SEPARATION  480           //The horizontal distance above the two ropes (mm 
#define LIMXMAX       ( X_SEPARATION*0.5)   //x-axis maximum value 0 bit in the center of the drawing board
#define LIMXMIN       (-X_SEPARATION*0.5)   //x-axis minimum

/* Vertical distance parameters: positive values under the drawing board, theoretically as long as the drawing board is large enough can be infinite, negative value area in the pen (before opening) above 
*/
#define LIMYMAX         (-240)   //y轴最大值 画板最下方
#define LIMYMIN         (240)    //y轴最小值 画板最上方  左右两线的固定点到笔的垂直距离，尽量测量摆放准确，误差过大会有畸变

//抬笔舵机的角度参数  具体数值要看摆臂的安放位置，需要调节
#define PEN_UP_ANGLE    50  //抬笔
#define PEN_DOWN_ANGLE  140  //落笔
#define PEN_DOWN 1  //笔状态  下笔
#define PEN_UP 0    //笔状态  抬笔


struct point { 
  float x; 
  float y; 
  float z; 
};

struct point actuatorPos;

// plotter position 笔位置.
static float posx;
static float posy;
static float posz;  // pen state
static float feed_rate = 0;

// pen state 笔状态（抬笔，落笔）.
static int ps;

/*以下为G代码通讯参数 */
#define BAUD            (115200)    //串口速率，用于传输G代码或调试 可选9600，57600，115200 或其他常用速率
#define MAX_BUF         (64)      //串口缓冲区大小

Servo pen;

TinyStepper_28BYJ_48 m1; //(7,8,9,10);  //M1 L步进电机   in1~4端口对应UNO  7 8 9 10
TinyStepper_28BYJ_48 m2; //(2,3,5,6);  //M2 R步进电机   in1~4端口对应UNO 2 3 5 6


//------------------------------------------------------------------------------
//正向运动计算 - 将L1，L2长度转换为XY坐标
// 使用余弦定律， theta = acos((a*a+b*b-c*c)/(2*a*b));
//找到M1M2和M1P之间的角度，其中P是笔的位置
void FK(float l1, float l2,float &x,float &y) {
  float a=l1 * TPS;
  float b=X_SEPARATION;
  float c=l2 * TPS;
  
   
  
  //方法1
  float theta = acos((a*a+b*b-c*c)/(2.0*a*b));
  x = cos(theta)*l1 + LIMXMIN;
  y = sin(theta)*l1 + LIMYMIN;          

  //方法2
/*   float theta = (a*a+b*b-c*c)/(2.0*a*b);
  x = theta*l1 + LIMXMIN;
  y = sqrt (1.0 - theta * theta ) * l1 + LIMYMIN;*/
}


//------------------------------------------------------------------------------
//反向运动 - 将XY坐标转换为长度L1，L2 
void IK(float x,float y,long &l1, long &l2) {
  float dy = y - LIMYMIN;
  float dx = x - LIMXMIN;
  l1 = round(sqrt(dx*dx+dy*dy) / TPS);
  dx = x - LIMXMAX;
  l2 = round(sqrt(dx*dx+dy*dy) / TPS);
}



//------------------------------------------------------------------------------
//笔状态
void pen_state(int pen_st) {
  if(pen_st==PEN_DOWN) {
        ps=PEN_DOWN_ANGLE;
        // Serial.println("Pen down");
      } else {
        ps=PEN_UP_ANGLE;
        //Serial.println("Pen up");
      }
  pen.write(ps);
}


//
void pen_down()
{
  if (ps==PEN_UP_ANGLE)
  {
    ps=PEN_DOWN_ANGLE;
    pen.write(ps);
    delay(TPD);
  }

}

void pen_up()
{
  if (ps==PEN_DOWN_ANGLE)
  {
    ps=PEN_UP_ANGLE;
    pen.write(ps);
  }

  
}

//------------------------------------------------------------------------------
//调试代码串口输出机器状态
void where() {
  Serial.print("X,Y=  ");
  Serial.print(posx);
  Serial.print(",");
  Serial.print(posy);
  Serial.print("\t");
  Serial.print("Lst1,Lst2=  ");
  Serial.print(laststep1);
  Serial.print(",");
  Serial.println(laststep2);
  Serial.println("");
}



//------------------------------------------------------------------------------
// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}



//------------------------------------------------------------------------------
// instantly move the virtual plotter position
// does not validate if the move is valid
static void teleport(float x, float y) {
  posx = x;
  posy = y;
  long l1,l2;
  IK(posx, posy, l1, l2);
  laststep1 = l1;
  laststep2 = l2;
}


//==========================================================
//参考————斜线程序
void moveto(float x,float y) {
  #ifdef VERBOSE
    Serial.println("Jump in line() function");
    Serial.print("x:");
    Serial.print(x);
    Serial.print(" y:");
    Serial.println(y);
  #endif

  long l1,l2;
  IK(x,y,l1,l2);
  long d1 = l1 - laststep1;
  long d2 = l2 - laststep2;

  #ifdef VERBOSE
    Serial.print("l1:");
    Serial.print(l1);
    Serial.print(" laststep1:");
    Serial.print(laststep1);
    Serial.print(" d1:");
    Serial.println(d1);
    Serial.print("l2:");
    Serial.print(l2);
    Serial.print(" laststep2:");
    Serial.print(laststep2);
    Serial.print(" d2:");
    Serial.println(d2);
  #endif

  long ad1=abs(d1);
  long ad2=abs(d2);
  int dir1=d1>0 ? M1_REEL_IN : M1_REEL_OUT;
  int dir2=d2>0 ? M2_REEL_IN : M2_REEL_OUT;
  long over=0;
  long i;


  if(ad1>ad2) {
    for(i=0;i<ad1;++i) {
      
      m1.moveRelativeInSteps(dir1);
      over+=ad2;
      if(over>=ad1) {
        over-=ad1;
        m2.moveRelativeInSteps(dir2);
      }
      delayMicroseconds(step_delay);
     }
  } 
  else {
    for(i=0;i<ad2;++i) {
      m2.moveRelativeInSteps(dir2);
      over+=ad1;
      if(over>=ad2) {
        over-=ad2;
        m1.moveRelativeInSteps(dir1);
      }
      delayMicroseconds(step_delay);
    }
  }

  laststep1=l1;
  laststep2=l2;
  posx=x;
  posy=y;

  
}

//------------------------------------------------------------------------------
//长距离移动会走圆弧轨迹，所以将长线切割成短线保持直线形态
static void line_safe(float x,float y) {
  // split up long lines to make them straighter?
  float dx=x-posx;
  float dy=y-posy;

  float len=sqrt(dx*dx+dy*dy);
  
  if(len<=TPS) {
    moveto(x,y);
    return;
  }
  
  // too long!
  long pieces=floor(len/TPS);
  float x0=posx;
  float y0=posy;
  float a;
  for(long j=0;j<=pieces;++j) {
    a=(float)j/(float)pieces;
    moveto((x-x0)*a+x0,(y-y0)*a+y0);
  }
  moveto(x,y);
}



//=======================================================================================
//------------------------------------------------------------------------------
static void help() {
  Serial.println(F("== Wall Drawing Machine https://github.com/shihaipeng03/Walldraw =="));
  Serial.println(F(" "));
  
}


void line(float x,float y) 
{
  line_safe(x,y);
}


//y=kx+b
float x[201]={};
float y[201]={};

void Linear(float k,float b)
{
  moveto(-100,(-k*100+b));
  pen_down();
  for(int i=-100;i<=100;i++)
  {
  x[i]=i;
  y[i]=k*x[i]+b;
  line(x[i],y[i]);
  }
  pen_up();
  moveto(0,0);
}

//y=ax**2+bx+c


void Quadratic(float a,float b,float c)
{
  moveto(-100,(a*100*100-b*100+c)/100);
  pen_down();
  for(int i=-100;i<=100;i++)
  {
  x[i]=i;
  y[i]=(a*x[i]*x[i]+b*x[i]+c)/100;
  line(x[i],y[i]);
  }
  pen_up();
  moveto(0,0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);
  m1.connectToPins(7,8,9,10); //M1 L步进电机   in1~4端口对应UNO  7 8 9 10
  m2.connectToPins(2,3,5,6);  //M2 R步进电机   in1~4端口对应UNO 2 3 5 6
  m1.setSpeedInStepsPerSecond(10000);
  m1.setAccelerationInStepsPerSecondPerSecond(100000);
  m2.setSpeedInStepsPerSecond(10000);
  m2.setAccelerationInStepsPerSecondPerSecond(100000);

  //抬笔舵机
  pen.attach(A0);
  ps=PEN_UP_ANGLE;
  pen.write(ps);

  //将当前笔位置设置为0，0
  teleport(0, 0);

  pen_up();
  delay(3000);
  Serial.println("Test OK!");
  Quadratic(1,0,-5000);
  //Linear(0.75,-20);
  pen_up();
  moveto(0,0);
}




void loop() {


}



void demo(){
  while(1){
   pen_up();
   delay(3000);
   pen_down();
   delay(3000);
  }
  }
