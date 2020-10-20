uint32 stoptime,runtime=40000;
int8  goflag,runflag;
int8  stopflag,stopflag1,stopflag2;

uint32  motor_protect_cnt;      //堵转计时

int8    start_line_flag=0;//检测到起跑线标志
uint32  irq_time,irq_cnt;//发生中断的时间，中断计数
uint32  stop_car_distance,stop_cnt=0;

uint32 chaoshengboTime=0;//读取到的超声波时间,单位微秒
uint32 ABDistance = 0;//换算后的发送接收模块的距离,单位毫米
/*********************舵机初始化调节******************/

int16 turn_add,old_turn_add; 
int16 steer_mid;                        //调整后舵机中值
/*********************电感电压采集和归一化******************/
#define   error_historyN  3
#define   NM              7
#define   MAX_ERROR_SUM   30             //积分幅值
int16     AD_valu[7],AD_V[7][NM];      //
int16     AD_history[7][40],AD_history_dif[7][39],AD_dif_sum[7];
float    ADv[7];   
int16     AD_sum[7];
int16     ADC[7];
float    ADre[7];
uint16    AD[7]; 
int16     max_v[7];
int16     min_v[7];
int16     maxV[7];//重新归一化的最大值
float    sensor_to_one[7];             //归一化，滤波，防干扰
int16     mavAD=200,flagAD=110;
/*********************电感归一化后数据分析******************/
//中间排电感2,3,4
int16 value42s,value10s,value65s;
int16 error_1[40],error1,preerror1lp,error1lp,dterror1,dterror2,dterror3,predt;//中间排电感偏差存储，当前偏差，前一次偏差拟合，本次偏差拟合，偏差变化率
int16 errorsum1[4];//偏差数组分段十个求和
int16 errorave1[4];
int16 dtsum1[3];//偏差和的比较
int16 preerror1;
//int16 error_1_ave[4];
//第一排电感6，0,5，1,7
int16 error_0[40],error0,preerror0lp,error0lp,dterror0,preerror0;
int16 error_2[20],error2,dterror2;
int16 error_1_ave[2]={0},error_2_ave[2]={0};
int16 last_error1,last_error2,last_error;
int16 errorsum0[4];//偏差数组分段十个求和
int16 errorave0[4];
int16 dtsum0[3];//偏差和的比较
int16 preerror0;
int16 error_0_ave[4];
int16 errorend,dterrorend,preend,predt,error;
//速度的反馈，影响偏差系数
float fbve,fbAg;
int16  fb,fbvn;
/*********************赛道情况处理******************/
int8  shizi,xiezi;
int8  zhidao,zhijiaoL,zhijiaoR,difference;
int8  state;
/*********************舵机调节PID******************/
int16   skp=22,skd=170;
int16   skp1,skp2,skp3;
float   servo_kp,servo_kd;
float   kp=0.05,kd=10;
int16   Angle,Angle_history[20],dtAngle;
int16   servo_PWM=0;
int16   prepwm,dtspwm;
int16   Ud,Ud_1;//微分环节的两个临近值
int16   Up;//比例环节
/*********************电机速度******************/
uint16  speedchoose[100]={100,105,110,115,120,125,130,135,140,145,150,155,160,165,170,175,180,185,190,
                         200,205,210,215,220,225,230,235,240,245,250,255,260,265,270,275,280,285,290,
                         300,305,310,315,320,325,330,335,340,345,350,355,360,365,370,375,380,385,390,};//速度档位
int   speedve=250;
int   ve_ramp=220;
int   right_angle_v=230;
int16   SPN=18;
int16   motor2s;
int16   motorout,premotorout;
int16   motorpw;

/*********************增量PI控制******************/
float  mkp=230;//用于增量式PI
float  mki=11;//用于增量式PI
/*********************位置PID控制******************/
float  mokp=20;
float  moki=2;
float  mokd=40;
float  mobase;
float  mokb=3.5;
float  mobasev;
float  moimax=300;
float  motorP;
float  motorI;
float  motorD;

int16   max_Expect_distance=60,min_Expect_distance=30;//期望的两车距离30cm
int16   ABerror,dtABerror,preABerror;//当前距离偏差
int16   ABkp,ABkd;
float   disfb;//距离反馈
/*********************无线串口发送数据******************/
float    senddata[8];                  //发送数据
int8 send_flag=0;
unsigned char Data_OutPut_Buffer[42];
/*********************获取电池电压和电机电流******************/
#define Powerhnum 20//电池电压相关
#define power0 3300//基准电压，因为电机的特性是在这个值下测的
float power;
u16 powerh[Powerhnum];
u16 powerIh[Powerhnum];
u8  powerpoint;
float  powerU,powerI,dianliu;
int16  powerUvalue;
int16  powerIvalue,Ivalue,preIvalue,dtIvalue[20],sumI;
int8  countI;
/*********************获取速度******************/
#define Vnhnum 20 // 速度记录
float  ve;         //现在速度的期望
float  vn;          //现在速度
int16  vnerr;       //当前速度偏差
int16  prevnerr;    //上一次速度偏差
int16  dtvn;       //本次速度误差变化率
int16  predtvn;  //上次速度误差变化率
int16  vnsum;//速度偏差积累
int16  vnh[Vnhnum]; //速度保存
int8   vnpoint;      //速度保存指针
float vnk;         //比例滤波后的速度或称卡尔曼滤波
float vnkk=0.3;    //比例系数或称卡尔曼增益
/*********************速度设定******************/
int16  minspeed,maxspeed;



/*********************按键******************/
#define change_page    (bool)(GPIOD_PDIR >> 11 & 0x00000001)
#define change_line    (bool)(GPIOD_PDIR >> 10 & 0x00000001)
#define Sub_10         (bool)(GPIOD_PDIR >> 9 & 0x00000001)
#define Add_10         (bool)(GPIOD_PDIR >> 8 & 0x00000001)
#define Sub_1          (bool)(GPIOD_PDIR >> 7 & 0x00000001)
#define Add_1          (bool)(GPIOD_PDIR >> 6 & 0x00000001)

unsigned char page_num=0;     //页序号
unsigned char line_num=0;     //行序号
/*********************线性拟合******************/
float k,b;//线性拟合系数
/*****************调试程序中间变量**************/
int8 area,last_area,area_time,area_flag,area1,area2;
const double five_times[5]={0.6,0.4,0.3,0.2,0.1};

/*******************标志位**********************/
int8 right_angle_flag=0,slopup_flag=0;
int flag_cnt=0;

/************************加速度mma8451和陀螺仪mpu6050**************************/
bool i2cdisable=0;
#define Mmahnum 50   //加速度保存个数
int16 mma0x;         //零点漂移值
int16 mma0y;
int16 mma0z;
int16 mmax[Mmahnum];  //历史值
int16 mmay[Mmahnum];
int16 mmaz[Mmahnum];
double mmaxn;  //滤波值
double mmayn;
double mmazn=4096;
double mmak=0.01;
uint8 mmapoint;       //加速度指针
float mma;
float mmatime=0;

#define Gyhnum 50//角速度
int16 gy0x;
int16 gy0y;
int16 gy0z;
int16 gyx[Gyhnum];
int16 gyy[Gyhnum];
int16 gyz[Gyhnum];
uint8 gypoint;
float carturnning;
float gytime=0;

double Gx_omiga,Gx_arfa,Gx_k=0.003;
double Gy_omiga,Gy_arfa,Gy_k=0.003;
double Gx_angle;
double Gy_angle;
double Angle_x,Angle_y;

double Distance;//总运动路程
double Direction;//总转过的角度
double stopdistance;//终点停车距离

int8 enablestop,enstop;
int8  ramp,ramp1=0,ramp2=0,ramp3=0;//坡道标识
int16 ramptime;//坡道时间
int16 ramp_delay=3000;  //发车坡道延时
float ramp_start=0;         //坡道开始路程
float ramp_distance=2000;  //坡道距离2m
uint32 ramp_over_time=0;

float zhijiao_start;   //直角开始角度

float xieshi_front_start;
float xieshi_behind_start;
/*******************************斜十字补偿*************************/
float slope1=1,slope2=1;

int16  countstop=0;//
int16  stopflagdelay;