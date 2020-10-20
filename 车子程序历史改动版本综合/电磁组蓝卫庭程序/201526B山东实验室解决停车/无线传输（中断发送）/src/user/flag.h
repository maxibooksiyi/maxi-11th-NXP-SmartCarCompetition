uint32 stoptime,runtime=40000;
int8  goflag,runflag;
int8  stopflag,stopflag1,stopflag2;

uint32  motor_protect_cnt;      //��ת��ʱ

int8    start_line_flag=0;//��⵽�����߱�־
uint32  irq_time,irq_cnt;//�����жϵ�ʱ�䣬�жϼ���
uint32  stop_car_distance,stop_cnt=0;

uint32 chaoshengboTime=0;//��ȡ���ĳ�����ʱ��,��λ΢��
uint32 ABDistance = 0;//�����ķ��ͽ���ģ��ľ���,��λ����
/*********************�����ʼ������******************/

int16 turn_add,old_turn_add; 
int16 steer_mid;                        //����������ֵ
/*********************��е�ѹ�ɼ��͹�һ��******************/
#define   error_historyN  3
#define   NM              7
#define   MAX_ERROR_SUM   30             //���ַ�ֵ
int16     AD_valu[7],AD_V[7][NM];      //
int16     AD_history[7][40],AD_history_dif[7][39],AD_dif_sum[7];
float    ADv[7];   
int16     AD_sum[7];
int16     ADC[7];
float    ADre[7];
uint16    AD[7]; 
int16     max_v[7];
int16     min_v[7];
int16     maxV[7];//���¹�һ�������ֵ
float    sensor_to_one[7];             //��һ�����˲���������
int16     mavAD=200,flagAD=110;
/*********************��й�һ�������ݷ���******************/
//�м��ŵ��2,3,4
int16 value42s,value10s,value65s;
int16 error_1[40],error1,preerror1lp,error1lp,dterror1,dterror2,dterror3,predt;//�м��ŵ��ƫ��洢����ǰƫ�ǰһ��ƫ����ϣ�����ƫ����ϣ�ƫ��仯��
int16 errorsum1[4];//ƫ������ֶ�ʮ�����
int16 errorave1[4];
int16 dtsum1[3];//ƫ��͵ıȽ�
int16 preerror1;
//int16 error_1_ave[4];
//��һ�ŵ��6��0,5��1,7
int16 error_0[40],error0,preerror0lp,error0lp,dterror0,preerror0;
int16 error_2[20],error2,dterror2;
int16 error_1_ave[2]={0},error_2_ave[2]={0};
int16 last_error1,last_error2,last_error;
int16 errorsum0[4];//ƫ������ֶ�ʮ�����
int16 errorave0[4];
int16 dtsum0[3];//ƫ��͵ıȽ�
int16 preerror0;
int16 error_0_ave[4];
int16 errorend,dterrorend,preend,predt,error;
//�ٶȵķ�����Ӱ��ƫ��ϵ��
float fbve,fbAg;
int16  fb,fbvn;
/*********************�����������******************/
int8  shizi,xiezi;
int8  zhidao,zhijiaoL,zhijiaoR,difference;
int8  state;
/*********************�������PID******************/
int16   skp=22,skd=170;
int16   skp1,skp2,skp3;
float   servo_kp,servo_kd;
float   kp=0.05,kd=10;
int16   Angle,Angle_history[20],dtAngle;
int16   servo_PWM=0;
int16   prepwm,dtspwm;
int16   Ud,Ud_1;//΢�ֻ��ڵ������ٽ�ֵ
int16   Up;//��������
/*********************����ٶ�******************/
uint16  speedchoose[100]={100,105,110,115,120,125,130,135,140,145,150,155,160,165,170,175,180,185,190,
                         200,205,210,215,220,225,230,235,240,245,250,255,260,265,270,275,280,285,290,
                         300,305,310,315,320,325,330,335,340,345,350,355,360,365,370,375,380,385,390,};//�ٶȵ�λ
int   speedve=250;
int   ve_ramp=220;
int   right_angle_v=230;
int16   SPN=18;
int16   motor2s;
int16   motorout,premotorout;
int16   motorpw;

/*********************����PI����******************/
float  mkp=230;//��������ʽPI
float  mki=11;//��������ʽPI
/*********************λ��PID����******************/
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

int16   max_Expect_distance=60,min_Expect_distance=30;//��������������30cm
int16   ABerror,dtABerror,preABerror;//��ǰ����ƫ��
int16   ABkp,ABkd;
float   disfb;//���뷴��
/*********************���ߴ��ڷ�������******************/
float    senddata[8];                  //��������
int8 send_flag=0;
unsigned char Data_OutPut_Buffer[42];
/*********************��ȡ��ص�ѹ�͵������******************/
#define Powerhnum 20//��ص�ѹ���
#define power0 3300//��׼��ѹ����Ϊ����������������ֵ�²��
float power;
u16 powerh[Powerhnum];
u16 powerIh[Powerhnum];
u8  powerpoint;
float  powerU,powerI,dianliu;
int16  powerUvalue;
int16  powerIvalue,Ivalue,preIvalue,dtIvalue[20],sumI;
int8  countI;
/*********************��ȡ�ٶ�******************/
#define Vnhnum 20 // �ٶȼ�¼
float  ve;         //�����ٶȵ�����
float  vn;          //�����ٶ�
int16  vnerr;       //��ǰ�ٶ�ƫ��
int16  prevnerr;    //��һ���ٶ�ƫ��
int16  dtvn;       //�����ٶ����仯��
int16  predtvn;  //�ϴ��ٶ����仯��
int16  vnsum;//�ٶ�ƫ�����
int16  vnh[Vnhnum]; //�ٶȱ���
int8   vnpoint;      //�ٶȱ���ָ��
float vnk;         //�����˲�����ٶȻ�ƿ������˲�
float vnkk=0.3;    //����ϵ����ƿ���������
/*********************�ٶ��趨******************/
int16  minspeed,maxspeed;



/*********************����******************/
#define change_page    (bool)(GPIOD_PDIR >> 11 & 0x00000001)
#define change_line    (bool)(GPIOD_PDIR >> 10 & 0x00000001)
#define Sub_10         (bool)(GPIOD_PDIR >> 9 & 0x00000001)
#define Add_10         (bool)(GPIOD_PDIR >> 8 & 0x00000001)
#define Sub_1          (bool)(GPIOD_PDIR >> 7 & 0x00000001)
#define Add_1          (bool)(GPIOD_PDIR >> 6 & 0x00000001)

unsigned char page_num=0;     //ҳ���
unsigned char line_num=0;     //�����
/*********************�������******************/
float k,b;//�������ϵ��
/*****************���Գ����м����**************/
int8 area,last_area,area_time,area_flag,area1,area2;
const double five_times[5]={0.6,0.4,0.3,0.2,0.1};

/*******************��־λ**********************/
int8 right_angle_flag=0,slopup_flag=0;
int flag_cnt=0;

/************************���ٶ�mma8451��������mpu6050**************************/
bool i2cdisable=0;
#define Mmahnum 50   //���ٶȱ������
int16 mma0x;         //���Ư��ֵ
int16 mma0y;
int16 mma0z;
int16 mmax[Mmahnum];  //��ʷֵ
int16 mmay[Mmahnum];
int16 mmaz[Mmahnum];
double mmaxn;  //�˲�ֵ
double mmayn;
double mmazn=4096;
double mmak=0.01;
uint8 mmapoint;       //���ٶ�ָ��
float mma;
float mmatime=0;

#define Gyhnum 50//���ٶ�
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

double Distance;//���˶�·��
double Direction;//��ת���ĽǶ�
double stopdistance;//�յ�ͣ������

int8 enablestop,enstop;
int8  ramp,ramp1=0,ramp2=0,ramp3=0;//�µ���ʶ
int16 ramptime;//�µ�ʱ��
int16 ramp_delay=3000;  //�����µ���ʱ
float ramp_start=0;         //�µ���ʼ·��
float ramp_distance=2000;  //�µ�����2m
uint32 ramp_over_time=0;

float zhijiao_start;   //ֱ�ǿ�ʼ�Ƕ�

float xieshi_front_start;
float xieshi_behind_start;
/*******************************бʮ�ֲ���*************************/
float slope1=1,slope2=1;

int16  countstop=0;//
int16  stopflagdelay;