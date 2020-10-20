/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ���ѧ��̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_gpio.h
 * @brief      gpio����ͷ�ļ�
 * @author     ɽ��Ƽ�
 * @version    v5.1
 * @date       2014-04-25
 */
#ifndef __MK60_GPIO_H__
#define __MK60_GPIO_H__
#include "MK60_gpio_cfg.h"

//�˿ں궨��
typedef enum PORTx
{
    PORTA,
    PORTB,
    PORTC,
    PORTD,
    PORTE
} PORTx;


/*
 * ����ܽŷ���
 */
typedef enum GPIO_CFG
{
    //�����ֵ���ܸģ�����
    GPI         = 0,                          //����ܽ����뷽��      GPIOx_PDDRn�0��ʾ���룬1��ʾ���
    GPO         = 1,                          //����ܽ��������

    GPI_DOWN    = 0x02,                       //��������              PORTx_PCRn��ҪPE=1��PS=0
    GPI_UP      = 0x03,                       //��������              PORTx_PCRn��ҪPE=1��PS=1
    GPI_PF      = 0x10,                       //���룬����Դ�˲���,�˲���Χ��10 MHz ~ 30 MHz ����֧�ָ��ٽӿڣ�>=2MHz��  0b10000           Passive Filter Enable
    GPI_DOWN_PF = GPI_DOWN | GPI_PF ,         //��������������Դ�˲���
    GPI_UP_PF   = GPI_UP   | GPI_PF ,         //��������������Դ�˲���

    GPO_HDS     = 0x41,                        //�������������   0b100 0001    High drive strength
    GPO_SSR     = 0x05,                        //������仯��          0b101     Slow slew rate
    GPO_HDS_SSR = GPO_HDS | GPO_SSR,           //������������������仯��
} GPIO_CFG;  //���λΪ0���϶������룻GPI_UP �� GPI_UP_PF�����λΪ1������Ϊ���
typedef enum exti_cfg
{
    zero_down     = 0x08u,     //�͵�ƽ�������ڲ�����
    rising_down   = 0x09u,     //�����ش������ڲ�����
    falling_down  = 0x0Au,     //�½��ش������ڲ�����
    either_down   = 0x0Bu,     //�����ش������ڲ�����
    one_down      = 0x0Cu,     //�ߵ�ƽ�������ڲ�����

    //�����λ��־����������
    zero_up       = 0x88u,     //�͵�ƽ�������ڲ�����
    rising_up     = 0x89u,     //�����ش������ڲ�����
    falling_up    = 0x8Au,     //�½��ش������ڲ�����
    either_up     = 0x8Bu,     //�����ش������ڲ�����
    one_up        = 0x8Cu      //�ߵ�ƽ�������ڲ�����
} exti_cfg;


#define HIGH  1u
#define LOW   0u


extern  GPIO_MemMapPtr      GPIOX[PTX_MAX];
#define GPIOX_BASE(PTxn)    GPIOX[PTX(PTxn)]       //GPIOģ��ĵ�ַ


/****************************�ⲿʹ��****************************/

extern void    gpio_init  (PTXn_e, GPIO_CFG, uint8 data);    //��ʼ��gpio
extern void    gpio_ddr   (PTXn_e, GPIO_CFG);                //�����������ݷ���
extern void    gpio_set   (PTXn_e,           uint8 data);    //��������״̬
extern void    gpio_turn  (PTXn_e);                          //��ת����״̬
extern uint8   gpio_get   (PTXn_e);                          //��ȡ����״̬

//���� 4�� ���� �� PTxn ֻ���� �궨�壬������ ����
#define GPIO_SET(PTxn,data)       (PTXn_T(PTxn,OUT)= (data))    //���������ƽ
#define GPIO_TURN(PTxn)           (PTXn_T(PTxn,T)= 1)           //��ת�����ƽ
#define GPIO_GET(PTxn)            (PTXn_T(PTxn,IN))             //��ȡ��������״̬
#define GPIO_DDR(PTxn,ddr)        (PTXn_T(PTxn,DDR) = ddr)      //�������״̬


//����  ���� �� PTxn ������  �궨�壬Ҳ������ ����



//GPIO  1λ����
#define GPIO_SET_1bit(PORTx,n,data)   GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])& ~(0x1<<(n)) )|(((data)&0x01)<<(n)))   //д1λ���ݣ�nΪ���λ���źţ�
#define GPIO_DDR_1bit(PORTx,n,ddr)    GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0x1<<(n)) )|(((ddr)&0x01)<<(n)))    //����1λ�����������nΪ���λ���źţ�
#define GPIO_GET_1bit(PORTx,n)        (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0x1)                                                        //��1λ���ݣ�nΪ���λ���źţ�

//GPIO  2λ����
#define GPIO_SET_2bit(PORTx,n,data)   GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])& ~(0x3<<(n)) )|(((data)&0x03)<<(n)))   //д2λ���ݣ�nΪ���λ���źţ�
#define GPIO_DDR_2bit(PORTx,n,ddr)    GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0x3<<(n)) )|(((ddr)&0x03)<<(n)))    //����2λ�����������nΪ���λ���źţ�
#define GPIO_GET_2bit(PORTx,n)        (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0x3)                                                        //��2λ���ݣ�nΪ���λ���źţ�

//GPIO  4λ����
#define GPIO_SET_4bit(PORTx,n,data)   GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])& ~(0xf<<(n)) )|(((data)&0x0f)<<(n)))   //д4λ���ݣ�nΪ���λ���źţ�
#define GPIO_DDR_4bit(PORTx,n,ddr)    GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0xf<<(n)) )|(((ddr)&0x0f)<<(n)))    //����4λ�����������nΪ���λ���źţ�
#define GPIO_GET_4bit(PORTx,n)        (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0xf)                                                        //��4λ���ݣ�nΪ���λ���źţ�

//GPIO  8λ����
#define GPIO_SET_8bit(PORTx,n,data)   GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])& ~(0xff<<(n)) )|(((data)&0xff)<<(n)))  //д8λ���ݣ�nΪ���λ���źţ�  Ұ��ע������8λ�������ȥ
#define GPIO_DDR_8bit(PORTx,n,ddr)    GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0xff<<(n)) )|(((ddr)&0x0ff)<<(n)))  //����8λ�����������nΪ���λ���źţ�
#define GPIO_GET_8bit(PORTx,n)        (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0xff)                                                       //��8λ���ݣ�nΪ���λ���źţ�


//GPIO  16λ����
#define GPIO_SET_16bit(PORTx,n,data)  GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])&~(0xffff<<(n)) )|(((data)&0xffff)<<(n)))   //д16λ���ݣ�nΪ���λ���źţ�
#define GPIO_DDR_16bit(PORTx,n,ddr)   GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0xffff<<(n)) )|(((ddr)&0x0ffff)<<(n)))  //����16λ�����������nΪ���λ���źţ�
#define GPIO_GET_16bit(PORTx,n)       (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0xffff)                                                         //��16λ���ݣ�nΪ���λ���źţ�

//GPIO  32λ����
#define GPIO_SET_32bit(PORTx,data)  GPIO_PDOR_REG(GPIOx[(PORTx)])=(data)                                                                    //д32λ����
#define GPIO_DDR_32bit(PORTx,ddr)   GPIO_PDDR_REG(GPIOx[(PORTx)])=(ddr)                                                                     //����32λ�����������
#define GPIO_GET_32bit(PORTx)       GPIO_PDIR_REG(GPIOx[(PORTx)])                                                                           //��32λ����

/****************************�ڲ�ʹ�ã��û�����Ҫ����****************************/
#define GPIO_SET_1(PORTx,n)          GPIO_PDOR_REG(GPIOx[(PORTx)]) |=  (1<<(n))      //�������Ϊ�ߵ�ƽ        ���磺GPIO_SET_H(PORTA,1)   PA1����ߵ�ƽ
#define GPIO_SET_0(PORTx,n)          GPIO_PDOR_REG(GPIOx[(PORTx)]) &= ~(1<<(n))      //�������Ϊ�͵�ƽ        ���磺GPIO_SET_L(PORTA,1)   PA1����͵�ƽ

#endif      //__MK60_GPIO_H__
