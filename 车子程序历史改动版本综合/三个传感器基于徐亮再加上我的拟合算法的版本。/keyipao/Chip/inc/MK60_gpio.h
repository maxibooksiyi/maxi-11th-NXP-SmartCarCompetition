/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外初学论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_gpio.h
 * @brief      gpio驱动头文件
 * @author     山外科技
 * @version    v5.1
 * @date       2014-04-25
 */
#ifndef __MK60_GPIO_H__
#define __MK60_GPIO_H__
#include "MK60_gpio_cfg.h"

//端口宏定义
typedef enum PORTx
{
    PORTA,
    PORTB,
    PORTC,
    PORTD,
    PORTE
} PORTx;


/*
 * 定义管脚方向
 */
typedef enum GPIO_CFG
{
    //这里的值不能改！！！
    GPI         = 0,                          //定义管脚输入方向      GPIOx_PDDRn里，0表示输入，1表示输出
    GPO         = 1,                          //定义管脚输出方向

    GPI_DOWN    = 0x02,                       //输入下拉              PORTx_PCRn需要PE=1，PS=0
    GPI_UP      = 0x03,                       //输入上拉              PORTx_PCRn需要PE=1，PS=1
    GPI_PF      = 0x10,                       //输入，带无源滤波器,滤波范围：10 MHz ~ 30 MHz 。不支持高速接口（>=2MHz）  0b10000           Passive Filter Enable
    GPI_DOWN_PF = GPI_DOWN | GPI_PF ,         //输入下拉，带无源滤波器
    GPI_UP_PF   = GPI_UP   | GPI_PF ,         //输入上拉，带无源滤波器

    GPO_HDS     = 0x41,                        //输出高驱动能力   0b100 0001    High drive strength
    GPO_SSR     = 0x05,                        //输出慢变化率          0b101     Slow slew rate
    GPO_HDS_SSR = GPO_HDS | GPO_SSR,           //输出高驱动能力、慢变化率
} GPIO_CFG;  //最低位为0，肯定是输入；GPI_UP 和 GPI_UP_PF的最低位为1，其他为输出
typedef enum exti_cfg
{
    zero_down     = 0x08u,     //低电平触发，内部下拉
    rising_down   = 0x09u,     //上升沿触发，内部下拉
    falling_down  = 0x0Au,     //下降沿触发，内部下拉
    either_down   = 0x0Bu,     //跳变沿触发，内部下拉
    one_down      = 0x0Cu,     //高电平触发，内部下拉

    //用最高位标志上拉和下拉
    zero_up       = 0x88u,     //低电平触发，内部上拉
    rising_up     = 0x89u,     //上升沿触发，内部上拉
    falling_up    = 0x8Au,     //下降沿触发，内部上拉
    either_up     = 0x8Bu,     //跳变沿触发，内部上拉
    one_up        = 0x8Cu      //高电平触发，内部上拉
} exti_cfg;


#define HIGH  1u
#define LOW   0u


extern  GPIO_MemMapPtr      GPIOX[PTX_MAX];
#define GPIOX_BASE(PTxn)    GPIOX[PTX(PTxn)]       //GPIO模块的地址


/****************************外部使用****************************/

extern void    gpio_init  (PTXn_e, GPIO_CFG, uint8 data);    //初始化gpio
extern void    gpio_ddr   (PTXn_e, GPIO_CFG);                //设置引脚数据方向
extern void    gpio_set   (PTXn_e,           uint8 data);    //设置引脚状态
extern void    gpio_turn  (PTXn_e);                          //反转引脚状态
extern uint8   gpio_get   (PTXn_e);                          //读取引脚状态

//如下 4个 函数 的 PTxn 只能是 宏定义，不能是 变量
#define GPIO_SET(PTxn,data)       (PTXn_T(PTxn,OUT)= (data))    //设置输出电平
#define GPIO_TURN(PTxn)           (PTXn_T(PTxn,T)= 1)           //翻转输出电平
#define GPIO_GET(PTxn)            (PTXn_T(PTxn,IN))             //读取引脚输入状态
#define GPIO_DDR(PTxn,ddr)        (PTXn_T(PTxn,DDR) = ddr)      //输入输出状态


//如下  函数 的 PTxn 可以是  宏定义，也可以是 变量



//GPIO  1位操作
#define GPIO_SET_1bit(PORTx,n,data)   GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])& ~(0x1<<(n)) )|(((data)&0x01)<<(n)))   //写1位数据（n为最低位引脚号）
#define GPIO_DDR_1bit(PORTx,n,ddr)    GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0x1<<(n)) )|(((ddr)&0x01)<<(n)))    //设置1位输入输出方向（n为最低位引脚号）
#define GPIO_GET_1bit(PORTx,n)        (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0x1)                                                        //读1位数据（n为最低位引脚号）

//GPIO  2位操作
#define GPIO_SET_2bit(PORTx,n,data)   GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])& ~(0x3<<(n)) )|(((data)&0x03)<<(n)))   //写2位数据（n为最低位引脚号）
#define GPIO_DDR_2bit(PORTx,n,ddr)    GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0x3<<(n)) )|(((ddr)&0x03)<<(n)))    //设置2位输入输出方向（n为最低位引脚号）
#define GPIO_GET_2bit(PORTx,n)        (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0x3)                                                        //读2位数据（n为最低位引脚号）

//GPIO  4位操作
#define GPIO_SET_4bit(PORTx,n,data)   GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])& ~(0xf<<(n)) )|(((data)&0x0f)<<(n)))   //写4位数据（n为最低位引脚号）
#define GPIO_DDR_4bit(PORTx,n,ddr)    GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0xf<<(n)) )|(((ddr)&0x0f)<<(n)))    //设置4位输入输出方向（n为最低位引脚号）
#define GPIO_GET_4bit(PORTx,n)        (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0xf)                                                        //读4位数据（n为最低位引脚号）

//GPIO  8位操作
#define GPIO_SET_8bit(PORTx,n,data)   GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])& ~(0xff<<(n)) )|(((data)&0xff)<<(n)))  //写8位数据（n为最低位引脚号）  野火注：先清8位，再填进去
#define GPIO_DDR_8bit(PORTx,n,ddr)    GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0xff<<(n)) )|(((ddr)&0x0ff)<<(n)))  //设置8位输入输出方向（n为最低位引脚号）
#define GPIO_GET_8bit(PORTx,n)        (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0xff)                                                       //读8位数据（n为最低位引脚号）


//GPIO  16位操作
#define GPIO_SET_16bit(PORTx,n,data)  GPIO_PDOR_REG(GPIOx[(PORTx)])=(( GPIO_PDOR_REG(GPIOx[(PORTx)])&~(0xffff<<(n)) )|(((data)&0xffff)<<(n)))   //写16位数据（n为最低位引脚号）
#define GPIO_DDR_16bit(PORTx,n,ddr)   GPIO_PDDR_REG(GPIOx[(PORTx)])=(( GPIO_PDDR_REG(GPIOx[(PORTx)])& ~(0xffff<<(n)) )|(((ddr)&0x0ffff)<<(n)))  //设置16位输入输出方向（n为最低位引脚号）
#define GPIO_GET_16bit(PORTx,n)       (( GPIO_PDIR_REG(GPIOx[(PORTx)])>>(n) ) & 0xffff)                                                         //读16位数据（n为最低位引脚号）

//GPIO  32位操作
#define GPIO_SET_32bit(PORTx,data)  GPIO_PDOR_REG(GPIOx[(PORTx)])=(data)                                                                    //写32位数据
#define GPIO_DDR_32bit(PORTx,ddr)   GPIO_PDDR_REG(GPIOx[(PORTx)])=(ddr)                                                                     //设置32位输入输出方向
#define GPIO_GET_32bit(PORTx)       GPIO_PDIR_REG(GPIOx[(PORTx)])                                                                           //读32位数据

/****************************内部使用，用户不需要关心****************************/
#define GPIO_SET_1(PORTx,n)          GPIO_PDOR_REG(GPIOx[(PORTx)]) |=  (1<<(n))      //设置输出为高电平        例如：GPIO_SET_H(PORTA,1)   PA1输出高电平
#define GPIO_SET_0(PORTx,n)          GPIO_PDOR_REG(GPIOx[(PORTx)]) &= ~(1<<(n))      //设置输出为低电平        例如：GPIO_SET_L(PORTA,1)   PA1输出低电平

#endif      //__MK60_GPIO_H__
