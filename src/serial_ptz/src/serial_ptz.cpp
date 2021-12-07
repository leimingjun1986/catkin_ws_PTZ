#include <cstdint>
#include "ros/init.h"
#include "ros/rate.h"
#include  "serial_ptz/serial_ptz.hpp"
#include <math.h>
#include <iostream> 
#include <iomanip>

namespace ptz_control_ns{

Serial_PTZ::Serial_PTZ(ros::NodeHandle nh_,std::string action_name):
  nh(nh_),
  ptz_control_as(nh_,action_name,boost::bind(&Serial_PTZ::goal_callback,this,_1),false)
{
  pelcod[0]=0xFF;
  pelcod[1]=0x01;
  pelcod[2]=0x00;
}

void Serial_PTZ::start_actionsever(){
  //ptz_control_as.registerGoalCallback(boost::bind(&Serial_PTZ::goal_callback,this));
  ptz_control_as.registerPreemptCallback(boost::bind(&Serial_PTZ::preempt_callback,this));  //取消任务
  ptz_control_as.start();
}

Serial_PTZ::~Serial_PTZ(){
   ser.close();
}

void  Serial_PTZ::rotation_horiz_left(){
   pelcod[0]=0xFF;
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x04;
   pelcod[4]=0x2F;   //水平速度
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);
}

void  Serial_PTZ::Go_To_Zero(){
   pelcod[0]=0xFF;
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x49;
   pelcod[4]=0x00;   //水平速度
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);
}

void  Serial_PTZ::rotation_horiz_right(){
   pelcod[0]=0xFF;
   pelcod[1]=0x01;    //云台地址
   pelcod[2]=0x00;
   pelcod[3]=0x02;
   pelcod[4]=0x2F;  //水平速度
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);  //校验位
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位
}

void Serial_PTZ::rotate_stop(){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x00;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5])%(0x100);
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位
}

void Serial_PTZ::pan_move(float angle){
   uint32_t ang = angle*ptz_param.angle_unit;   //角度值与命令中的角度值的变换关系  100
   pan_move(ang);
}

void Serial_PTZ::pan_move(uint32_t angle){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x4b;
   pelcod[4]=(angle>>8);
   pelcod[5]=angle;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位
}

void Serial_PTZ::tilt_move(float angle){
   uint32_t ang = angle*ptz_param.angle_unit;
   tilt_move(ang);
}

void Serial_PTZ::tilt_move(uint32_t angle){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x4d;
   pelcod[4]=(angle>>8);
   pelcod[5]=angle;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位
}
void  Serial_PTZ::get_pan_position(float& angle){
   //发送获取角度的命令
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x51;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);  // 直接取8位
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位

   //串口接收返回的角度
   int num=7;
   uint8_t data[num];
   while(1)
   {
    size_t n = ser.available();
    if (n==num)
    {
    ser.read(data,num);
    int ang=(data[4]<<8)+data[5];
    angle=ang*1.0/100.0;
    ROS_INFO("get_pan_position:%02x %d %f",data[3],ang,angle);
    break;
    }
    usleep(1000);  //
    //ROS_INFO("usleep(1000)");
   }

}

void  Serial_PTZ::get_pan_position(uint32_t& angle){
   //发送获取角度的命令
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x51;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);  // 直接取8位
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位

   //串口接收返回的角度
   int num=7;
   uint8_t data[num];
   while(1)
   {
    size_t n = ser.available();
    if (n==num)
    {
     ser.read(data,num);
     angle=(data[4]<<8)+data[5];
     break;
    }
    usleep(1000);  //
    //ROS_INFO("usleep(1000)");
   }
}

void  Serial_PTZ::get_tilt_position(float& angle){

 //发送获取角度的命令
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x53;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);  // 直接取8位
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位

   //串口接收返回的角度
   int num=7;
   uint8_t data[num];
   while(1)
   {
    size_t n = ser.available();
    if (n==num)
    {
    ser.read(data,num);
    int ang=(data[4]<<8)+data[5];
    angle=ang*1.0/100.0;
    ROS_INFO("get_tilt_position:%02x %d %f",data[3],ang,angle);
    break;
    }
    usleep(1000);  //
    //ROS_INFO("usleep(1000)");
   }
}

void  Serial_PTZ::get_tilt_position(uint32_t& angle){

 //发送获取角度的命令
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x53;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);  // 直接取8位
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位

   //串口接收返回的角度
   int num=7;
   uint8_t data[num];
   while(1)
   {
    size_t n = ser.available();
    if (n==num)
    {
    ser.read(data,num);
    angle=(data[4]<<8)+data[5];
    break;
    }
    usleep(1000);  //
    //ROS_INFO("usleep(1000)");
   }
}

//得到焦距变倍值  (00-ffff  0-65535) 0时焦距最小 视野最大   655355时焦距达变倍到zoom_limit上限 同时视野最小
void  Serial_PTZ::get_zoom_position(int32_t& zoom){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x55;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //发送变倍查询命令

   //串口接收返回的变倍值
   int num=7;
   uint8_t data[num];
   while(1)
   {
    size_t n = ser.available();
    if (n==num)
    {
    ser.read(data,num);
    zoom=(data[4]<<8)+data[5];
    break;
    }
    usleep(1000);  //
   }
}

//得到焦距变倍值  (00-ffff  0-65535) 0时焦距最小 视野最大   655355时焦距达变倍到zoom_limit上限 同时视野最小
void  Serial_PTZ::set_zoom_position(int32_t& zoom){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x4f;
   pelcod[4]=(zoom>>8);
   pelcod[5]=zoom;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //发送变倍查询命令
}

void  Serial_PTZ::set_zoom_position(float zoom_){  
   float  zoom_limit= ptz_param.zoom_limit;  //相机固定参数
   int32_t zoom=(zoom_/zoom_limit)*65535;
   set_zoom_position(zoom);
}

//设置变倍增加   相机视野变小
void  Serial_PTZ::set_zoom_increase(){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x40;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //发送变倍查询命令
}

//设置变倍减小   相机视野变大
void  Serial_PTZ::set_zoom_decrease(){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x20;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //发送变倍查询命令
}

//设置焦距近  只是调焦使图片清晰 不影响视野大小  
void  Serial_PTZ::set_focus_near(){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x01;
   pelcod[3]=0x00;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //发送变倍查询命令
}

//设置焦距远  只是调焦使图片清晰 不影响视野大小 
void  Serial_PTZ::set_focus_far(){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x80;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //发送变倍查询命令
}

//按键弹起  停止自动运动
void  Serial_PTZ::Key_pop_up(){
   pelcod[0]=0xFF; 
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x00;
   pelcod[4]=0x00;
   pelcod[5]=0x00;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   writeSerialWithMutex(pelcod,7,mutex);  //发送变倍查询命令
}

void  Serial_PTZ::rotation_horiz_right(float angle){
    ROS_INFO("rotation_horiz_right(flat angle)");
     uint32_t cur_ang;
     uint32_t move_ang =angle*100;
     get_pan_position(cur_ang);
     //根据当前角度计算旋转的角度   角度坐标系逆时针
     int32_t R_ang=36000;
     int32_t tar_angle = cur_ang - move_ang;
     //归一化至(0-36000)
     tar_angle=(tar_angle%R_ang+R_ang)%R_ang;
     ROS_INFO("start move cur_ang:%d  move_ang:%d  tar_angle:%d",cur_ang,move_ang,tar_angle);
     pan_move((uint32_t)tar_angle);
}


bool  Serial_PTZ::check_angle(float cur_ang,float tar_ang,float error){
  //计算角度偏差是否在误差范围内
  bool re=false;
  int  temp=(tar_ang-cur_ang)*100;
  int  de=36000;
  int  er=error*100;
  if(abs((temp%de+de)%de)<error)
    {re=true;}
  return re;
}

bool  Serial_PTZ::wait_rote_finish(float tar_ang,float error){

   ros::Rate r(30);
   float cur_ang;
   while(ros::ok()){
      get_pan_position(cur_ang);
      bool re=check_angle(cur_ang,tar_ang, error);
      if(re) break;
      r.sleep();
      ros::spinOnce();
   }

}



void  Serial_PTZ::rotation_horiz_left(float angle){
     ROS_INFO("rotation_horiz_left(flat angle)");
     uint32_t cur_ang;
     uint32_t move_ang =angle*100;
     get_pan_position(cur_ang);

     //根据当前角度计算旋转的角度   角度坐标系逆时针
     int32_t R_ang=36000;
     int32_t tar_angle = cur_ang + move_ang;
     //归一化至(0-36000)
     tar_angle=(tar_angle%R_ang+R_ang)%R_ang;

     ROS_INFO("start move cur_ang:%d  move_ang:%d  tar_angle:%d",cur_ang,move_ang,tar_angle);
     pan_move((uint32_t)tar_angle);   
     uint32_t check_ang;
     //get_pan_position(check_ang);    //发送查询命令  旋转命令不会停止 旋转完成后返回  查询值?
        
     //ROS_INFO("finish move check_ang:%d  move_ang:%d  tar_angle:%d",check_ang,move_ang,tar_angle);
 }

 bool  isRang(int32_t data,int32_t min,int32_t max){
   if(data>=min&&data<=max)
   return true;
   else
    return false;
 }

 void  Serial_PTZ::rotation_verti_down(float angle){

     uint32_t down_band =  ptz_param.tilt_down_limit;   //上下限位角度值   [0-89.99]
     uint32_t up_band   =  ptz_param.tilt_up_limit;  //[269.99-359.99]
     ROS_INFO("rotation_verti_down(flat angle)");
     uint32_t cur_ang;
     int32_t move_ang =angle*100;
     get_tilt_position(cur_ang);
     //根据当前角度计算旋转的角度   角度坐标系逆时针
     int32_t R_ang=36000;
     int32_t tar_angle = cur_ang + move_ang;
     ROS_INFO("  tar_angle:%d", tar_angle);
     //归一化至(0-36000)
     tar_angle=(tar_angle%R_ang+R_ang)%R_ang;
     ROS_INFO("norm  tar_angle:%d", tar_angle);

     if((!isRang(tar_angle,0,down_band))&&(!isRang(tar_angle,up_band,35999))||(!isRang(move_ang,0,18000)))
     tar_angle=down_band;

     ROS_INFO("start move cur_ang:%d  move_ang:%d  tar_angle:%d",cur_ang,move_ang,tar_angle);
     tilt_move((uint32_t)tar_angle);
 }

 void  Serial_PTZ::rotation_verti_up(float angle){
     int32_t down_band =   ptz_param.tilt_down_limit;   //上下限位角度值   [0-89.99]
     int32_t up_band   =   ptz_param.tilt_up_limit;  //[269.99-359.99]
     ROS_INFO("rotation_verti_down(flat angle)");
     uint32_t cur_ang;
     uint32_t move_ang =angle*100;
     get_tilt_position(cur_ang);
     //根据当前角度计算旋转的角度   角度坐标系逆时针
     int32_t R_ang=36000;   //有符号
     int32_t tar_angle = cur_ang - move_ang;   //一定要是有符号型
     ROS_INFO("  tar_angle:%d", tar_angle);
     //归一化至(0-36000)
     tar_angle=(tar_angle%R_ang+R_ang)%R_ang;   //有符号
     ROS_INFO("norm  tar_angle:%d", tar_angle);

     if((!isRang(tar_angle,0,down_band))&&(!isRang(tar_angle,up_band,35999))||(!isRang(move_ang,0,18000)))
     tar_angle=up_band;

     ROS_INFO("start move cur_ang:%d  move_ang:%d  tar_angle:%d",cur_ang,move_ang,tar_angle);
     tilt_move((uint32_t)tar_angle);
 }

 //垂直方向向上旋转
  void  Serial_PTZ::rotation_verti_up(){
   pelcod[0]=0xFF;
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x08;
   pelcod[4]=0x00;
   pelcod[5]=0x2F;
   //pelcod[6]=0x38;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5])%(0x100);
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位
}

 //垂直方向向下旋转
void Serial_PTZ::rotation_verti_down(){
   pelcod[0]=0xFF;
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x10;
   pelcod[4]=0x00;
   pelcod[5]=0x2F;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5])%(0x100);
   writeSerialWithMutex(pelcod,7,mutex);  //写pelcod协议数据是固定的7位
}


void  Serial_PTZ::set_preset_position(uint8_t num){

   pelcod[0]=0xFF;
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x03;
   pelcod[4]=0x00;
   pelcod[5]=num;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5])%(0x100);
   printf("%02x %02x %02x %02x %02x %02x %02x\n",pelcod[0],pelcod[1],pelcod[2],pelcod[3],pelcod[4],pelcod[5],pelcod[6]);
   writeSerialWithMutex(pelcod,7,mutex);

}


void  Serial_PTZ::call_preset_position(uint8_t num){

   pelcod[0]=0xFF;
   pelcod[1]=0x01;
   pelcod[2]=0x00;
   pelcod[3]=0x07;
   pelcod[4]=0x00;
   pelcod[5]=num;
   pelcod[6]=(pelcod[1]+pelcod[2]+pelcod[3]+pelcod[4]+pelcod[5]);
   //printf("%02x %02x %02x %02x %02x %02x %02x\n",pelcod[0],pelcod[1],pelcod[2],pelcod[3],pelcod[4],pelcod[5],pelcod[6]);
   writeSerialWithMutex(pelcod,7,mutex);

}
void  Serial_PTZ::set_ptz_param(PTZParam param){
   ptz_param =param;
}


  //打开串口s  serial_ptz_open
int Serial_PTZ::serial_ptz_open(SerialParam  ser_param){ 
    try
    {
      ser.setPort(ser_param.serial_dev);
      ser.setBaudrate(ser_param.baudrate);
      serial::Timeout to = serial::Timeout::simpleTimeout(ser_param.timeout);
      ser.setTimeout(to);
      ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("PTZ Serial Port open failure");
        return -1;
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("PTZ Serial Port open successful");
        return 0;
    }
    else
    {
        ROS_INFO_STREAM("PTZ Serial Port open failure");
        return -2;
    }
}  
// 打开串口e  serial_ptz_open

/*
* biref: 带互斥量写串口数据 
*/
void Serial_PTZ::writeSerialWithMutex(uint8_t *buffer, uint16_t length, pthread_mutex_t mutex)
{ 
    //pthread_mutex_lock(&mutex);
    
     ser.write(buffer,length);
     std::cout<<"command:";
     for(int n=0;n<length;n++){
      std::cout<<std::hex<< (buffer[n]&0xff)<<" ";
     }
     std::cout<<std::endl;
    //pthread_mutex_unlock(&mutex);      
}
void Serial_PTZ::serial_send(uint8_t *buffer, uint16_t length)
{ 
     ser.write(buffer,length);
     std::cout<<"command:";
     for(int n=0;n<length;n++){
      std::cout<<std::hex<< (buffer[n]&0xff)<<" ";
     }
     std::cout<<std::endl;     
}

void Serial_PTZ::serial_receive(uint8_t *buffer, uint16_t length)
{ 
   //串口接收返回的角度
   while(ros::ok())
   {
    size_t n = ser.available();
    if (n==length)
    {
    ser.read(buffer,n);
    break;
    }
    usleep(1000);  //
   }       
}

Serial_PTZ::iState_enum_type Serial_PTZ::GetState(void)
{
    return iState_;
}

char *Serial_PTZ::GetStateString(iState_enum_type state)
{
    switch(state){
		case STANDBY_STATE:
			return (char *)"STANDBY_STATE";
		break;
        case EMERGENCY_STATE:
            return (char *)"EMERGENCY_STATE";
        break;
        case FINISH_STATE:
            return (char *)"FINISH_STATE";
        break;
        case FAIL_STATE:
            return (char *)"FAIL_STATE";
        break;
       
		default:
			return (char *)"UNKNOWN";
		break;
	}
}

void Serial_PTZ::SwitchToState(iState_enum_type new_state)
{
    if(new_state == iState_)
        return;
    ROS_INFO("SwitchToState: %s -> %s", GetStateString(iState_), GetStateString(new_state));
    iState_ = new_state;
}

void Serial_PTZ::goal_callback(const serial_ptz::SerialPTZGoalConstPtr& goal)
{
   if(ptz_control_as.isNewGoalAvailable())
    {ptz_control_as.acceptNewGoal();}
   int aid= goal->ActionID;
   int md =goal->mode;
   int re=aid*30+md;  //对任务编码
   ROS_INFO(" goal_callback re:%d  aid:%d  md:%d",re,aid,md);
   
   switch(re){
      case 31:
         //向左旋转某角度 
         ROS_INFO(" rotation_horiz_left %f)",goal->pitch);
         rotation_horiz_left(goal->pitch);      
         break;
      case 32:
        //向右旋转某角度
         ROS_INFO(" rotation_horiz_right %f",goal->pitch);
         rotation_horiz_right(goal->pitch);
         break;
      case 33:
        //向上旋转某角度
         ROS_INFO("rotation_verti_up %f",goal->yaw);
         rotation_verti_up(goal->yaw);
         break;
      case 34:
        //向下旋转某角度
         ROS_INFO("rotation_verti_down %f",goal->yaw);
         rotation_verti_down(goal->yaw);
         break;
      case 35:
        //向下旋转某角度
         ROS_INFO("go pre_position %d",goal->pre_id);
         call_preset_position(goal->pre_id);
         break;
      case 60:
        //回到坐标原点位置
         ROS_INFO("Go_To_Zero");
         Go_To_Zero();
         break; 
      case 61:
        //水平方向旋转到确定坐标位置
         ROS_INFO("pan_move %f",goal->pitch);
         pan_move(goal->pitch);
         break;
      case 62:
        //垂直方向旋转到确定坐标位置
         ROS_INFO("tilt_move %f",goal->yaw);
         tilt_move(goal->yaw);
         break; 
      case 91:
         //一直向左旋转  
         ROS_INFO("rotation_horiz_left");
         rotation_horiz_left();      
         break;
      case 92:
        //一直向右旋转 
         ROS_INFO("rotation_horiz_right");
         rotation_horiz_right();
         break;
      case 93:
        //一直向上旋转
         ROS_INFO("rotation_verti_up");
         rotation_verti_up();
         break;
      case 94:
        //一直向下旋转
         ROS_INFO("rotation_verti_down");
         rotation_verti_down();
         break; 
      case 95:
        //停止旋转
         ROS_INFO("rotation_stop");
         rotate_stop();
         break;
      case 121:
         //查询水平方向 角度值
         ROS_INFO(" get_pan_position(angle)");
         float  angle;
         get_pan_position(angle);
         break;
      case 122:
        //查询垂直方向 角度值
         ROS_INFO(" get_tilt_position(angle)");
         get_tilt_position(angle);
         break;
      case 151:
        //zoom +++
         ROS_INFO("zoom ++++++++++++++");
         set_zoom_increase();
         break;
      case 152:
        //zoom ---
         ROS_INFO("zoom --------------");
         set_zoom_decrease();
         break;
      case 153:
        //stop
         ROS_INFO("camera control stop");
         Key_pop_up();
         break;
      case 154:
         //focus near
         ROS_INFO("focus near");
         set_focus_near();
         break;
      case 155:
         //focus near
         ROS_INFO("focus near");
         set_focus_far();
         break;
      case 156:
         //设置焦距变倍 数值位置 (0.0-20.0(zoom_limit))
         ROS_INFO("set zoom positon");
         set_zoom_position(goal->zoom);
         break;
      case 157:
         //查询变倍 数值位置 (0.0-20.0(zoom_limit))
         ROS_INFO("get zoom positon");
         int32_t zoom_position;
         get_zoom_position(zoom_position);
         break;
      default:
         ROS_INFO("PTZ undefined command");  //
         if(ptz_control_as.isActive()) ptz_control_as.setAborted();
         break;
   }
   if(ptz_control_as.isActive()&&ros::ok())   
      {
         //发送一个结果
         result.result=  1;
         result.ActionID =aid;
         result.mode=md;
         ptz_control_as.setSucceeded(result);     
      }

   // check that preempt has not been requested by the client
   if (ptz_control_as.isPreemptRequested() || !ros::ok())
    {
       // set the action state to preempted
      result.result=  -1;
      result.ActionID =aid;
      result.mode=md;
      ptz_control_as.setPreempted(result);
    }
   
}


void Serial_PTZ::preempt_callback()
{
    ROS_WARN("ptz_control_as preempt_callback()");
    if(ptz_control_as.isActive()){
        ptz_control_as.setAborted();
    }
}





}


