#ifndef SERIAL_PTZ_ACTION_SERVER_H
#define SERIAL_PTZ_ACTION_SERVER_H

#include   <ros/ros.h>
#include   <serial/serial.h>
#include   <actionlib/server/simple_action_server.h>
#include   <serial_ptz/SerialPTZAction.h>
#include   <serial_ptz/SerialPTZResult.h>
#include   <mutex>
#include   <vector>

namespace ptz_control_ns{
typedef actionlib::SimpleActionServer<serial_ptz::SerialPTZAction>  PTZActionServer;

struct  SerialParam{
   std::string  serial_dev ; //串口设备名称
   int  baudrate ;  //波特率
   int  databit;   //数据位
   int  parity;  //奇偶校验
   int  timeout;
};

struct PTZParam{

   int zoom_limit ;
   int angle_unit ;
   int tilt_down_limit ;
   int tilt_up_limit ;
   int pan_min_limit ;
   int pan_max_limit ;
   int ch0;
   int ch1;
   int ch2;
};

class Serial_PTZ{
    
    public:
    enum iState_enum_type{
        STANDBY_STATE,
        EMERGENCY_STATE,
        FINISH_STATE,
        FAIL_STATE
    };
    
    iState_enum_type iState_;
    void SwitchToState(iState_enum_type new_state);
    char *GetStateString(iState_enum_type state);
    iState_enum_type GetState();
    Serial_PTZ(ros::NodeHandle nh_,std::string action_name);
    ~Serial_PTZ();
    serial::Serial  ser;
    int  serial_ptz_open(SerialParam);  //打开串口
    void   set_ptz_param(PTZParam param);
    pthread_mutex_t mutex;
    void  writeSerialWithMutex(uint8_t *buffer, uint16_t length, pthread_mutex_t mutex);
    // 一直向左旋转
    void  rotation_horiz_left();
    //一直向右旋转
    void  rotation_horiz_right();

    void  set_speed(uint8_t  speed);

    void  pan_move(float angle);  //水平旋转到坐标系某个角度(0-360)
    void  pan_move(uint32_t   angle);  //水平旋转到坐标系某个角度(0-36000)
    void  tilt_move(float angle);  //倾斜旋转到某个角度  0-180
    void  tilt_move(uint32_t   angle);

    void  get_pan_position(float& angle);
    void  get_tilt_position(float& angle);
    void  get_zoom_position(float& angle);
    void  get_pan_position(uint32_t& angle);
    void  get_tilt_position(uint32_t& angle);

    void  get_zoom_position(int32_t& zoom);
    void  set_zoom_position(int32_t& zoom);
    void  set_zoom_position(float zoom);
    void  set_zoom_increase();
    void  set_zoom_decrease();
    void  set_focus_near();
    void  set_focus_far();
    void  Key_pop_up();//按键弹起

    void  Go_To_Zero();

    //垂直方向向上旋转
    void  rotation_verti_up();
    //垂直方向向下旋转
    void rotation_verti_down();
    //停止旋转
    void  rotate_stop();

    void  set_preset_position(uint8_t num);

    void  call_preset_position(uint8_t num);
    // 回到原点
   //相对当前位置 水平向左旋转指定角度
    void  rotation_horiz_left(float  angle);
    void  rotation_horiz_right(float  angle);
    void  rotation_verti_up(float  angle);
    void  rotation_verti_down(float  angle);
    void  serial_send(uint8_t *buffer, uint16_t length);
    void  serial_receive(uint8_t *buffer, uint16_t length);
    bool  check_angle(float cur_ang,float tar_ang,float error);  //检查当前角度与目标角度是否在误差范围内
    bool  wait_rote_finish();  //等待旋转运动完成
    bool  wait_rote_finish(float tar_ang,float error);

    //镜头的控制
    bool  device_status;  // 串口设备状态
    ros::Timer  timer_listener;   //串口号设备 监听
    boost::thread  Receive_thread;  //串口数据接收线程 
    void  goal_callback(const serial_ptz::SerialPTZGoalConstPtr& goal);
    void  preempt_callback();
    void  start_actionsever();
    serial_ptz::SerialPTZFeedback  feedback_;
    serial_ptz::SerialPTZResult  result;
    PTZActionServer  ptz_control_as;   //ptz控制服务器   

private:
    ros::NodeHandle  nh;
    ros::Subscriber  sub_;
    float  speed;  //旋转速度设置
    uint8_t  pelcod[7];  //PELCO-D协议的7字节
    std::vector<uint8_t>  pelcod_rec;
    PTZParam ptz_param;
};


}


#endif
