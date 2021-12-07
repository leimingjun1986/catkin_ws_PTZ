#include  "serial_ptz/serial_ptz.hpp"
#include  "serial_ptz/yaml.hpp"

using namespace  ptz_control_ns;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "PTZ_Control_Node");
    ros::NodeHandle nh;
    SerialParam ser_param;
    PTZParam ptz_param;
    //ser_param.serial_dev ="/dev/ttyUSB0";
    //ser_param.baudrate =9600;
   // ser_param.databit = 8;
    //ser_param.parity = 0;
    string  config_file_path ="/home/flfjepl/catkin_ws_PTZ/src/serial_ptz/config/ptz_param.yaml";
    yaml_import_serial_param(config_file_path, ser_param );
    yaml_import_ptz_param(config_file_path,  ptz_param);

    Serial_PTZ  ser_ptz(nh,"action_ptz");
    int re=ser_ptz.serial_ptz_open(ser_param);   //打开串口
    if(re<0)  return 0;
    
    ser_ptz.set_ptz_param(ptz_param);
    ser_ptz.start_actionsever();  //打开action server
    ros::spin();

    return 0;
}

