#include "jbd_bms_status.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "jbd_bms_status_node");
  ros::NodeHandle private_nh("~");

  iqr::JbdBmsStatus jbd_bms_status(private_nh);  
  jbd_bms_status.initPort();

  uint16_t buffer_sum = 0x00;
  std::vector<uint8_t> buffer_v;
  ros::Rate loop_rate(jbd_bms_status.looprate_);
  while(ros::ok) {
    jbd_bms_status.buffer_all_ = jbd_bms_status.dataRead(
        jbd_bms_status.cmd_status_,
        jbd_bms_status.cmd_status_sum_,
        buffer_sum,
        buffer_sum,
        buffer_v);

    jbd_bms_status.buffer_vol_ = jbd_bms_status.dataRead(
        jbd_bms_status.cmd_voltage_,
        jbd_bms_status.cmd_voltage_sum_,
        buffer_sum,
        buffer_sum,
        buffer_v);

    jbd_bms_status.dataParsing(jbd_bms_status.buffer_all_, jbd_bms_status.buffer_vol_);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
