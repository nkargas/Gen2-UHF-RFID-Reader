#include "IPC_controller_forRN16.h"

IPC_controller_forRN16::IPC_controller_forRN16(){
}

IPC_controller_forRN16::~IPC_controller_forRN16(){
}

int IPC_controller_forRN16::send_avg_corr(std::vector<float> RN16, double avg_corr){
  for(int i = 0; i<16; i++){
    data.RN16[i] = RN16[i];
  }

  data.avg_corr = avg_corr;

  data_send(&data, sizeof(struct avg_corr_data));

  wait_ack();
}
