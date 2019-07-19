#ifndef _IPC_CONTROLLER_forRN16_
#define _IPC_CONTROLLER_forRN16_

#include <iostream>
#include <vector>
#include "IPC_controller.h"

class IPC_controller_forRN16 : public IPC_controller{

private:
  struct avg_corr_data{
    char RN16[16];
    float avg_corr;
  } data;

public:
  IPC_controller_forRN16();
  ~IPC_controller_forRN16();

  int send_avg_corr(std::vector<float>, double);

};



#endif
