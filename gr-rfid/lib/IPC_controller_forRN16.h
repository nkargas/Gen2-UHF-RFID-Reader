#ifndef _IPC_CONTROLLER_forRN16_
#define _IPC_CONTROLLER_forRN16_

#include <iostream>
#include <vector>
#include <complex>
#include "IPC_controller.h"

class IPC_controller_forRN16 : public IPC_controller{

private:
  struct avg_corr_data{
    char successFlag;
    char RN16[16];
    float avg_corr;
    float avg_i;
    float avg_q;
  } data;

public:
  IPC_controller_forRN16();
  ~IPC_controller_forRN16();

  int send_avg_corr(std::vector<float>, double);
  int send_avg_corr(std::vector<float>, std::complex<float>);
  int send_avg_corr(std::vector<float>, double, std::complex<float>);

  int send_failed(void);

};



#endif
