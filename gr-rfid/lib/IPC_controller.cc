#include "IPC_controller.h"
#include <iostream>
#include <cstring>
#include <algorithm>

//#define __IPC_DEBUG__


IPC_controller::IPC_controller(){
  receiver_key = ftok(PATH, 'c');
  sender_key = ftok(PATH, 's');

  if((receiver_key < 0) || (sender_key < 0)){
    std::cerr << "IPC key initialize failed!";
    exit(1);
  }

  receiver_id = msgget(receiver_key, 0666 | IPC_CREAT);
  sender_id = msgget(sender_key, 0666 | IPC_CREAT);

#ifdef __IPC_DEBUG__
  std::cout << "r_id : "<<receiver_id<<std::endl;
  std::cout << "s_id : "<<sender_id<<std::endl;
#endif

  std::cout<<"IPC initialize success"<<std::endl;
}



IPC_controller::~IPC_controller(){

  //clear all buf
  while(1){
    int n = msgrcv(receiver_id, &recv_buf, sizeof(recv_buf) - sizeof(long), IPC_ALL__, IPC_NOWAIT);
    if(n == -1)
      break;
  }

  while(1){
    int n = msgrcv(sender_id, &recv_buf, sizeof(recv_buf) - sizeof(long), IPC_ALL__, IPC_NOWAIT);
    if(n == -1)
      break;
  }

  //remove ids
  msgctl(receiver_id, IPC_RMID, NULL);
  msgctl(sender_id, IPC_RMID, NULL);
}




int IPC_controller::data_send(void * buf, int buf_len, long type){
  send_buf.msg_type = type;
  memset(send_buf.msg_data, 0, IO_BUF_SIZE);
  memcpy(send_buf.msg_data, buf, std::min(IO_BUF_SIZE, buf_len));

  int rt = msgsnd(sender_id, &send_buf, sizeof(send_buf) - sizeof(long), 0);

#ifdef __IPC_DEBUG__
  std::cout<<"ipc data send result : "<<rt<<std::endl;
  std::cout<<"size of : "<<sizeof(send_buf)<<std::endl;
#endif

  return rt;
}




int IPC_controller::data_recv(void * buf, int buf_len, long type){
  int n = msgrcv(receiver_id, &recv_buf, sizeof(recv_buf) - sizeof(long), type, 0);

#ifdef __IPC_DEBUG__
  std::cout << "ipc data receive"<<std::endl;
  std::cout << "flag : "<<(int)(recv_buf.msg_flag)<< ", n : "<<n<<std::endl;
  std::cout << "return value would be " << (recv_buf.msg_flag & IPC_SYNC__) <<std::endl;
#endif

  memcpy(buf, recv_buf.msg_data, std::min(IO_BUF_SIZE, buf_len));

  return n;
}


int IPC_controller::send_sync(){
  int rt;

  if(data_send(NULL, 0, IPC_SYNC__)){
    std::cerr <<"SYNC send error";
    return -1;
  }

  std::cout <<"Waiting SYNC ACK"<<std::endl;
  if(wait_ack() == -1){
    std::cout<< "SYNC ACK Error"<<std::endl;
    return -1;
  }else{ //Receive proper ACK
    std::cout<< "SYNC complete"<<std::endl; 
    return 0;
  }
}



int IPC_controller::wait_sync(){
  int rt;

  rt = data_recv(NULL, 0, IPC_SYNC__);
#ifdef __IPC_DEBUG__
  std::cout<<"data_recv return : "<<rt<<std::endl;
  std::cout<<"It is supposed to be : "<<-IPC_SYNC__<<std::endl;
#endif

  if(rt == -1)
    return -1;
  else
    return 0;
}



int IPC_controller::send_ack(){
  char dummy_buf[IO_BUF_SIZE] = {};

  if(data_send(dummy_buf, IO_BUF_SIZE, IPC_ACK__) == -1){
    std::cerr <<"ACK send error";
    return -1;
  }

  return 0;
}



int IPC_controller::wait_ack(){

  if(data_recv(NULL, 0, IPC_ACK__) == -1){
    std::cerr <<"ACK wait error";
    return -1;
  }

  return 0;

}

