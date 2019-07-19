#ifndef __IPC_CONTROLLER__
#define __IPC_CONTROLLER__

#include <cstdio> 
#include <sys/ipc.h> 
#include <sys/msg.h>

#define PATH "/tmp"

#define IO_BUF_SIZE 64

#define IPC_ALL__   0x00
#define IPC_DATA__  0x01
#define IPC_SYNC__  0x02
#define IPC_ACK__   0x04


struct msg_format{
  long msg_type;
  char msg_data[IO_BUF_SIZE];
};

class IPC_controller{
  private:
    key_t receiver_key, sender_key;
    int receiver_id, sender_id;
    struct msg_format recv_buf, send_buf;
  
  public:
    IPC_controller();
    ~IPC_controller();
    int data_send(void * buf, int buf_len = IO_BUF_SIZE, long type = IPC_DATA__);
    int data_recv(void * buf, int buf_len = IO_BUF_SIZE, long type = IPC_DATA__);
    int send_sync();
    int wait_sync();
    int send_ack();
    int wait_ack();

};


#endif
