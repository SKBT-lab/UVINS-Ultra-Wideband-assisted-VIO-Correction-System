#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h> 
#include     <sys/stat.h>
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include <stdint.h>
using namespace std;
class serialPort
{   
    private:
	   int fd;
	   struct  termios Opt;
       int speed_arr[14] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
                         B38400, B19200, B9600, B4800, B2400, B1200, B300, };
       int name_arr[14] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,  
                         19200,  9600, 4800, 2400, 1200,  300, };
       
	public:
	 serialPort();
	 bool OpenPort(const char * dev);
	 int setup(int speed,int flow_ctrl,int databits,int stopbits,int parity)  ;
	 void set_speed(int speed);
	 int set_Parity(int databits,int stopbits,int parity);
	 int readBuffer(uint8_t * buffer,int size);
	 int writeBuffer(uint8_t * buffer,int size);
	 uint8_t getchar();
	 void ClosePort(); 
};
