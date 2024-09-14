#include "Serial.hpp"
#include <iostream>
uint8_t buff[100];
int dis_T0_A0, dis_T0_A1, dis_T0_A2;
const char *dev  = "/dev/ttyACM1";

void renew_data(serialPort& myserial){
  while (true)
    {
      //nwrite = myserial.writeBuffer( buff, 8);
      myserial.readBuffer(buff, 1);
      if(buff[0] == '\n'){
        break;
      }
    }
}

int hex2int(char c){
  if((c >= '0') && (c <= '9')){
    return c - '0';
  }
  else if ((c >= 'a') && (c <= 'e')){
    return c - 'a' + 10;
  }
  else{
    return 0;
  }
  
}


int main()
{ serialPort myserial;
int nread,nwrite;
  cout<<"serialPort Test"<<endl;
  myserial.OpenPort(dev);
  myserial.setup(19200,0,8,1,'N'); 
  renew_data(myserial);
  
  while (true)
  {
    myserial.readBuffer(buff, 12);
    if(buff[0] != 'm'){
      renew_data(myserial);
      continue;
    }
    printf("%.*s",12 ,buff);
    if(buff[1] == 'a'){
      dis_T0_A0 = hex2int(buff[10]) + 16 * hex2int(buff[9]) + 16 * 16 * hex2int(buff[8]);
      cout << "T0_A0: "<< dis_T0_A0 << endl;
    }

    if(buff[1] == 'b'){
      dis_T0_A1 = hex2int(buff[10]) + 16 * hex2int(buff[9]) + 16 * 16 * hex2int(buff[8]);
      cout << "T0_A1: "<< dis_T0_A1 << endl;
    }

    if(buff[1] == 'c'){
      dis_T0_A2 = hex2int(buff[10]) + 16 * hex2int(buff[9]) + 16 * 16 * hex2int(buff[8]);
      cout << "T0_A2: "<< dis_T0_A2 << endl;
    }
  
	  usleep(400);
  }
  
}
