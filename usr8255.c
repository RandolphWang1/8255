#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#define C8255 "/dev/c8255_driver"
unsigned char escposstr[] = {
            0x1b, 0x64, 0x03,
            0x1c,0x26,
            0xd3,0xae,0xc8,0xf3,0xbd,0xdd,0xcd,0xa7,0x0d,0x0a,
            0xd3,0xae,0xc8,0xf3,0xbd,0xdd,0xcd,0xa7,0x0d,0x0a,
            0xd3,0xae,0xc8,0xf3,0xbd,0xdd,0xcd,0xa7,0x0d,0x0a,
            0x1b, 0x64, 0x03,
};
char* escposstr1 = "ABCDEFG";
char* escposstr2 = "打印下行OK";
void sPAbusy(int fd)
{
    int ret;
    ret = ioctl(fd, 12, 0x00); //set PC5 busy
}

void sPAACK(int fd)
{
    int ret;
    ret = ioctl(fd, 13, 0x00); //set PC5 busy
}

void m_delay(int fd, int ms)
{
    int ret;
    ret = ioctl(fd, 15, ms); //set PC5 busy
}

int main(int argc, char** argv)
{
    int fd1,i;
    unsigned char databuf, dbuf1;
    int cmd;
    int arg, ret;
    fd1 = open(C8255, O_RDWR);
    if(fd1 ==-1) {
        printf("c8255 open failed %d\n", fd1);
        return -1;
    }
    if(argc < 3){
        cmd = 8;
        arg = 10000;
    } else {
        printf("argc %d, argv[1] %s, argv[2] %s\n", argc, argv[1], argv[2]);
        cmd = atoi(argv[1]);
        arg = atoi(argv[2]);
    }
    if(cmd == 8) {
        for(i = 0; i < arg; i++)
        {
            read(fd1, &databuf, 1);//10);
            printf("8255 port A data = %x\n", databuf);//[i]);
            write(fd1,&databuf,1);
        }
    } else if( cmd == 256) {
        ret = ioctl(fd1, 3, 0x90);
        for(i = 0; i < 10000; i++)
        {
#if 0           
            ret = ioctl(fd1, 10, 1);
     //       sleep(1);
            read(fd1, &databuf, 1);//10);
            ret = ioctl(fd1, 6, 0);//just read PC
            ret = ioctl(fd1, 2, 0x20); //set PC5 busy
            sleep(1);
            printf("8255 port A data = %x\n", databuf);//[i]);
            ret = ioctl(fd1, 2, 0x00); //set PC5 busy
            ret = ioctl(fd1, 10, 0); //set PC5 busy
            sleep(1);
            write(fd1,&databuf,1);
#else   
            sPAbusy(fd1);
            read(fd1, &databuf, 1);
            printf("8255 port A data = %x\n", databuf);//[i]);
            sPAACK(fd1);
            write(fd1,&databuf, 1);
#endif
          
        }
    } else if( cmd == 257) {
        
        for(i = 0; i < 10000; i++)
        {
          //  sleep(1);
        ret = ioctl(fd1, 8, 0x0);
        }
    } else if( cmd == 259) {
        
            printf("\n\n");//[i]);
        for(i = 0; i < 10000; i++)
        {
            read(fd1, &databuf, 1);
            printf(" %x", databuf);//[i]);
            write(fd1,&databuf, 1);
        }
            printf("\n\n");//[i]);
    } else if( cmd == 258) {
        
            write(fd1,escposstr1,strlen(escposstr1));
            write(fd1,escposstr,sizeof(escposstr));
            write(fd1,escposstr2,strlen(escposstr2));
    } else {
        ret = ioctl(fd1, cmd, arg);
    }
    close(fd1);
    return 0;
}
