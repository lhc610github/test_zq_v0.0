#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include <string.h>
#include <boost/thread.hpp>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <stdlib.h>
#include <time.h>
#include"checksum.h"


#define BAUDRATE B57600
#define MODEMDEVICE "/dev/ttyUSB0"

typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

demo_test::pos_data xyz;
demo_test::pos_data xyz_old;
float vx = 0;
float vy = 0;
float vz = 0;
boost::mutex pos_mutex;
struct timespec time1 = {0, 0};
struct timespec time2 = {0, 0};

int open_port(const char * tty_port)
{
    int fd;
    fd = open(tty_port, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        printf("open tty error\n");
        return (-1);
    }
    else
    {
        printf("open %s successfully\n", tty_port);
    }
    if (!isatty(fd))
    {
        printf("%s is not a serial port\n", tty_port);
        return (-1);
    }
    return (fd);
}

int setup_port(int fd)
{
    struct termios config;
    if (tcgetattr(fd, &config) < 0)
    {
        printf("get serial pot attribute error\n");
        return (-1);
    }
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
#ifdef OLCUC
  config.c_oflag &= ~OLCUC;
#endif
#ifdef ONOEOT
  config.c_oflag &= ~ONOEOT;
#endif
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  config.c_cc[VMIN] = 0;
  config.c_cc[VTIME] = 0; 
   if (cfsetispeed(&config, BAUDRATE) < 0 || cfsetospeed(&config, BAUDRATE) < 0)
    {
		printf("\nERROR: Could not set desired baud rate of %d Baud\n", BAUDRATE);
		return (-1);
    }
   if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return (-1);
    }
    return 1;
}


void udpcallback(const demo_test::pos_data& pos)
{
    /*
    unsigned int num = pos.num;
    ROS_INFO("num is: %d", num);
    for(int i=0; i<num; i++)
    {
        ROS_INFO("rigidbody_%d: x=%f, y=%f, z=%f", i+1, pos.pos[i*3], pos.pos[i*3+1], pos.pos[i*3+2]);
    }
    */
    //lock mutex
    clock_gettime(CLOCK_MONOTONIC, &time1);
    long int dt = (time1.tv_sec-time2.tv_sec)*1000000000+time1.tv_nsec-time2.tv_nsec;  
    memcpy(&time2,&time1,sizeof(time1));
    //printf("\n-----------dt:%ld-----------\n", dt);
    boost::mutex::scoped_lock lock(pos_mutex);
    memcpy(&xyz, &pos, sizeof(pos));
    if(dt<14000000)
    {
        vx = (xyz.pos[0]-xyz_old.pos[0])*120;
        vy = (xyz.pos[1]-xyz_old.pos[1])*120;
        vz = (xyz.pos[2]-xyz_old.pos[2])*120;
    }
    memcpy(&xyz_old, &xyz,sizeof(xyz));

    //ROS_INFO_STREAM("callback: thread_id="<<boost::this_thread::get_id());
}

// to be complete------------------
int send(demo_test::pos_data& xyz, int fd)
{
    //to be complete
    ROS_INFO("sending temp_xyz---uav1: x=%.4f, y=%.4f, z=%.4f", xyz.pos[0], xyz.pos[2], -xyz.pos[1]);

    char buffer[44]={0xFE,36,0,1,50,138};
	unsigned int seq=1;
	typedef struct _mocap_data
	{
        uint64_t time_usec;
		float q[4];
		float x;
		float y;
		float z;
	}mocap_data;
	mocap_data _data;
    memset(&_data, 0, sizeof(_data));
	_data.x = xyz.pos[0];
	_data.y = xyz.pos[2];
	_data.z = -xyz.pos[1];

    memcpy(buffer+6, &_data, sizeof(_data));
	buffer[2]=seq++;		
	buffer[42]=109;
	uint16_t crcTmp = crc_calculate((uint8_t *)(buffer+1),42);
	uint8_t ck[2];
	ck[0] = (uint8_t)(crcTmp & 0xFF);
	ck[1] = (uint8_t)(crcTmp >> 8);
	buffer[42]=ck[0];
	buffer[43]=ck[1];

    int write_n = 0;
	write_n=write(fd,buffer,44);
	if(write_n!=44)
	{
		printf("send error\n");
		return -1;
	}
    return 0;

}



int main(int argc, char **argv)
{
/*    if(argc<5)
    {
        printf("missing arguments\n");
        return -1;
    }
*/
    memset(&xyz, 0, sizeof(xyz));
    memset(&xyz_old, 0,sizeof(xyz_old));

    // setup_port
	int fd;
	fd = open_port(MODEMDEVICE);
	if(fd < 0)
	{
		return -1;
	}
	if (setup_port(fd) < 0)
	{
		return -1;
	}
	printf("%s is open...\n", MODEMDEVICE);

    //ros init
    ros::init(argc, argv, "demo_client");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("demo_udp", 1000, udpcallback);
    
    //motion capture postion data
    demo_test::pos_data temp_xyz;
    memset(&temp_xyz, 0, sizeof(temp_xyz));
    float temp_vx = 0;
    float temp_vy = 0;
    float temp_vz = 0;

    int i = 0;

    ros::AsyncSpinner s(1);
    s.start();

    ros::Rate loop_rate(26);
    while(ros::ok())
    {
        //ROS_INFO_STREAM("main: thread_id="<<boost::this_thread::get_id());

        //lock mutex
        boost::mutex::scoped_lock lock(pos_mutex);
        memcpy(&temp_xyz, &xyz, sizeof(xyz));
        temp_vx = vx;
        temp_vy = vy;
        temp_vz = vz;
        lock.unlock();

        if(temp_xyz.num>0)
        {
            ROS_INFO("%d sending data to %d uav wirelessly", ++i, temp_xyz.num);
            send(temp_xyz, fd);
        }
        loop_rate.sleep();
    }
    return 0;
}

