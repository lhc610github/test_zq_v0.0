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
boost::mutex pos_sp_mutex;
boost::mutex send_mutex;
struct timespec time1 = {0, 0};
struct timespec time2 = {0, 0};
static bool flag = true;
float send_currentpos_freq=1;
float send_desirepos_freq=1;
float ref_data[4];

int send_write(int fd,char *buffer,int length);

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
  //  clock_gettime(CLOCK_MONOTONIC, &time1);
    //long int dt = (time1.tv_sec-time2.tv_sec)*1000000000+time1.tv_nsec-time2.tv_nsec;  
    //memcpy(&time2,&time1,sizeof(time1));
    //printf("\n-----------dt:%ld-----------\n", dt);
    boost::mutex::scoped_lock lock(pos_mutex);
    memcpy(&xyz, &pos, sizeof(pos));
    /*if(dt<14000000)
    {
        vx = (xyz.pos[0]-xyz_old.pos[0])*120;
        vy = (xyz.pos[1]-xyz_old.pos[1])*120;
        vz = (xyz.pos[2]-xyz_old.pos[2])*120;
    }*/
    //memcpy(&xyz_old, &xyz,sizeof(xyz));

    //ROS_INFO_STREAM("callback: thread_id="<<boost::this_thread::get_id());
}

// to be complete------------------
int send_current_pos(int fd)
{
    //to be complete
	 
    
	
  

    char buffer[20]={0xFE,12,0,1,50,232};
	unsigned int seq=1;
	typedef struct _mocap_data
	{
		float x;
		float y;
		float z;
	}mocap_data;
	mocap_data _pos_data;
	uint8_t ck[2];
   	int write_n;
	
	float delay_time;
	ros::Rate loop_rate_send_current(send_currentpos_freq);
	while(1)
	{
	//delay_time=1000000/send_currentpos_freq-20;
	boost::mutex::scoped_lock lock(pos_mutex);
	_pos_data.x = xyz.pos[0];
	_pos_data.y = xyz.pos[2];
	_pos_data.z = -xyz.pos[1];
	lock.unlock();
ROS_INFO("sending position_feedback to uav1: x=%.4f, y=%.4f, z=%.4f", _pos_data.x, _pos_data.y, _pos_data.z);
    	memcpy(buffer+6, &_pos_data, sizeof(_pos_data));
	buffer[2]=seq++;		
	buffer[18]=88;
	uint16_t crcTmp = crc_calculate((uint8_t *)(buffer+1),18);
	
	ck[0] = (uint8_t)(crcTmp & 0xFF);
	ck[1] = (uint8_t)(crcTmp >> 8);
	buffer[18]=ck[0];
	buffer[19]=ck[1];

        write_n = 0;
	boost::mutex::scoped_lock send_lock(send_mutex);
	write_n=write(fd,buffer,20);
	//write_n=send_write(fd,buffer,20);	
	send_lock.unlock();
	if(write_n!=20)
	{
		printf("send current position feedback error\n");
		return -1;
	}
	else
	{ ROS_INFO("sending position_feedback to uav1: successful");
	}
	
	loop_rate_send_current.sleep();
	}
return 0;
}

//-------------------------------------------------------------------------------
//
int send_desire_pos(int fd)
{

 


    char buffer2[24]={0xFE,16,0,1,50,233};
	unsigned int seq2=1;
	typedef struct _pos_sp_data
	{
		float x_d;
		float y_d;
		float z_d;
        	float yaw_d;
	}pos_sp_data;
	pos_sp_data _xyz_sp;
	
	uint8_t ck2[2];
	int write_n2;
	float delay_time;
	ros::Rate loop_rate_send_desire(send_desirepos_freq);
	while(1){
	//delay_time=1000000/send_desirepos_freq-20;
	boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
	_xyz_sp.x_d = ref_data[0];
	_xyz_sp.y_d = ref_data[1];
	_xyz_sp.z_d = ref_data[2];
    	_xyz_sp.yaw_d = ref_data[3];
	possp_lock.unlock();
  ROS_INFO("sending position setpoint to uav1: x_d=%.4f, y_d=%.4f, z_d=%.4f, yaw_d=%.4f\n", _xyz_sp.x_d, _xyz_sp.y_d, _xyz_sp.z_d, _xyz_sp.yaw_d);
    	memcpy(buffer2+6, &_xyz_sp, sizeof(_xyz_sp));
	buffer2[2]=seq2++;		
	buffer2[22]=193;
	uint16_t crcTmp2 = crc_calculate((uint8_t *)(buffer2+1),22);
	
	ck2[0] = (uint8_t)(crcTmp2 & 0xFF);
	ck2[1] = (uint8_t)(crcTmp2 >> 8);
	buffer2[22]=ck2[0];
	buffer2[23]=ck2[1];
        write_n2 = 0;
	boost::mutex::scoped_lock send_lock(send_mutex);
	write_n2=write(fd,buffer2,24);
	//write_n2=send_write(fd,buffer2,24);	
	send_lock.unlock();
	if(write_n2!=24)
	{
		printf("send desire position setpoint  error\n");
		return -1;
	}
	else
	{ ROS_INFO("sending position setpoint to uav1: successful");
	}
	loop_rate_send_desire.sleep();
	}
    return 0;

}
////////////////////////////////////////////////////////////////////////

int send_write(int fd,char *buffer,int length)
{
int status_write;
long int dt;

    clock_gettime(CLOCK_MONOTONIC, &time1);
    dt = (time1.tv_sec-time2.tv_sec)*1000000000+time1.tv_nsec-time2.tv_nsec;  
    
if (dt>(1000000000/40))
	{
		status_write=write(fd,buffer,length);
	}
else 
	{	usleep(dt/1000);
	status_write=write(fd,buffer,length);
	}
	memcpy(&time2,&time1,sizeof(time1));
return status_write;
}

//////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    if(argc<5)
    {
        printf("missing arguments\n");
        return -1;
    }

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
    
    //read trajectory data from file argv[1]
    ROS_INFO("argv[0]:%s",argv[0]);
    ROS_INFO("argv[1]:%s",argv[1]);
    ROS_INFO("argv[2]:%s",argv[2]);
    ROS_INFO("argv[3]:%s",argv[3]);
    ROS_INFO("argv[4]:%s",argv[4]);
    boost::mutex::scoped_lock possp_lock(pos_sp_mutex);
    ref_data[0] = atof(argv[1]);
    ref_data[1] = atof(argv[2]);
    ref_data[2] = atof(argv[3]);
    ref_data[3] = atof(argv[4]);
	possp_lock.unlock();
    //motion capture postion data
    demo_test::pos_data temp_xyz;
    memset(&temp_xyz, 0, sizeof(temp_xyz));
    float temp_vx = 0;
    float temp_vy = 0;
    float temp_vz = 0;

    int i = 0;

boost::thread send_current(boost::bind(&send_current_pos,fd));
boost::thread send_desire(boost::bind(&send_desire_pos,fd));

    ros::AsyncSpinner s(1);
    s.start();

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        //ROS_INFO_STREAM("main: thread_id="<<boost::this_thread::get_id());

        //lock mutex
   //     boost::mutex::scoped_lock lock(pos_mutex);
     //   memcpy(&temp_xyz, &xyz, sizeof(xyz));
       // temp_vx = vx;
        //temp_vy = vy;
        //temp_vz = vz;
        //lock.unlock();

       /* if(temp_xyz.num>0)
        {
            ROS_INFO("%d sending data to %d uav wirelessly", ++i, temp_xyz.num);
            send(temp_xyz, ref_data, fd);
        }*/
	ROS_INFO("RUNING... 1 s");		
        loop_rate.sleep();
    }
    return 0;
}


