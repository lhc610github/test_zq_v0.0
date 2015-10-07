#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include <string.h>
#include <boost/thread.hpp>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

demo_test::pos_data xyz;
boost::mutex pos_mutex;

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
    boost::mutex::scoped_lock lock(pos_mutex);
    memcpy(&xyz, &pos, sizeof(pos));
    ROS_INFO_STREAM("callback: thread_id="<<boost::this_thread::get_id());
}

// to be complete------------------
void send(demo_test::pos_data& xyz, float *data, int& i)
{
    //to be complete
    for(int k=0; k<xyz.num; k++,i++)
    {
        ROS_INFO("sending temp_xyz---uav_%d: x=%f, y=%f, z=%f", k, xyz.pos[k*3], xyz.pos[k*3+1], xyz.pos[k*3+2]);
        ROS_INFO("sending trajectory data---uav_%d: x_%d=%f, y_%d=%f, z_%d=%f", k, i/xyz.num, data[i*3], i/xyz.num, data[i*3+1], i/xyz.num, data[i*3+2]);
    }

}

//calculate how many data sets,    float x, y, z
int read_file_try(char *file);
int read_file(int num, char *data, char *file);


int main(int argc, char **argv)
{
    //ros init
    ros::init(argc, argv, "demo_client");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("demo_udp", 1000, udpcallback);
    
    //read trajectory data from file argv[1]
    ROS_INFO("argv[0]:%s",argv[0]);
    ROS_INFO("argv[1]:%s",argv[1]);
    //get number of data sets,  one set----12 bytes
    int count = read_file_try(argv[1]);
    ROS_INFO("count:%d", count);
    //trajectory data
    float data[count*3];
    int ret = read_file(count*3*4, (char *)&data, argv[1]);
    if(ret!=0)
    {
        printf("read data error\n");
        return -1;
    }
    ROS_INFO("data[2]:%f\n", data[2]);

    //motion capture postion data
    demo_test::pos_data temp_xyz;
    //trajectory data iterator
    int data_i = 0;

    ros::AsyncSpinner s(1);
    s.start();

    ros::Rate loop_rate(0.5);
    while(ros::ok())
    {
        ROS_INFO_STREAM("main: thread_id="<<boost::this_thread::get_id());

        //lock mutex
        boost::mutex::scoped_lock lock(pos_mutex);
        memcpy(&temp_xyz, &xyz, sizeof(xyz));
        lock.unlock();

        if(temp_xyz.num>0)
        {
            ROS_INFO("    sending data to %d uav wirelessly", temp_xyz.num);
            send(temp_xyz, data, data_i);
        }
        loop_rate.sleep();
    }
    return 0;
}

int read_file_try(char *file)
{
    int fd;
    fd = open(file, O_RDONLY);
    if(fd == -1)
    {
        ROS_INFO("open file error");
        return -1;
    }
    ROS_INFO("open file successfully");
    //read file, calculate data sets-------i
    float xyz[3];
    int count = 0;
    int i = 0;
    while(count = read(fd, xyz, 12))
    {
        if(count==12)
        {
            printf("data: i=%d, x=%.4f, y=%.4f, z=%.4f\n", i, xyz[0], xyz[1], xyz[2]);
            i++;
        }
        else
        {
            printf("end of file\n");
            break;
        }
    }
    if(close(fd)==-1)
    {
        printf("close file error\n");
    }

    return i;
}
int read_file(int num, char *data, char *file)
{
    int fd;
    fd = open(file, O_RDONLY);
    if(fd == -1)
    {
        ROS_INFO("open file error");
        return -1;
    }
    ROS_INFO("open file successfully");
    int count;
    while(count = read(fd, (char *)data, num))
    {
        if(count==num)
        {
            ROS_INFO("read data successfully-------bytes:%d",count);
        }
        else
        {
            printf("read data error\n");
            return -1;
        }
    }
    if(close(fd)==-1)
    {
        printf("close file error\n");
        return -1;
    }
    return 0; 
   
}
