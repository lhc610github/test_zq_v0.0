/*************************************************************************
	> File Name: temp_file.cpp
	> Author: 
	> Mail: 
	> Created Time: 2015年09月11日 星期五 15时49分40秒
 ************************************************************************/

#include<fcntl.h>
#include<stdio.h>
#include<unistd.h>

int main(int argc, char** argv)
{
    int fd;
    fd = open("datafile", O_WRONLY | O_CREAT);
    if(fd==-1)
    {
        printf("open file error\n");
        return -1;
    }
    struct test_data
    {
        unsigned int i;
        float xyz[3];
    };
    test_data _data[5];
    for(int i=0; i<5; i++)
    {
        _data[i].i = i+1;
        _data[i].xyz[0] = i+0.1;
        _data[i].xyz[1] = i+0.2;
        _data[i].xyz[2] = i+0.3;
    }
    int ret = write(fd, _data, sizeof(_data));
    printf("wrote %d bytes\n", ret);
    if(close(fd)==-1)
    {
        printf("close file error\n");
        return -1;
    }
    return 0; 
}

