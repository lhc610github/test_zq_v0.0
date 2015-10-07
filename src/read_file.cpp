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
    if(argc<=1)
    {
        printf("usage error!\nusage: ./read_file  data.bin\n");
        return -1;
    }
    int fd;
    //open file first time
    fd = open(argv[1], O_RDONLY);
    if(fd==-1)
    {
        printf("open file error\n");
        return -1;
    }
    
    //read file, calculate data sets-------i
    float xyz[3];
    int count = 0;
    int i = 0;
    while(count = read(fd, xyz, 12))
    {
        if(count==12)
        {
            i++;
            printf("data: i=%d, x=%.4f, y=%.4f, z=%.4f\n", i, xyz[0], xyz[1], xyz[2]);
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
    
    //read file second time, copy data sets from disk to memory
    fd = open(argv[1], O_RDONLY);
    if(fd==-1)
    {
        printf("open file error\n");
    }

    float data[3*i];
    char *ptr = (char *)&data;
    while(count = read(fd, ptr, sizeof(data)))
    {
        if(count==sizeof(data))
        {
            printf("read data successfully\n");
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

