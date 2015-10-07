/*************************************************************************
	> File Name: client.cpp
	> Author: 
	> Mail: 
	> Created Time: 2015年07月28日 星期二 17时09分59秒
 ************************************************************************/
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/types.h>
#include<unistd.h>
#include<sys/socket.h>
#include<netdb.h>
#include<arpa/inet.h>
#include<netinet/in.h>

#define MCAST_PORT 1511 
#define MCAST_ADDR "239.255.42.99"

using namespace std;
struct rigid_pos
{
    unsigned int num;
    float pos[6][3];
};

int unpack(char *pdata, rigid_pos *pos);

int main(void)
{
    int sock;
    struct sockaddr_in local_addr;
    struct sockaddr_in temp_addr;
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
    {
        cout<<"socket error"<<endl;
        return -1;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    memset(&temp_addr, 0, sizeof(temp_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//    local_addr.sin_addr.s_addr = inet_addr("169.254.6.162");
    local_addr.sin_port = htons(MCAST_PORT);
    
    int err = 0;
    int value = 1;
//    err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &local_addr.sin_addr.s_addr, sizeof(local_addr.sin_addr.s_addr));
    err = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value));
    if (err < 0)
    {
        cout<<"set reuseaddr error"<<endl;
        return -1;
    }
    err = bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr));
    if (err < 0)
    {
        cout<<"bind error"<<endl;
        return -1;
    }

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(MCAST_ADDR);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
//    mreq.imr_interface.s_addr = inet_addr("169.254.6.162");
    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    if (err < 0)
    {
        cout<<"set sock ip_add_membership error"<<endl;
        return -1;
    }

    socklen_t addr_len = 0;
    addr_len = sizeof(temp_addr);
    char buff[1000];
    cout<<"start: local_addr is "<<inet_ntoa(local_addr.sin_addr)<<" now."<<endl;
    int n = 0;
    int i = 0;
    int j = 0;
    rigid_pos rigidbody_pos;
    while(1)
    {
        //i--;
        n = recvfrom(sock, buff, 1000, 0, (struct sockaddr*)&temp_addr, &addr_len);
    //    n = read(sock, buff, sizeof(buff));
    //    if (n < 0)
    //    {
    //        cout<<"recvfrom() errro"<<endl;
    //        return -2;
    //    }
        if (n>0)  
        cout<<"receive "<<n<<" bytes ";
        cout<<"from "<<inet_ntoa(temp_addr.sin_addr)<<endl;
        int test;
        test = unpack(buff, &rigidbody_pos);
        if (test > 0)
        {
            j++;
            printf("rigidbody_pos.num: %d\n", rigidbody_pos.num);
            for(int k=0; k<rigidbody_pos.num; k++)
            {
                printf("rigid_pos: id:%d, x:%f, y:%f, z:%f\n", k, rigidbody_pos.pos[k][0], rigidbody_pos.pos[k][1],rigidbody_pos.pos[k][2]);
            }
        }
        i++;
        printf("i++: %d\n", i);
        printf("j++: %d\n", j);
        //
        
    }
}

int unpack(char *pdata, rigid_pos *pos)
{
    int major = 2;
    int minor = 7;
 //decoding
   char *ptr =pdata;
   int msgid = 0;
   memcpy(&msgid, ptr, 2);
   if (msgid == 7)
    {
        printf("\nBegin Packet\n---------------------\n");
        //1 msgid
        msgid = 0;
        memcpy(&msgid, ptr, 2); ptr += 2;
        printf("msgid: %d\n", msgid);
        //2 bytes count
        int nbytes = 0;
        memcpy(&nbytes, ptr, 2); ptr += 2;
        printf("bytes count: %d\n", nbytes);
        //3 frame number
        int framenumber = 0;
        memcpy(&framenumber, ptr, 4); ptr += 4;
        printf("Frame number: %d\n", framenumber);
        //4 number of data sets (markersets, rigidbodies, etc)
        int nmarksets = 0;
        memcpy(&nmarksets, ptr, 4); ptr += 4;
        printf("marker sets count: %d\n", nmarksets);
        //5 unidentified markers
        int othermarkers = 0;
        memcpy(&othermarkers, ptr, 4); ptr += 4;
        printf("unidentified marker count: %d\n", othermarkers);
        //6 rigid bodies
        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        printf("rigid body count: %d\n", nRigidBodies);
        //6.1 rigid bodies 
        for (int j=0; j < nRigidBodies; j++)
        {
            //6.1 rigid body pos/ori 32bytes
            int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
            float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
            float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
            float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
            float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
            float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
            float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
            float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
            printf("ID : %d\n", ID);
            printf("pos: [%4.3f,%4.3f,%4.3f]\n", x,y,z);
            printf("ori: [%4.3f,%4.3f,%4.3f,%4.3f]\n", qx,qy,qz,qw);
            
            pos->pos[j][0] = x;
            pos->pos[j][1] = y;
            pos->pos[j][2] = z;
            pos->num = j+1;

            //6.2 associated marker positions  40bytes
            int nRigidMarkers = 0;
            memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
            printf("Marker Count: %d\n", nRigidMarkers);
            int nBytes = nRigidMarkers*3*sizeof(float);
            float* markerData = (float*)malloc(nBytes);
            memcpy(markerData, ptr, nBytes); ptr += nBytes;
            
            if(major >= 2)
            {
                //6.3 associated marker IDs  12bytes
                nBytes = nRigidMarkers*sizeof(int);
                int* markerIDs = (int*)malloc(nBytes);
                memcpy(markerIDs, ptr, nBytes);
                ptr += nBytes;

                //6.4 associated marker sizes  12bytes
                nBytes = nRigidMarkers*sizeof(float);
                float* markerSizes = (float*)malloc(nBytes);
                memcpy(markerSizes, ptr, nBytes);
                ptr += nBytes;

                for(int k=0; k < nRigidMarkers; k++)
                {
                    printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
                }

                if(markerIDs)
                    free(markerIDs);
                if(markerSizes)
                    free(markerSizes);

            }
            else
            {
                for(int k=0; k < nRigidMarkers; k++)
                {
                    printf("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k, markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
                }
            }
            if(markerData)
                free(markerData);

            if(major >= 2)
            {
                //6.5 Mean marker error
                float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
                printf("Mean marker error: %3.2f\n", fError);
            }

            // 2.6 and later
            if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
            {
                //6.6 params
                short params = 0; memcpy(&params, ptr, 2); ptr += 2;
                bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
            }
            
        } // next rigid body
        //
        //7 skeletons
        if ( ((major == 2)&&(minor > 0)) || (major > 2) )
        {
            int nSkeletons = 0;
            memcpy(&nSkeletons, ptr, 4); ptr+=4;
            printf("Skeleton count: %d\n", nSkeletons);
            if (nSkeletons > 0)
                printf("error: Skeleton count > 0\n");
        }
        //8 Labeled Markers
        int nLabeledMarkers = 0;
        memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
        printf("Labeled Marker count: %d\n", nLabeledMarkers);
        //9 latency
        float latency = 0.0f;
        memcpy(&latency, ptr, 4); ptr += 4;
        printf("latency: %3.3f\n", latency);
        //10 timecode
        unsigned int timecode = 0;
        memcpy(&timecode, ptr, 4); ptr += 4;
        unsigned int timecodesub = 0;
        memcpy(&timecodesub, ptr, 4); ptr +=4;
        //11 timestamp
        double timestamp = 0.0f;
        if ( ((major == 2)&&(minor>=7)) || (major>2) )
        {
            memcpy(&timestamp, ptr, 8); ptr += 8;
        }
        else
        {
            float ftemp = 0.0f;
            memcpy(&ftemp, ptr, 4); ptr += 4;
            timestamp = (double)ftemp;
        }
        // frame param
        short params = 0;
        memcpy(&params, ptr, 2); ptr += 2;
        bool bIsRecording = params & 0x01;             //0x01  Motive is recording 
        bool bTrackedModelsChanged = params & 0x02;    //0x02  Actively tracked model list has changed 

        //13 end of data tag
        int eod = 0; 
        memcpy(&eod, ptr, 4); ptr += 4;
        printf("eod: %d\n", eod);
        printf("End Packet\n----------------\n");
        return 1;
            
    }
   else
   {
       printf("unrecognized packet\n");
       return -1;
   }

}
