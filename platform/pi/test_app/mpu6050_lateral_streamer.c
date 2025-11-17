/**
 * @file mpu6050_lateral_streamer.c
 * @brief IMU lateral motion streamer using only accelerometer data (no gyro, no quaternion).
 *
 * Build:
 *   gcc -O2 -Wall -Wextra -o mpu6050_lateral_streamer mpu6050_lateral_streamer.c -lm
 *
 * Description:
 *   - Calibrates accelerometer bias (assuming stillness).
 *   - Applies low-pass filtering and moving-average smoothing.
 *   - Integrates acceleration to velocity and position.
 *   - Uses ZUPT (zero-velocity update) to stop drift when still.
 *   - Streams data as JSON lines over TCP for viewer.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <math.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "mpu6050_ioctl.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// -----------------------------------------------------------
// helpers
// -----------------------------------------------------------
static void msleep(unsigned ms){
    struct timespec ts={.tv_sec=ms/1000,.tv_nsec=(long)(ms%1000)*1000000L};
    nanosleep(&ts,NULL);
}
static int64_t mono_ns(void){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    return (int64_t)ts.tv_sec*1000000000LL + ts.tv_nsec;
}
static int open_mpu6050(void){
    glob_t g;
    if(glob("/dev/mpu6050-*",0,NULL,&g)!=0||g.gl_pathc==0){globfree(&g);return -ENOENT;}
    int fd=open(g.gl_pathv[0],O_RDONLY|O_CLOEXEC);
    int err=(fd<0)?-errno:0; globfree(&g); return (fd<0)?err:fd;
}
static int read_sample(int fd, struct mpu6050_sample *s){
    ssize_t r=read(fd,s,sizeof(*s));
    if(r==(ssize_t)sizeof(*s)) return 0;
    return (r<0)?-errno:-EIO;
}


static void raw_to_si(const struct mpu6050_fs* fs, const struct mpu6050_sample* s,float gyro_radps[3], float accel_mps2[3]) 
{
    const float g_range = (fs->accel == ACCEL_2G) ? 16384.0f :
                          (fs->accel == ACCEL_4G) ? 8192.0f :
                          (fs->accel == ACCEL_8G) ? 4096.0f : 2048.0f;
    const float dps_range = (fs->gyro == GYRO_250DPS) ? 131.0f :
                            (fs->gyro == GYRO_500DPS) ? 65.5f :
                            (fs->gyro == GYRO_1000DPS) ? 32.8f : 16.4f;

    s->ax = s->ax/g_range;
    s->ay = s->ay/g_range;
    s->az = s->az/g_range;
    s->gx = s->gx/dps_range;
    s->gy = s->gy/dps_range;
    s->gz = s->gz/dps_range;
    
    
    const float a_lsb_to_mps2 = (g_range * 9.860665f) / 32768.0f; // m/s^2 per LSB
    const float g_lsb_to_rad = ((dps_range * (float)M_PI / 180.0f) / 32768.0f); // rad/s per LSB
    accel_mps2[0] = s->ax * a_lsb_to_mps2;
    accel_mps2[1] = s->ay * a_lsb_to_mps2;
    accel_mps2[2] = s->az * a_lsb_to_mps2;
    gyro_radps[0] = s->gx * g_lsb_to_rad;
    gyro_radps[1] = s->gy * g_lsb_to_rad;
    gyro_radps[2] = s->gz * g_lsb_to_rad;
}
// -----------------------------------------------------------
// config
// -----------------------------------------------------------
typedef struct {
    int port;
    int rate_hz;
    int calib_s;
} cfg_t;

static void parse_args(int argc,char**argv,cfg_t*c){
    c->port=9010; c->rate_hz=100; c->calib_s=5;
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--port")&&i+1<argc) c->port=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--rate_hz")&&i+1<argc) c->rate_hz=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--calib_s")&&i+1<argc) c->calib_s=atoi(argv[++i]);
    }
}

// -----------------------------------------------------------
// main
// -----------------------------------------------------------
int main(int argc,char**argv){
    cfg_t C; parse_args(argc,argv,&C);
    int fd=open_mpu6050();
    if(fd<0){fprintf(stderr,"open failed: %s\n",strerror(-fd));return 1;}

    struct mpu6050_fs fs={0};
    if(ioctl(fd,MPU6050_IOC_GET_FS,&fs)!=0){
        fprintf(stderr,"GET_FS failed: %s\n",strerror(errno));return 1;
    }

    fprintf(stderr,"[INFO] Calibrating accelerometer for %d s ... keep IMU still\n",C.calib_s);
    double suma[3]={0};
    struct mpu6050_sample s;
    int N=C.calib_s*C.rate_hz;
    for(int i=0;i<N;i++){
        if(read_sample(fd,&s)==0){
            float a[3]; raw_to_si(&fs,&s,a);
            for(int k=0;k<3;k++) suma[k]+=a[k];
        }
        msleep(1000/C.rate_hz);
    }

    float ba[3]={suma[0]/N,suma[1]/N,suma[2]/N};
    fprintf(stderr,"[CAL] Accel bias: [%.6f %.6f %.6f] m/s²\n",ba[0],ba[1],ba[2]);

    // TCP server
    int srv=socket(AF_INET,SOCK_STREAM,0);
    int yes=1; setsockopt(srv,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
    struct sockaddr_in addr={0};
    addr.sin_family=AF_INET; addr.sin_addr.s_addr=INADDR_ANY; addr.sin_port=htons(C.port);
    bind(srv,(struct sockaddr*)&addr,sizeof(addr));
    listen(srv,1);
    fprintf(stderr,"[INFO] Starting TCP server on port %d\n",C.port);

    // smoothing buffers
    const int MA_WIN=10;
    float ma_buf[3][MA_WIN]; int ma_idx=0; int ma_fill=0;

    for(;;){
        struct sockaddr_in ca; socklen_t calen=sizeof(ca);
        int cs=accept(srv,(struct sockaddr*)&ca,&calen);
        if(cs<0){perror("accept");continue;}
        fprintf(stderr,"[INFO] Client connected: %s\n",inet_ntoa(ca.sin_addr));

        float vel[3]={0},pos[3]={0};
        float a_lp[3]={0};
        bool lp_init=false;
        int64_t t_prev=mono_ns();
        int still_cnt=0;

        for(;;){
            if(read_sample(fd,&s)!=0){msleep(1);continue;}
            float a_raw[3]; raw_to_si(&fs,&s,a_raw);

            // bias correction
            float a[3]={a_raw[0]-ba[0],a_raw[1]-ba[1],a_raw[2]-ba[2]};

            // simple 1st-order IIR low-pass
            const float alpha=0.1f;
            if(!lp_init){memcpy(a_lp,a,sizeof(a_lp));lp_init=true;}
            else for(int i=0;i<3;i++) a_lp[i]+=alpha*(a[i]-a_lp[i]);

            // moving average
            for(int k=0;k<3;k++){ ma_buf[k][ma_idx]=a_lp[k]; }
            ma_idx=(ma_idx+1)%MA_WIN;
            if(ma_fill<MA_WIN) ma_fill++;
            float a_ma[3]={0};
            for(int k=0;k<3;k++){
                for(int j=0;j<ma_fill;j++) a_ma[k]+=ma_buf[k][j];
                a_ma[k]/=(float)ma_fill;
            }

            // time step
            int64_t t_now=mono_ns();
            float dt=(float)((t_now-t_prev)/1e9);
            if(dt<1e-4f) dt=1.0f/C.rate_hz;
            t_prev=t_now;

            // compute magnitude for stillness detection
            float amag=sqrtf(a_ma[0]*a_ma[0]+a_ma[1]*a_ma[1]+a_ma[2]*a_ma[2]);
            bool still=fabsf(amag)<0.05f; // adjust threshold (m/s²)

            // adaptive bias correction
            const float k_bias=0.001f;
            if(still){
                for(int i=0;i<3;i++)
                    ba[i]=(1.0f-k_bias)*ba[i]+k_bias*a_raw[i];
            }

            // Zero Velocity Update (ZUPT)
            static const int STILL_CONFIRM=5;
            if(still){ still_cnt++; } else { still_cnt=0; }
            if(still_cnt>STILL_CONFIRM){
                vel[0]=vel[1]=vel[2]=0;
                continue; // skip integration completely while still
            }

            // integrate to velocity
            for(int i=0;i<3;i++) vel[i]+=a_ma[i]*dt;

            // soft velocity decay to remove drift
            const float vel_decay=0.005f;
            for(int i=0;i<3;i++) vel[i]*=(1.0f-vel_decay);

            // integrate to position
            for(int i=0;i<3;i++) pos[i]+=vel[i]*dt;

            // clamp range
            const float maxr=5.0f;
            for(int i=0;i<3;i++){if(pos[i]>maxr)pos[i]=maxr;if(pos[i]<-maxr)pos[i]=-maxr;}

            // send JSON packet
            char line[256];
            int n=snprintf(line,sizeof(line),
                "{\"t_ns\":%lld,\"pos_m\":[%.4f,%.4f,%.4f],"
                "\"vel_mps\":[%.4f,%.4f,%.4f],\"lin_acc_mps2\":[%.4f,%.4f,%.4f]}\n",
                (long long)t_now,pos[0],pos[1],pos[2],
                vel[0],vel[1],vel[2],
                a_ma[0],a_ma[1],a_ma[2]);

            if(send(cs,line,n,MSG_NOSIGNAL)<=0){
                fprintf(stderr,"[INFO] Client disconnected\n");
                break;
            }

            msleep(1000/C.rate_hz);
        }
        close(cs);
    }
    close(srv);
    close(fd);
    return 0;
}
