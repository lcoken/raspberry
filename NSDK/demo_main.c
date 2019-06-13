/*
 * Copyright (c) 2018 Qianxun SI Inc. All rights reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "qxwz_sdk_hl.h"

//this is fake account info,please use the corrent to replace it.

#define APPKEY                  "5074f5b"
#define APPSECRET               "ad0612808626d1fc0c71fd4ad50a34b164f4a3510ae7fdce824968d9432b519a"
#define DEVICE_ID               "ID01"
#define DEVICE_TYPE             "typeA"

#define MC120_TYPE_M              0
#define MC120_TYPE_A              1

//root working dir which QXWZ SDK has write/read permission
#define ROOT_DIR                "./data/"
//NOTE:Magic Cube Device Node, please use the correct dev node according to your platform.
#define MC120_SERIAL_DEV         "/dev/ttyUSB0"
#define DEFAULT_SERIAL_BAUT     (115200)
#define DEFAULT_MC120M_BAUT     (9600)
// since the MC120_TYPE_M device only ouput nmea sentence in 1HZ(1000ms).
#define SERIAL_TIME_OUT_MS      (1050)

#define QXLOGI printf
#define QXLOGE printf

//commands to config ublox for MC120M, send these commands to MC120M before inject magic cubic data to QXWZ SDK
static unsigned char cmd10[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x28, 0x4E };	//1 02-15 RXM-RAWX    16
static unsigned char cmd13[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E };	// 115200
//static unsigned char cmd15[] = { 0xB5, 0x62, 0x06, 0x3E, 0x34, 0x00, 0x00, 0x00, 0x20, 0x06, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x14, 0xE1};	//GPS+BD
static unsigned char cmd15[] = { 0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2E, 0x75};	//GPS+BD+QZSS

static unsigned char cmd22[] = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F };	//NAV_SOL
static unsigned char cmd23[] = { 0xB5,0x62,0x06,0x31,0x20,0x00,0x00,0x01,0x00,0x00,0x32,0x00,0x00,0x00,0x40,0x42,0x0F,0x00,0x40,0x42,0x0F,0x00,0x00,0x00,0x00,0x00,0xA0,0x86,0x01,0x00,0x00,0x00,0x00,0x00,0xF7,0x00,0x00,0x00,0xCA,0xB6 };//PPS
static unsigned char cmd24[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};//5HZ
static unsigned char cmd25[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x25, 0x3d}; //sub-frame


static int g_serial_fd = 0;//handler of serial device.
static QXWZSdkConfig default_cfg = {0}; //default config of QXSDK.

static int hl_main_opt(const char *dev, int type);
static int do_sdk_loop(int serial_fd, QXWZSdkConfig *pcfg);

/**
 *  change baut rate and set flags.
 */
static int set_serial_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    QXLOGI("Enter \n");
    struct termios newtio = {0};

    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_iflag &= ~( IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR );//for linux

    switch ( nBits ) {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch ( nEvent ) {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch ( nSpeed ) {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 230400:
        cfsetispeed(&newtio, B230400);
        cfsetospeed(&newtio, B230400);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    
    if ( nStop == 1 )
        newtio.c_cflag &=  ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |=  CSTOPB;

    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 1;

    newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);//for linux
    newtio.c_oflag &= ~OPOST;/*No Output Processing*/

    tcflush(fd, TCIOFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }

    QXLOGI("Leave \n");
    return 0;
}

/**
 *  make sure send out the whole frame.
 */
static void send_cmd2mc(int fd, unsigned char *cmd, int len)
{
    int offset = 0;
    size_t wcnt = 0;
    int total = len;

    do {
        wcnt = write(fd, cmd + offset, total);
        if (wcnt <= 0) {
            QXLOGI("write cmd to ublox failed....\n");
            usleep(1000);
            continue;
        } else {
            offset += wcnt;
            total -= wcnt;
        }
    } while (total > 0);

}

/**
 *  read data from seiral in noblock mode,and inject the data to QX NSDK.
 */
static int serial_poll(int serial_fd, QXWZSdkInterface *sdk_hl)
{
    fd_set input;
    int max_fd;
    int n;
    unsigned char data[4096];
    struct timeval  tv;

    while (1) {
        /* Initialize the input set */
        FD_ZERO(&input);
        FD_SET(serial_fd, &input);

        max_fd = serial_fd + 1;

        tv.tv_sec = SERIAL_TIME_OUT_MS / 1000;
        tv.tv_usec = (SERIAL_TIME_OUT_MS % 1000) * 1000;

        /* Do the select */
        n = select(max_fd,  &input, NULL, NULL, &tv);

        /* See if there was an error */
        if (n < 0) {
            perror("select failed");
//            break;
            continue;
        } else if (n == 0) {
            QXLOGE("serial read timeout in :%d (ms) \n", SERIAL_TIME_OUT_MS);
        } else {
            /* We have input */
            if (FD_ISSET(serial_fd, &input)) {
                size_t len = read(serial_fd, data, sizeof(data));
                if (sdk_hl && (len > 0) ) {
                    sdk_hl->inject_gnss_data(data, len);
                }

            }

        }
    }

    return 0;
}

/**
 *  @seiral_dev: the device name of the seiral in linux host.
 *  @type: MC120_TYPE_M or MC120_TYPE_A.
 */
static int hl_main_opt(const char *serial_dev, int type)
{
    QXLOGI("Enter \n");
    int ret;
    int set_gps;
    int serial_fd;
    QXWZSdkConfig * pcfg = &default_cfg;
    int baut = DEFAULT_SERIAL_BAUT;

    default_cfg.size           = sizeof(QXWZSdkConfig);
    default_cfg.log_enable     = 1;
    default_cfg.apply_scenario = QXWZ_GNSS_APPLY_SCENE_AUTOROOF;//customer should set this args by their scenario.
    sprintf(default_cfg.app_key, "%s", APPKEY);
    sprintf(default_cfg.app_secret, "%s", APPSECRET);
    sprintf(default_cfg.device_id, "%s", DEVICE_ID);
    sprintf(default_cfg.device_type, "%s", DEVICE_TYPE);
    sprintf(default_cfg.root_dir, "%s", ROOT_DIR);
    sprintf(default_cfg.socket_dir, "%s", "/tmp");
    sprintf(default_cfg.cfg_filename,"./%s","testCfg.ini");


    QXLOGI("current account info: %s %s %s %s \n",
           pcfg->app_key,
           pcfg->app_secret,
           pcfg->device_id,
           pcfg->device_type);

    QXLOGI("opening device :%s \n", serial_dev);

    /* O_RDWR Read/Write access to serial port           */
    /* O_NOCTTY - No terminal will control the process   */
    /* O_NDELAY -Non Blocking Mode,Does not care about-  */
    /* -the status of DCD line,Open() returns immediatly */
    serial_fd = open(serial_dev, O_RDWR | O_NONBLOCK | O_NOCTTY); //open serial
    if (serial_fd == -1) {
        QXLOGI("open :%s failed \n", serial_dev);
        return -1;
    }

    /* NOTE: MUST CONFIG MC120M BY THESE COMMANDS BEFORE INJECT SERIAL DATA to QXWZ SDK */
    if (type == MC120_TYPE_M) {
        baut = DEFAULT_MC120M_BAUT;
        QXLOGI("communicate baut is %d \n", baut);
        set_gps = set_serial_opt(serial_fd, baut, 8, 'N', 1); //setting serial attribution
        if (set_gps == -1) {
            QXLOGI("config serial device failed on setting \n");
            return -1;
        }
        QXLOGI("start reading... \n");

        //configure for the MC120_TYPE_M module
        QXLOGI("configure the device....\n");
        sleep(1);
        send_cmd2mc(serial_fd, cmd10, sizeof(cmd10));
        usleep(100000);
        send_cmd2mc(serial_fd, cmd15, sizeof(cmd15));
        usleep(100000);
        send_cmd2mc(serial_fd, cmd25, sizeof(cmd25));
        usleep(100000);
        send_cmd2mc(serial_fd, cmd22, sizeof(cmd22));
        usleep(100000);
        send_cmd2mc(serial_fd, cmd23, sizeof(cmd23));
        usleep(100000);
        send_cmd2mc(serial_fd, cmd13, sizeof(cmd13)); //change baud rate to 115200
        sleep(1);
        QXLOGI("configure over\n");

        close(serial_fd);
        QXLOGI("re-opening device %s\n", serial_dev);
        serial_fd = open(serial_dev, O_RDWR | O_NONBLOCK | O_NOCTTY); //open serial
        if (serial_fd == -1) {
            QXLOGI("open :%s failed \n", serial_dev);
            return -1;
        }

    }

    baut = DEFAULT_SERIAL_BAUT;
    QXLOGI("communicate baut is %d \n", baut);
    set_gps = set_serial_opt(serial_fd, baut, 8, 'N', 1); //CHANG BUAD
    if (set_gps == -1) {
        QXLOGI("failed to change baut:%d \n", baut);
        return -1;
    }

    g_serial_fd = serial_fd;
    /* After setting serial device, here start to read data & inject data to QXSDK  */
    ret = do_sdk_loop(serial_fd, pcfg);
    
    close(serial_fd);
    g_serial_fd = 0;

    QXLOGI("Leave");
    return ret;
}

//TODO: User should implement this callback if want to get position information by NMEA
static void cb_fill_nmea_info(QXGnssUtcTime time, const char* nmea, int len)
{
    if (nmea) {
        QXLOGI("nmea:%s", nmea);
    }
}

//TODO: User should implement this callback if want to get position information by paramter
static void cb_fill_position(QXWZGnssLocation *pos)
{
    if (pos) {
        QXLOGI("posflag:%d \n", pos->posflag);
    }
}

//NOTE: Write raw data back to Magic Cube device
static void cb_fill_raw_data(unsigned char *buf, int len)
{
    if (buf) {
        QXLOGI("len:%d \n", len);
    }

    if (g_serial_fd > 0) {
        send_cmd2mc(g_serial_fd, buf, len);
    }
}

//Notify status of SDK
static void cb_status_response(QXWZSdkStatus status)
{
    QXLOGI("status:%d \n", status);
}

/*  
 * This is demonstrating the API using.
 */
int do_sdk_loop(int serial_fd, QXWZSdkConfig* pcfg)
{
    int ret;
    const QXWZSdkInterface *sdk_hl = NULL;
    QXWZSdkCallbacks cbs;
    memset(&cbs, 0, sizeof(QXWZSdkCallbacks));
    cbs.size            = sizeof(cbs);
    cbs.fill_nmea_info  = cb_fill_nmea_info;
    cbs.fill_position   = cb_fill_position;
    cbs.fill_raw_data   = cb_fill_raw_data;
    cbs.status_response = cb_status_response;

    sdk_hl = getQXWZSdkInterface();
    //User should notify SDK of network status immediately once the network changes.
    //sdk_hl->update_conn_status(QXWZ_NET_TYPE_WIFI);
    sdk_hl->init(&cbs, pcfg);
    sdk_hl->start();


    /* [raw] => [QXSDK] */
    ret = serial_poll(serial_fd, (QXWZSdkInterface *)sdk_hl);

    sdk_hl->stop();
    sdk_hl->cleanup();

    QXLOGI("Leave");
    return ret;
}

/* This is a demo, Users need to operate UART of MC120A/MC120M, and inject data from UART to QXWZ SDK,
 * and control the call sequence 
 */
int  main(int argc, char *argv[])
{
    return hl_main_opt(MC120_SERIAL_DEV, MC120_TYPE_M);
}
