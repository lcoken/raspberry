/*
 **************************************************************************
 * @file    main.c
 * @author  Feilong Wang
 * @version V1.2.0
 * @date    Oct 25, 2017
 **************************************************************************
 * @attetion
 *
 * Copyright(c) 2017 QXWZ Corporation.  All rights reserved.
 */
#include <time.h>
#include <unistd.h>
#include "qxwz_trace.h"
#include "qxwz_types.h"
#include "qxwz_sdk.h"


static char* s_gga = "";

static const qxwz_usr_config_t s_config = {
    "",
    "",
    "",
    ""
};

//#undef DEBUG
//#ifdef DEBUG
static void receive_iprtcm(qxwz_void_t *rtcm, qxwz_u32_t len, qxwz_data_type_e type)
{
    printf("got ip rtcm\n");
}

static void receive_status(qxwz_s32_t status)
{
    printf("got rtcm status=%d\n",status);
}

qxwz_data_response_t data_res = {
	receive_iprtcm,
	NULL
};

qxwz_status_response_t status_res = {
	receive_status
};

qxwz_s32_t main()
{	
	// The sdk current time
	static int s_current_time = 0;
    qxwz_s32_t ret = 0;
    s_current_time = time(NULL);

	/***********SDK API***********/
    qxwz_setting(&s_config,QXWZ_TRUE);
    ret = qxwz_start(&data_res,&status_res);
    if(0 != ret)
        return 0;
    qxwz_s32_t j = 0;
    while(1){
        
        ret = qxwz_tick(s_current_time);
        sleep(1);
        s_current_time += 1;
        if(j>5){
		qxwz_send_data(s_gga,strlen(s_gga)+1,UDATA_GGA);
                j = 0;
        }
        j++;
    }

    qxwz_stop();
    qxwz_release();
    return 0;	
}


