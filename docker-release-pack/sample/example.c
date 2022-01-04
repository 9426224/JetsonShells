#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cqbit_target2mas.h"



char *test = "{\"device_id\":\"1000\",\"time_stamp\":1628679021,\"object_info\":[{\"target_id\":1,\"sign\":1,\"vx\":0.1,\"vy\":0.3,\"lng\":114.53758,\"lat\":39.578908,\"class\":\"car\",\"course_angle\":56.7},{\"target_id\":2,\"sign\":1,\"vx\":0.1,\"vy\":0.3,\"lng\":114.53758,\"lat\":39.578908,\"class\":\"car\",\"course_angle\":56.7}]}";

int main()
{
    /* 1. 建立TCP 链接 */

    int socket = tcp_connected("ined.racodf.com", 63098);

    while(1)
    {
        /* 1. 转换JSON数据，发送结果 */
        int flag = trans_json2bin_mas_send(test, socket);
        if (flag != 0)
        {
            /* 失败重连 */
            socket = tcp_connected("ined.racodf.com", 63098);
            printf("failed\n");
        }
        sleep(1);
    }
    
    printf("hello cJaon\n");

    return 0;
}
