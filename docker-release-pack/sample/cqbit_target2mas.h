
#ifndef _CQBIT_TARGET2MAS_H_
#define _CQBIT_TARGET2MAS_H_

#ifdef __cplusplus
    extern "C" {
#endif

/*
    函数说明：tcp 链接
    函数名称：tcp_connected(char *ip, int port)
    参数说明：
        char *ip， ip
        int port， port
    返回值：
        >0，成功
        <0, 失败，需要重新链接socket
    备注：
        none
*/
int tcp_connected(char *ip, int port);

/*
    函数说明：将json格式的数据转换成梅安森使用的TCP二进制格式，并发送出去
    函数名称：trans_json2bin_mas_send(char* json_data_in, int socket)
    参数说明：
        char* json_data_in， json数据内容
        int socket        ， tcp链接的socket
    返回值：
        =0，成功
        <0, 失败，需要重新链接socket
    备注：
        1. 前置函数：必须首先调用tcp_connected函数获取到socket
        2. socket获取一次即可，之后可重复调用trans_json2bin_mas_send，除非返回值<0

*/
int trans_json2bin_mas_send(char* json_data_in, int socket);

#ifdef __cplusplus
	}
#endif

#endif /* _CQBIT_TARGET2MAS_H_ */
