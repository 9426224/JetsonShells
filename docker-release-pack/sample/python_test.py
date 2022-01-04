from ctypes import *
import time


library = cdll.LoadLibrary("libcqbit_target2mas.so")

library.tcp_connected.argtypes = [c_char_p, c_int]
library.tcp_connected.restype  = c_void_p

ip = create_string_buffer("image.racodf.com".encode('utf-8'))
port = c_int(6002)

ret_socket = library.tcp_connected(ip, port)

# trans_json2bin_mas_send(char* json_data_in, int socket)
library.trans_json2bin_mas_send.argtypes = [c_char_p, c_int]
library.trans_json2bin_mas_send.restype  = c_void_p

json_data = create_string_buffer("{\"device_id\":\"1000\",\"time_stamp\":1628679021,\"object_info\":[{\"target_id\":1,\"sign\":1,\"vx\":0.1,\"vy\":0.3,\"lng\":114.53758,\"lat\":39.578908,\"class\":\"car\",\"course_angle\":56.7},{\"target_id\":2,\"sign\":1,\"vx\":0.1,\"vy\":0.3,\"lng\":114.53758,\"lat\":39.578908,\"class\":\"car\",\"course_angle\":56.7}]}".encode('utf-8'))

while True:
    socket_val = c_int(ret_socket)
    library.trans_json2bin_mas_send(json_data, socket_val)
    time.sleep(1.0)



