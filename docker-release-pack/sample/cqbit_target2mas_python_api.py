from ctypes import *
import time

class cqbit_target2mas_tcp_python_api:

    library = cdll.LoadLibrary("libcqbit_target2mas.so")
    ret_socket = 0
    def tcp_connected(self, ip, port):
        self.library.tcp_connected.argtypes = [c_char_p, c_int]
        self.library.tcp_connected.restype  = c_void_p
        
        tmp_ip = create_string_buffer(ip.encode('utf-8'))
        tmp_port = c_int(port)
        self.ret_socket = self.library.tcp_connected(tmp_ip, tmp_port)
    
    def trans_json2bin_mas_send(self, json_data):
        self.library.trans_json2bin_mas_send.argtypes = [c_char_p, c_int]
        self.library.trans_json2bin_mas_send.restype  = c_int

        tmp_json_data = create_string_buffer(json_data.encode('utf-8'))
        
        socket_val = c_int(self.ret_socket)
        flag = self.library.trans_json2bin_mas_send(tmp_json_data, socket_val)
        return flag

        
        

json_data = "{\"device_id\":\"1000\",\"time_stamp\":1628679021,\"object_info\":[{\"target_id\":1,\"sign\":1,\"vx\":0.1,\"vy\":0.3,\"lng\":114.53758,\"lat\":39.578908,\"class\":\"car\",\"course_angle\":56.7},{\"target_id\":2,\"sign\":1,\"vx\":0.1,\"vy\":0.3,\"lng\":114.53758,\"lat\":39.578908,\"class\":\"car\",\"course_angle\":56.7}]}"


a = cqbit_target2mas_tcp_python_api()

tmp_ip = "ined.racodf.com"
tmp_port = 63098
a.tcp_connected(tmp_ip, tmp_port)
while True:
    flag = a.trans_json2bin_mas_send(json_data)
    print(flag)
    if flag < 0:
        a.tcp_connected(tmp_ip, tmp_port)
   
    time.sleep(1.0)

