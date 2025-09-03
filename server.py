#!/usr/bin/env python3
import rclpy                         
from rclpy.node import Node         
import socket                        
import serial                        
import threading                     

class UdpUartBridge(Node):
    def __init__(self):
        super().__init__('udp_uart_bridge')

        self.udp_ip = self.get_ip()
        self.udp_port = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("UART 연결 성공")
            
        except Exception as e:
            self.get_logger().error(f"UART 연결 실패: {e}")
            self.ser = None
            
        self.udp_thread = threading.Thread(target=self.receive_udp_loop)
        self.udp_thread.daemon = True  
        self.udp_thread.start()
        self.get_logger().info(f"UDP 수신 중: {self.udp_ip}:{self.udp_port}")

    def get_ip(self):

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip

    def receive_udp_loop(self):
 
        
        while True:
            # 최대 1024 바이트 크기의 패킷을 수신
            data, addr = self.sock.recvfrom(1024)
            # 받은 데이터를 UTF-8 형식으로 디코딩하여 문자열로 변환
            msg = data.decode('utf-8')
            self.get_logger().info(f"UDP 수신: {msg} from {addr}")
            self.sock.sendto("Connected successfully!".encode('utf-8'), addr)

            if self.ser:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    self.ser.write(data)
                    self.get_logger().info(f"UART 송신: {msg.strip()}")
                except Exception as e:
                    self.get_logger().error(f"UART 오류: {e}")
            else:
                self.get_logger().info(f"[디버그] UART 없음 - 메시지 출력만: {msg.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = UdpUartBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
