import socket
from threading import Thread
from typing import Optional

import rclpy
from rclpy.node import Node

from tm_get_status import translate_json_to_list


class Talker(Node):
    def __init__(self, ip: str) -> None:
        super().__init__("talker")
        self.ip = ip
        self.prot = 5891
        self.socket_connect = None
        self.is_connected = False
        self.background_listen: Optional[Thread] = None

    def socket_connect(self) -> None:
        self.socket_connect = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_connect.connect((self.ip, self.prot))
        self.is_connected = True
        self.background_listen = Thread(target=self.listener_callback)
        self.background_listen.start()

    def listener_callback(self) -> None:
        remaining_str = ""
        while self.is_connected:
            data_byte = self.socket_connect.recv(1024)
            print("get data from server which is")
            data = str(data_byte, encoding="utf-8")
            print(data)
            data = data + remaining_str
            print("New data is")
            remaining_str, new_str = translate_json_to_list.TmJsonToDiction.split_package(data)
            if new_str is not None:
                print(new_str[0])
                json_string = translate_json_to_list.TmJsonToDiction.tm_string_to_json(new_str[0])
                print(json_string)
                json_dict = translate_json_to_list.TmJsonToDiction.json_to_dict(json_string)
                print(json_dict)

            # time.sleep(0.1)
            # msg = PythonTalker()
            # msg.data = "Hello World!"
            # msg.talk_times = self.counter
            # self.counter += 1
            # self.get_logger().info('Publishing something !')
            # self.publisher.publish(msg)


def main() -> None:
    rclpy.init()

    ip = "192.168.132.242"

    node = Talker(ip)

    node.socket_connect()

    node.listener_callback()

    rclpy.spin(node)

    node.destroy_node()
    node.is_connected = False
    rclpy.shutdown()


if __name__ == "__main__":
    main()
