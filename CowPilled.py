import socket

UDP_IP = "0.0.0.0" 
UDP_PORT = 4210    

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
sock.bind((UDP_IP, UDP_PORT))

print(f"Data on port {UDP_PORT}")

while True:
    try:
        data, addr = sock.recvfrom(1024) 
        print(f"Data: {data.decode('utf-8')}")
    except KeyboardInterrupt:
        break