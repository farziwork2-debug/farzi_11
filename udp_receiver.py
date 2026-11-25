import socket
import struct 
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0",5005))

while True:

	data, addr = sock.recvfrom(4096)
	
	seq,ts, *q = struct.unpack("<Id12d", data)
	
	print(
		f"seq={seq} "
		f"FL={[round(x,3) for x in q[:3]]}"
	)
