import socket

def call_orders(server_ip):
	bufferSize=1024
	serverAddress=(server_ip,2222)
	UDPClient=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

	cmd='Gimmie_orders'
	bytesToSend=cmd.encode('utf-8')

	UDPClient.sendto(bytesToSend,serverAddress)

	data = None

	while True:
		data,address=UDPClient.recvfrom(bufferSize)
		if(data):
			data=data.decode('utf-8')

			print('Data from server: ', data)
			break
		#print('Server IP: ',   address[0])
		#print('Server port: ', address[1])

def main():
	while True:
		call_orders('172.16.10.198')

if __name__ == '__main__':
    main()