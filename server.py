import socket
import time
import cv2
from qr_recognition import read_qr

#Setup address and message size
bufferSize=1024
ServerPort=2222
ServerIP='172.16.10.195'   #WiFi Adrress Radek

#Setup socket and bind it to address
RPIsocket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
RPIsocket.bind((ServerIP,ServerPort))
print('Server is up and listening')

# Camera setup
cap = cv2.VideoCapture(1)

# Setting up resolution
cap.set(3, 640)  # szerokość
cap.set(4, 480)  # wysokość

load_present = False
load_destination = None

#Communication
while True:
    message,address=RPIsocket.recvfrom(bufferSize)
    message=message.decode('utf-8')

    print(message)
    print('Client Address: ',address[0])

    #read frame from camera
    ret, frame = cap.read()

    load_present, load_destination, x, y, w, h = read_qr(frame)

    if(load_present):
        # Rysowanie prostokąta wokół kodu QR
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, load_destination, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    if(message=='Gimmie_orders'):
        match load_destination:
            case 'smartfony':
                msg = '11'
            case 'laptopy':
                msg = '0'
            case _:
                msg = 'wait'

        #msg=str(counter)
        msg=msg.encode('utf-8')

        #Back message
        RPIsocket.sendto(msg,address)

    # Wyświetlanie obrazu z kamery
    cv2.imshow("QR Code Detection", frame)

    # Zakończenie pętli po naciśnięciu klawisza 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()