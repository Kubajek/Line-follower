import cv2
import pyzbar.pyzbar as pyzbar

def read_qr(frame):
    qr_detected = False
    x, y, w, h = 0, 0, 0, 0
    qr_data = ''

    # Zmiana obrazu na skalę szarości
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Wyszukiwanie kodów QR
    qr_codes = pyzbar.decode(gray)

    if len(qr_codes) != 0:
        qr_detected = True
        for qr_code in qr_codes:
            (x, y, w, h) = qr_code.rect

        # Odczytanie danych z kodu QR
        qr_data = qr_code.data.decode("utf-8")
    
    return qr_detected, qr_data, x, y, w, h

def main():
# Inicjalizacja kamery
    cap = cv2.VideoCapture(1)

# Ustawienie rozdzielczości kamery
    cap.set(3, 640)  # szerokość
    cap.set(4, 480)  # wysokość

    while True:
    # Odczytanie obrazu z kamery
        ret, frame = cap.read()

        qr_detected, qr_data, x, y, w, h = read_qr(frame)

        if(qr_detected):
            # Rysowanie prostokąta wokół kodu QR
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print('QR data: ', qr_data)
        else:
            print('No QR detected')


        # Wyświetlanie obrazu z kamery
        cv2.imshow("QR Code Detection", frame)

        # Zakończenie pętli po naciśnięciu klawisza 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Zatrzymanie kamery i zamknięcie okna
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()