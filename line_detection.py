import cv2
import numpy as np

def line_follower(image):
    direction = 0

    # Konwertowanie kolorów z BGR do HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Definicja zakresu koloru czarnej linii na białym tle w przestrzeni HSV
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 255, 50])

    # Tworzenie maski koloru czarnej linii
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Wykrywanie konturów
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Wybieranie największego konturu
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M['m00'] != 0:
            # Obliczanie centroidy konturu
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # print("CX: " + str(cx) + " CY: " + str(cy))

            if cx >= 480:
                # print("Turn Left")
                direction = 1
            elif cx < 480 and cx > 160:
                # print("On Track")
                direction = 2
            elif cx <= 160:
                # print("Turn Right")
                direction = 3

            # Rysowanie centroidy
            cv2.circle(image, (cx, cy), 5, (255, 255, 255), -1)

            # # Rysowanie konturu
            cv2.drawContours(image, [c], -1, (0, 255, 0), 1)

    return image, direction

def main():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        image, direction = line_follower(frame)

        if(direction == 0):
            print("No path")
        elif(direction == 1):
            print("Go left")
        elif(direction == 2):
            print("Go straight")
        elif(direction == 3):
            print("Go right")
        else:
            print("Error")

        cv2.imshow("Frame", image)

        # Wyjście po naciśnięciu klawisza 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()