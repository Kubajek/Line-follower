import cv2
import numpy as np

def detect_shape(image, shape):
    # Konwertowanie obrazu z BGR do HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Definicja zawężonego zakresu koloru czerwonego w przestrzeni HSV
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Definicja zawężonego zakresu koloru żółtego w przestrzeni HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])

    # Tworzenie maski koloru czerwonego
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = mask1 + mask2

    # Tworzenie maski koloru żółtego
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Wykrywanie konturów
    if shape == "triangle":
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    elif shape == "square":
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    shape_detected = False
    for contour in contours:
        # Aproksymacja konturu
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Sprawdzenie, czy kontur ma odpowiednią liczbę wierzchołków i czy jest wystarczająco duży
        if shape == "triangle" and len(approx) == 3 and cv2.contourArea(approx) > 1000:
            shape_detected = True
            cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)

        elif shape == "square" and len(approx) == 4 and cv2.contourArea(approx) > 1000:
            shape_detected = True
            cv2.drawContours(image, [approx], 0, (0, 255, 255), 2)

    return image, shape_detected

def main():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        _, triangle_detected = detect_shape(frame, "triangle")
        result_frame, square_detected = detect_shape(frame, "square")

        cv2.imshow("Shape detection", result_frame)

        if triangle_detected:
            print("Detected red triangle!")

        if square_detected:
            print("Detected yellow square!")

        # Zakończenie pętli po naciśnięciu klawisza 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
