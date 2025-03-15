import cv2
import numpy as np

# lower_purple = np.array([125, 50, 50])
# upper_purple = np.array([150, 255, 255])


def get_rects(image: cv2.Mat, lower_: np.array, upper_: np.array) -> list:
    hsv_ = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_ = cv2.inRange(hsv_, lower_, upper_)

    # Улучшение маски с помощью морфологических операций
    kernel_ = np.ones((5, 5), np.uint8)
    mask_ = cv2.morphologyEx(mask_, cv2.MORPH_OPEN, kernel_)
    mask_ = cv2.morphologyEx(mask_, cv2.MORPH_CLOSE, kernel_)

    # Поиск контуров
    contours_, _ = cv2.findContours(mask_, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rectangles_ = []
    for cnt_ in contours_:
        # Аппроксимация контура полигоном
        epsilon_ = 0.02 * cv2.arcLength(cnt_, True)
        approx_ = cv2.approxPolyDP(cnt_, epsilon_, True)

        # Отбор только четырехугольников
        if len(approx_) == 4:
            # Преобразование координат и добавление в результат
            points_ = approx_.reshape(-1, 2).tolist()
            rectangles_.append(points_)

            # Отрисовка контура (опционально)
            cv2.drawContours(image, [approx_], -1, (0, 255, 0), 3)

    return rectangles_


# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     get_rects(frame)

#     cv2.imshow('Detection', frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()