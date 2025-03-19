import time


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(None, None)):
        self.Kp = Kp  # Пропорциональный коэффициент
        self.Ki = Ki  # Интегральный коэффициент
        self.Kd = Kd  # Дифференциальный коэффициент
        self.setpoint = setpoint  # Заданное значение (уставка)
        self.output_limits = output_limits  # Ограничения выхода

        self._integral = 0  # Интегральная сумма
        self._prev_error = 0  # Предыдущая ошибка
        self._last_time = None  # Время последнего вызова

    def update(self, current_value):
        # Вычисляем время с последнего обновления
        current_time = time.monotonic()
        dt = current_time - self._last_time if self._last_time is not None else 0
        self._last_time = current_time

        # Вычисляем ошибку регулирования
        error = self.setpoint - current_value

        # Пропорциональная составляющая
        P = self.Kp * error

        # Интегральная составляющая
        self._integral += error * dt
        I = self.Ki * self._integral

        # Дифференциальная составляющая
        D = 0
        if dt > 0:
            D = self.Kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Суммируем составляющие
        output = P + I + D

        # Применяем ограничения выхода
        if self.output_limits is not None:
            min_limit, max_limit = self.output_limits
            if min_limit is not None and output < min_limit:
                output = min_limit
                self._integral -= error * dt  # Антивиндовая защита
            elif max_limit is not None and output > max_limit:
                output = max_limit
                self._integral -= error * dt  # Антивиндовая защита

        return output

    def reset(self):
        """Сброс интегральной суммы и предыдущей ошибки"""
        self._integral = 0
        self._prev_error = 0
        self._last_time = None

# # Пример использования
# if __name__ == "__main__":
#     # Инициализация PID-регулятора
#     pid = PID(
#         Kp=0.5,
#         Ki=0.1,
#         Kd=0.05,
#         setpoint=50,  # Целевая температура
#         output_limits=(0, 100)  # Ограничения мощности нагревателя
#     )

#     current_temp = 20  # Начальная температура
#     last_time = time.monotonic()

#     for _ in range(100):
#         # Вычисляем управляющее воздействие
#         control = pid.update(current_temp)

#         # Имитация изменения температуры (простая модель системы)
#         dt = time.monotonic() - last_time
#         current_temp += control * 0.1 * dt  # Нагрев
#         current_temp -= 0.2 * dt  # Естественное охлаждение

#         print(f"Управление: {control:.2f}%, Температура: {current_temp:.2f}°C")
#         last_time = time.monotonic()
#         time.sleep(0.1)