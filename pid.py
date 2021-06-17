from collections import deque
import math
import numpy as np
import carla


class VehiclePIDController():
    """
    VehiclePIDController là sự kết hợp của hai bộ điều khiển PID
    (lateral (ngang) và  longitudinal (dọc)) để thực hiện điều khiển cấp độ thấp một chiếc xe từ phía client side
    """


    def __init__(self, vehicle, knowledge, args_lateral, args_longitudinal, offset=0, max_throttle=0.75, max_brake=0.3,
                 max_steering=0.8):
        """
       

        :param vehicle: Ám chỉ xe điều khiển cung cấp logic lên
        :param args_lateral: Kiểu dữ liệu từ điển các đối số để đặt bộ điều khiển PID ngang
        sử dụng các ngữ nghĩa sau: 
            K_P: Hệ số tỉ lệ, thông số điều chỉnh
            K_D: Độ lợi tích phân
            K_I: Độ lợi vi phân
        :param args_longitudinal: Kiểu dữ liệu từ điển các đối số để đặt bộ điều khiển PID dọc
        sử dụng các ngữ nghĩa sau: 
            K_P: Hệ số tỉ lệ, thông số điều chỉnh
            K_D: Độ lợi tích phân
            K_I: Độ lợi vi phân
        :param offset: Nếu khác 0, xe sẽ bị dịch chuyển ra khỏi vạch giữa.
        Giá trị dương có nghĩa là bù bên phải trong khi giá trị âm có nghĩa là giá trị bên trái. Con số đủ cao
        khiến xe chạy qua các làn đường khác có thể bị hỏng bộ điều khiển. 
        """

        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering
        self.knowledge = knowledge
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self.past_steering = self._vehicle.get_control().steer
        self._lon_controller = PIDLongitudinalController(self._vehicle, self.knowledge, **args_longitudinal)
        self._lat_controller = PIDLateralController(self._vehicle, offset, **args_lateral)

    def run_step(self, target_speed, waypoint):
        """
        Thực hiện một bước điều khiển gọi cả ngang và dọc
        Bộ điều khiển PID để đạt được điểm tham chiếu mục tiêu
        ở một vận tộc đặc định nhất định. 

            :param target_speed: Tốc độ xe mong muốn
            :param waypoint: Vị trí mục tiêu được mã hóa dưới dạng điểm tham chiếu
            :return: Khoảng cách (tính bằng mét) đến điểm tham chiếu
        """

        acceleration = self._lon_controller.run_step(target_speed)
        current_steering = self._lat_controller.run_step(waypoint)
        control = carla.VehicleControl()
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throt)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Điều chỉnh steer: không thể thay đổi đột ngột, không thể chỉ đạo quá nhiều.

        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering

        return control


class PIDLongitudinalController():
    """
    PIDLongitudinalController thực hiện kiểm soát dọc bằng PID.
    """


    def __init__(self, vehicle, knowledge, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        Constructor method.
            :param vehicle: Ám chỉ xe điều khiển cung cấp logic lên 
            :param K_P: Hệ số tỉ lệ, thông số điều chỉnh
            :param K_D: Độ lợi tích phân
            :param K_I: Độ lợi vi phân
            :param dt:thời gian chênh lệch tính bằng giây 
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._dt = dt
        self._error_buffer = deque(maxlen=10)
        self.knowledge = knowledge
    def run_step(self, target_speed, debug=False):
        """
        Thực hiện một bước điều khiển theo chiều dọc để đạt được tốc độ mục tiêu nhất định.

            :param target_speed: Vận tốc đặt định (Km/h)
            :param debug: boolean để gỡ lỗi
            :return: điều khiển ga
        """
        current_speed = self.knowledge.retrieve_data('speed')

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Ước tính ga / phanh của xe dựa trên phương trình PID 

            :param target_speed:  Vận tốc đặt định   (Km/h)
            :param current_speed: vận tốc thực tại của xe  (Km/h)
            :return: điều khiển  ga / phanh 
        """

        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

class PIDLateralController():
    """
    PIDLateralController thực hiện kiểm soát biên ngang bằng PID.
    """

    def __init__(self, vehicle, offset=0, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: ám chỉ xe điều khiển cung cấp logic lên 
            :param offset: khoảng cách đến đường tâm. Nếu có xảy ra sự cố nếu giá trị
                 đủ lớn để làm cho xe lấn sang các làn đường khác. 
            :param K_P: Hệ số tỉ lệ, thông số điều chỉnh
            :param K_D: Độ lợi tích phân
            :param K_I: Độ lợi vi phân
            :param dt:thời gian chênh lệch tính bằng giây 
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._dt = dt
        self._offset = offset
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoint):
        """
        Thực hiện một bước điều khiển bên để hướng xe về một điểm nhất định. 

            :param waypoint: điểm tham chiếu mục tiêu
            :return: kiểm soát góc lái trong phạm vi  [-1, 1] Khi:
            -1 lái tối đa sang trái
            +1 lái tối đa sang phải
        """
        return self._pid_control(waypoint, self._vehicle.get_transform())

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Ước tính góc lái của xe dựa trên phương trình PID

            :param waypoint: điểm tham chiếu mục tiêu
            :param vehicle_transform: biến đổi hiện tại của vehicle
            :return: kiểm soát góc lái trong phạm vi [-1, 1]
        """
        # Nhận vị trí của xe và vectơ chuyển tiếp
        ego_loc = vehicle_transform.location
        v_vec = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_vec.x, v_vec.y, 0.0])

        # Lấy vector vehicle-target_wp
        if self._offset != 0:
            # Dịch chuyển wp sang một bên
            w_tran = waypoint.transform
            r_vec = w_tran.get_right_vector()
            w_loc = w_tran.location + carla.Location(x=self._offset*r_vec.x,
                                                         y=self._offset*r_vec.y)
        else:
            w_loc = waypoint.transform.location

        w_vec = np.array([w_loc.x - ego_loc.x,
                          w_loc.y - ego_loc.y,
                          0.0])

        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)
