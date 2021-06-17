#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
import math

from knowledge import Status
try:
    sys.path.append(glob.glob('./carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Can't find CARLA")
    exit(0)

import carla
from pid import VehiclePIDController
from route_planner import RoutePlanner
from route_planner_dao import RoutePlannerDAO
# Executor chịu trách nhiệm di chuyển của xe 
# Trong quá trình triển khai này, nó chỉ cần phù hợp với tay lái và tốc độ để chúng tôi đến các điểm tham chiếu được cung cấp
# BONUS TODO: Thực hiện các giới hạn tốc độ khác nhau để người lập kế hoạch cũng sẽ cung cấp tốc độ mục tiêu tốc độ ngoài hướng
class Executor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        self.world = self.knowledge.retrieve_data('world')
        self.map = self.world.get_map()
        self.target_pos = knowledge.get_location()
        self.dt = 1.0/20.0
        self.max_brake = 0.9
        self.max_throt = 0.6
        self.max_steer = 0.8
        self.offset = 0
    # Khởi tạo PID 
    def createPID(self):
        args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.2,
            'K_I': 0.07,
            'dt': self.dt}
        args_longitudinal_dict = {
            'K_P': 1.55,
            'K_D': 0.1,
            'K_I': 0.1,
            'dt': self.dt}
        self.knowledge.update_data('vehicle_controller',VehiclePIDController(self.vehicle,
                                                        self.knowledge,
                                                        args_lateral=args_lateral_dict,
                                                        args_longitudinal=args_longitudinal_dict,
                                                        offset=self.offset,
                                                        max_throttle=self.max_throt,
                                                        max_brake=self.max_brake,
                                                        max_steering=self.max_steer))
    # Cập nhật điều hành vào một số khoảng thời gian để điều khiển xe theo hướng mong muốn
    def update(self, time_elapsed):
        status = self.knowledge.get_status()
        # TODO: khả năng xử lý
        if status == Status.DRIVING:
            dest = self.knowledge.get_current_destination()
            self.update_control(dest, [1], time_elapsed)
        if status == Status.ARRIVED :
            self.vehicle.apply_control(carla.VehicleControl(throttle = 0, steer = 0, brake = 1.0))
    def update_control(self, destination, additional_vars, delta_time):
        # speed = self.knowledge.retrieve_data('speed')
        target_speed = self.knowledge.retrieve_data('target_speed')
        vehicle_controller = self.knowledge.retrieve_data('vehicle_controller')
        # # Tạo biến điều khiển 
        control = self.vehicle.get_control()
        control = vehicle_controller.run_step(target_speed,self.map.get_waypoint(destination))
        # # TODO: Kết hợp tính ngưỡng và tính góc này trong công cụ lập kế hoạch và tinh chỉnh nó.
        # heading_diff = self.knowledge.retrieve_data('heading_diff')
        # control.steer = heading_diff / 3

        # # Tính toán sự khác biệt giữa tốc độ hiện tại và tốc độ mục tiêu, và điều chỉnh ga cho phù hợp.
        # # Vì các xe khác nhau phản ứng khác nhau với cùng một lượng ga, điều này cần được mở rộng với
        # # vòng lặp phản hồi thích ứng.
        # # TODO: Phản hồi thích ứng
        # speed_diff = target_speed - speed
        # if target_speed == 0:
        #     control.throttle = 0.0
        #     control.brake = 1.0
        # elif speed_diff == 0:
        #     control.throttle = 0
        #     control.brake = 0
        # elif speed_diff < 0:
        #     control.throttle = 0
        #     control.brake = 0.3
        # elif speed_diff > 0:
        #     control.throttle = min((speed_diff / target_speed) + 0.2 , 1.0) - abs(control.steer)
        #     control.brake = 0.0

        self.vehicle.apply_control(control)


# Planner chịu trách nhiệm tạo ra một kế hoạch để di chuyển
# Trong trường hợp này, nó tạo ra một danh sách các điểm tham chiếu để theo dõi để phương tiện đến đích
class Planner(object):
    def __init__(self, knowledge,vehicle):
        self.knowledge = knowledge
        self.path = deque([])
        self.world = self.knowledge.retrieve_data('world')
        self.map = self.world.get_map()
        self.vehicle = vehicle
    # Tạo bản đồ các điểm tham chiếu để đi theo đến đích và lưu nó
    def make_plan(self, source, destination):
        self.path = self.build_path(source, destination)
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

    # Hàm được gọi trong khoảng thời gian để cập nhật trạng thái ai
    def update(self, time_elapsed):
        at_lights = self.knowledge.retrieve_data('at_lights')

        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

        if at_lights or len(self.path) == 0:
            self.knowledge.update_data('target_speed', 0)
        else:
            self.knowledge.update_data('target_speed', self.knowledge.retrieve_data('speed_limit'))

    # Cập nhật trạng thái nội bộ để đảm bảo rằng có các điểm tham chiếu để theo dõi và xem xe đã đến đích chưa
    def update_plan(self):
        if len(self.path) == 0:
            return

        if self.knowledge.arrived_at(self.path[0]):
            self.path.popleft()

        if len(self.path) == 0:
            self.knowledge.update_status(Status.ARRIVED)
        else:
            self.knowledge.update_status(Status.DRIVING)

    # Lấy điểm đến hiện tại
    def get_current_destination(self):
        status = self.knowledge.get_status()
        # Xe đang chạy tiếp, thì điểm đến hiện tại là điểm tham chiếu tiếp theo
        if status == Status.DRIVING:
            return self.path[0]
        if status == Status.ARRIVED:
            return self.knowledge.get_location()
        if status == Status.HEALING:
            return self.knowledge.get_location()
        if status == Status.CRASHED:
            return self.knowledge.get_location()
        return self.knowledge.get_location()

    # Tạo đường dẫn các điểm tham chiếu từ vị trí hiện tại đến điểm đến hiện tại
    def build_path(self, source, destination):

        def find_next(current, destination):
            right_distance = math.inf
            left_distance = math.inf
            found = None
            min_distance = math.inf
            nexts = current.next(8)
            

            for point in nexts:
                next_nexts = point.next(25)
                for next_point in next_nexts:
                    next_dist = destination.distance(next_point.transform.location)
                    if next_dist < min_distance:
                        min_distance = next_dist
                        found = point
                if found.lane_change == carla.LaneChange.Right or point.lane_change == carla.LaneChange.Both:
                    if found.get_right_lane() != None:
                        right_point = self.get_waypoint(found.get_right_lane().transform.location)
                        right_distance = right_point.transform.location.distance(destination_wp)
                if found.lane_change == carla.LaneChange.Left or found.lane_change == carla.LaneChange.Both:
                    if found.get_right_lane() != None:
                        left_point = self.get_waypoint(found.get_left_lane().transform.location)
                        left_distance = left_point.transform.location.distance(destination_wp)

                if right_distance < min_distance and right_distance < left_distance:
                    found = right_point
                    min_distance = right_distance
                elif left_distance < min_distance and left_distance < right_distance:
                    found = left_point
                    min_distance = left_distance

            return found if found is not None else self.get_waypoint(destination)

        destination_wp = self.get_waypoint(destination).transform.location
        source_wp = self.get_waypoint(source.location).transform.location

        self.path = deque([])
        next_point = self.get_waypoint(source_wp)
        self.path.append(source_wp)

        # while destination_wp.distance(next_point.transform.location) > 5:
            
        #     next_point = find_next(next_point, destination_wp)
        #     print(next_point.transform.location)
        #     self.path.append(next_point.transform.location)
        # self.path.append(destination_wp)
        dao = RoutePlannerDAO(self.vehicle.get_world().get_map(), 3.0)
        
        grp = RoutePlanner(dao)
        grp.setup()
        self._grp = grp
        # Obtain route plan
        self.pathas = self._grp.trace_route(source_wp,destination_wp)
        for point in self.pathas:
            target = point[0].transform.location
            # print(target)
            self.path.append(target)
        self.draw_debug_path(self.path)
        
        return self.path

    def draw_debug_path(self, path):
        debug_markers_lifetime = math.inf
        self.world.debug.draw_string(path[0], "START", life_time=debug_markers_lifetime)
        for point in path:
            color = carla.Color(r=0,b=255,g=0) if self.get_waypoint(point).is_intersection else carla.Color(r=255,b=0,g=0)
            self.world.debug.draw_point(point + carla.Location(y=0.5), 0.15, life_time=debug_markers_lifetime, color=color)
        self.world.debug.draw_string(path[len(path) - 1], "END", life_time=debug_markers_lifetime)

    def get_waypoint(self, location):
        return self.map.get_waypoint(location)