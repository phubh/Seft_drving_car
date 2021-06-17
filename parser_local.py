#!/usr/bin/env python

import glob
import os
import sys
import math
import weakref

try:
    sys.path.append(glob.glob('./carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Can't find CARLA")
    exit(0)

import carla


# Monitor chịu trách nhiệm đọc dữ liệu từ các cảm biến và thông báo cho kho quản lí kiến thức
class Monitor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        weak_self = weakref.ref(self)

        # Khởi tạo tất cả các biến bộ nhớ đã xác định
        self.update(0)

        world = self.vehicle.get_world()
        self.knowledge.update_data('world', world)
        # Thiết lập bộ dò làn đường
        lane_sensor = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.lane_detector = world.spawn_actor(lane_sensor, carla.Transform(), attach_to=self.vehicle)
        self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))

        # Thiết lập cảm biến va chạm
        collision_sensor = world.get_blueprint_library().find('sensor.other.collision')
        self.collision_detector = world.spawn_actor(collision_sensor, carla.Transform(), attach_to=self.vehicle)
        self.collision_detector.listen(lambda event: self._on_collision(weak_self, event))

       # Thiết lập máy ảnh để cho phép hiển thị chế độ xem phối cảnh người thứ nhất và có thể thực hiện nhận dạng đối tượng
        camera_rgb_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_rgb_blueprint.set_attribute('image_size_x', '1280')
        camera_rgb_blueprint.set_attribute('image_size_y', '720')
        camera_rgb_blueprint.set_attribute('fov', '120')
        camera_rgb_blueprint.set_attribute('sensor_tick', '0.04')
        camera_transform = carla.Transform(carla.Location(x=-8.5, z=10.5), carla.Rotation(-40,0,0))
        self.camera_rgb = world.spawn_actor(camera_rgb_blueprint, camera_transform, attach_to=self.vehicle)
        self.camera_rgb.listen(lambda image: self.knowledge.update_data('rgb_camera', image))

    # Hàm được gọi trong khoảng thời gian để cập nhật trạng thái ai
    def update(self, time_elapsed):
        # Cập nhật vị trí của xe vào kho quản lí thông tin 
        location = self.vehicle.get_transform().location
        destination = self.knowledge.get_current_destination()
        distance = location.distance(destination)

        self.knowledge.update_data('location', self.vehicle.get_transform().location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
        self.knowledge.update_data('velocity', self.vehicle.get_velocity()) # Đây là vectơ sẽ được xử lý trong Analyser
        self.knowledge.update_data('distance', distance)
        self.knowledge.update_data('speed_limit', self.vehicle.get_speed_limit())
        # self.knowledge.update_data('speed_limit', 60)


        # at_lights is true, vì phương tiện đang ở đèn giao thông màu đỏ hoặc vàng
        self.knowledge.update_data('at_lights', self.vehicle.is_at_traffic_light() and
                                   str(self.vehicle.get_traffic_light_state()) == 'Red' or
                                   str(self.vehicle.get_traffic_light_state()) == 'Yellow')

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        self.knowledge.update_data('lane_invasion', True)
        if not self:
            return

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return


# Analyser chịu trách nhiệm phân tích cú pháp tất cả dữ liệu mà kho thông tin đã nhận được từ Monitor và biến nó thành thứ có thể sử dụng được
# TODO: Trong bước cập nhật phân tích cú pháp dữ liệu bên trong kho thông tin thành thông tin mà người lập kế hoạch có thể sử dụng để lập kế hoạch tuyến đường
class Analyser(object):
    def __init__(self, knowledge):
        self.knowledge = knowledge

    # Hàm được gọi trong khoảng thời gian để cập nhật trạng thái ai
    def update(self, time_elapsed):
        # Nhận khoảng cách 2D đến điểm đến hiện tại
        distance_x, distance_y = self.calculate_XY_distances()

        # Tính hướng 2D đến điểm đến hiện tại và sự khác biệt so với hướng hiện tại của chúng ta
        
        heading = math.atan2(distance_y, distance_x)
        heading_diff = self.calculate_heading_diff()

        self.knowledge.update_data('distance_x', distance_x)
        self.knowledge.update_data('distance_y', distance_y)
        self.knowledge.update_data('heading', heading)
        self.knowledge.update_data('heading_diff', heading_diff)
        self.knowledge.update_data('speed', self.get_speed())
        return

    def get_speed(self):
        velocity = self.knowledge.retrieve_data('velocity')
        return math.sqrt(velocity.x ** 2.0 + velocity.y ** 2.0 + velocity.z ** 2.0) * 3.6

    def calculate_XY_distances(self):
        location = self.knowledge.retrieve_data('location')
        destination = self.knowledge.get_current_destination()
        distance_x = destination.x - location.x
        distance_y = destination.y - location.y
        return distance_x, distance_y

    def calculate_heading_diff(self):
        current_location = self.knowledge.get_location()
        current_angle = math.radians(self.knowledge.get_rotation())
        next_waypoint = self.knowledge.get_current_destination()

        target_vector = carla.Vector3D(x=next_waypoint.x - current_location.x, y=next_waypoint.y - current_location.y)
        target_angle = math.atan2(target_vector.y, target_vector.x)

        diff = target_angle - current_angle
        if diff > math.pi: diff = diff - (2 * math.pi)
        if diff < - math.pi: diff = diff + (2 * math.pi)

        return diff
