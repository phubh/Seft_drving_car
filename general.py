#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('./carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Can't find CARLA")
    exit(0)

import carla
import knowledge as data
import control 
import parser_local as ps
import time


# Mã điều khiển 
class Autopilot(object):
  def __init__(self, vehicle):
    self.vehicle = vehicle
    self.knowledge = data.Knowledge()
    self.knowledge.set_status_changed_callback(self.status_updated)
    self.analyser = ps.Analyser(self.knowledge)
    self.monitor = ps.Monitor(self.knowledge, self.vehicle)
    self.planner = control.Planner(self.knowledge,self.vehicle)
    self.executor = control.Executor(self.knowledge, self.vehicle)
    self.prev_time = int(round(time.time() * 1000))
    self.route_finished = lambda *_, **__: None
    self.crashed = lambda *_, **__: None

    self.knowledge.update_data('graph', self.vehicle.get_world().get_map().get_topology())
    self.executor.createPID()
  def status_updated(self, new_status):
    if new_status == data.Status.ARRIVED:
      self.route_finished(self)
    if new_status == data.Status.CRASHED:
      self.crashed(self)

  def set_route_finished_callback(self, callback):
    self.route_finished = callback

  def set_crash_callback(self, callback):
    self.crashed = callback

  def get_vehicle(self):
    return self.vehicle

  # Cập nhật tất cả các mô-đun và trả về trạng thái hiện tại
  def update(self):
    ctime = int(round(time.time() * 1000))
    delta_time = ctime - self.prev_time
    self.prev_time = ctime
    
    self.monitor.update(delta_time)
    self.analyser.update(delta_time)
    self.planner.update(delta_time)
    self.executor.update(delta_time)

    return self.knowledge.get_status()

  # Điểm tương tác chính với tính năng lái tự động - đặt điểm đến
  def set_destination(self, destination):
    self.planner.make_plan(self.vehicle.get_transform(), destination)
