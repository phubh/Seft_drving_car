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
import random
import numpy as np
import pygame
import general as ai
import math

IMG_HEIGHT = 720
IMG_WIDTH = 1280
pygame.init()
display = pygame.display.set_mode((IMG_WIDTH,IMG_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.flip()
font = pygame.font.SysFont('Arial', 20)

def render(image,text,clock):
    
    def blit_text(surface, text, pos, font, color=pygame.Color('white')):
        words = [word.split(' ') for word in text.splitlines()]  # 2D array where each row is a list of words.
        space = font.size(' ')[0]  # The width of a space.
        max_width, max_height = surface.get_size()
        x, y = pos
        for line in words:
            for word in line:
                word_surface = font.render(word, 0, color)
                word_width, word_height = word_surface.get_size()
                if x + word_width >= max_width:
                    x = pos[0]  # Reset the x.
                    y += word_height  # Start on new row.
                surface.blit(word_surface, (x, y))
                x += word_width + space
            x = pos[0]  # Reset the x.
            y += word_height  # Start on new row  
    if image:
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        display.blit(surface, (0, 0))
        blit_text(display, text, (20, 120), font)
    pygame.display.update()
def show_info(controller):  
    vehicle = controller.get_vehicle()
    v = vehicle.get_velocity()  
    c = vehicle.get_control()
    t = vehicle.get_transform()
    info_text = "Speed:   % 15.0f km/h" % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
    info_text += "\nLocation:% 20s" % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y))
    info_text += "\nThrottle: %0.5f" % (c.throttle)
    info_text += "\nSteer: %0.5f" % (c.steer)
    info_text += "\nBrake: %0.5f" % (c.brake)
    return info_text
        
def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(12.0)

    try:
        world = client.load_world("Town05")
        blueprints = world.get_blueprint_library().filter('vehicle.citroen.c3')

        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]

        def draw_forward_vector():
            start_transform = world.get_map().get_waypoint(actor_list[0].get_location()).transform
            start_location = start_transform.location
            f_vect = start_transform.get_forward_vector()
            raiser = carla.Location(y=0.5)
            line_end = start_location + carla.Location(x=f_vect.x * 5, y=f_vect.y * 5)
            world.debug.draw_line(start_location + raiser, line_end + raiser, thickness=0.3, life_time=10.0)

        def label_spawn_points():
            wps = world.get_map().get_spawn_points()
            for i in range(len(wps)):
                world.debug.draw_string(wps[i].location, str(i))

        # Một hàm gọi lại sẽ được gọi khi chế độ lái tự động đến đích
        def route_finished(autopilot):
            print("Vehicle arrived at destination")

        # Hàm sinh ra đối tượng xe
        def try_spawn_random_vehicle_at(transform, destination):
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            vehicle = world.try_spawn_actor(blueprint, transform)
            if vehicle is not None:
                actor_list.append(vehicle)
                autopilot = ai.Autopilot(vehicle)
                autopilot.set_destination(destination.location)
                # Đăng ký gọi lại để biết khi nào xe đã đến đích
                autopilot.set_route_finished_callback(route_finished)
                print('spawned %r at %s' % (vehicle.type_id, transform.location))
                return autopilot
            return False
        spawn_points = list(world.get_map().get_spawn_points())
        print('found %d spawn points.' % len(spawn_points))

        start = spawn_points[124]
        end = spawn_points[225]
        ex2 = [carla.Transform(location=carla.Location(0, 0, 1.8431), rotation=carla.Rotation(yaw=80)), carla.Transform(location=carla.Location(x=-80, y=-230, z=1.8431))]
        start, end = ex2
        end = world.get_map().get_waypoint(end.location).transform
        controller = try_spawn_random_vehicle_at(start, end)
        clock = pygame.time.Clock()
        
        while True:
            status = controller.update()
            render(controller.knowledge.retrieve_data('rgb_camera'),show_info(controller),clock)
            pygame.display.flip()

            from pygame.locals import K_ESCAPE
            from pygame.locals import K_l
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if event.key == K_ESCAPE:
                        return True
    finally:
        print('\ndestroying %d actors' % len(actor_list))
        client.apply_batch([carla.command.DestroyActor(x.id) for x in actor_list])
        pygame.quit()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
