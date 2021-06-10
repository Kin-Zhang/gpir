import carla
import time
import rospy
import argparse
import pygame as pg

from numpy import random
from pygame.locals import *
from sensor_msgs.msg import Joy


class AgentGenerator():
    def __init__(self, args):
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.tm = self.client.get_trafficmanager(args.tm_port)
        self.tm.set_global_distance_to_leading_vehicle(1.0)

        blueprints = self.world.get_blueprint_library().filter(args.filterv)
        blueprints = [x for x in blueprints if int(
            x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]
        self.blueprints = blueprints

        self.SpawnActor = carla.command.SpawnActor
        self.SetAutopilot = carla.command.SetAutopilot
        self.SetVehicleLightState = carla.command.SetVehicleLightState
        self.FutureActor = carla.command.FutureActor
        self.agent_list = []
        self.minimum_distance = 8
        self.spawn_offset = {"front": 8, "left": 0, "right": 0, "back": 8}

        self.autopilot_enabled = False

        while True:
            actors = self.world.get_actors().filter('vehicle.*')
            if actors != None:
                self.ego = actors[0]
                print("find ego vehicle, id: {}".format(self.ego.id))
                break
            print("wait for ego vehicle")
            time.sleep(1)

    def random_blueprint(self):
        random.seed(int(time.time()))
        blueprint = random.choice(self.blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(
                blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(
                blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')
        return blueprint

    def get_transform(self, waypoint):
        tf = waypoint.transform
        tf.location.z += 1
        return tf

    def spawn_agent(self, transform):
        batch = []
        batch.append(self.SpawnActor(self.random_blueprint(), transform))
        for response in self.client.apply_batch_sync(batch, False):
            if response.error:
                print("{}".format(response.error))
                return False
            else:
                self.agent_list.append(response.actor_id)
        return True

    def spawn_agent_at(self, location="front"):
        ego_waypoint = self.map.get_waypoint(self.ego.get_transform().location)
        spawn_point = None

        if location == "front":
            spawn_point = ego_waypoint.next(self.spawn_offset[location])
            if spawn_point:
                spawn_point = spawn_point[-1]
        elif location == "back":
            spawn_point = ego_waypoint.previous(self.spawn_offset[location])
            if spawn_point:
                spawn_point = spawn_point[-1]
        elif location == "left":
            spawn_point = ego_waypoint.get_left_lane()
            if spawn_point and self.spawn_offset[location] > 0:
                spawn_point = spawn_point.next(self.spawn_offset[location])
                if spawn_point:
                    spawn_point = spawn_point[-1]

        if not spawn_point:
            print("can't find spawn location at ego's {}".format(location))
            return False

        if self.spawn_agent(self.get_transform(spawn_point)):
            self.spawn_offset[location] += self.minimum_distance

        return True

    def trigger_autopilot(self):
        port = self.tm.get_port()
        print("{} autopilot".format(
            "Disable" if self.autopilot_enabled else "Enable"))
        for agent in self.agent_list:
            self.world.get_actor(agent).set_autopilot(
                ~self.autopilot_enabled, port)
        self.autopilot_enabled = not self.autopilot_enabled

    def destory(self):
        print('\ndestroying %d agents' % len(self.agent_list))
        self.client.apply_batch([carla.command.DestroyActor(x)
                                 for x in self.agent_list])
        time.sleep(0.5)


class KeyboardHandler():
    def __init__(self, args):
        self.cmd_count = 0
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.agent_generator = AgentGenerator(args)

    def init_joy(self):
        joy = Joy()
        joy.header.frame_id = "map"
        joy.header.stamp = rospy.Time.now()
        for i in range(8):
            joy.axes.append(0)
        for i in range(11):
            joy.buttons.append(0)
        return joy

    def publish_joy(self, joy):
        self.cmd_count += 1
        self.joy_pub.publish(joy)

    def update(self):
        for event in pg.event.get():
            if event.type == pg.KEYDOWN:
                joy = self.init_joy()
                if event.key == pg.K_w:
                    print("{}: Increse reference speed.".format(self.cmd_count))
                    joy.buttons[3] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_s:
                    print("{}: Decrease reference speed.".format(self.cmd_count))
                    joy.buttons[0] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_a:
                    print("{}: Suggest left lane change.".format(self.cmd_count))
                    joy.buttons[2] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_d:
                    print("{}: Suggest right lane change.".format(self.cmd_count))
                    joy.buttons[1] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_e:
                    print("{}: Add virtual obstacles to lane".format(self.cmd_count))
                    joy.buttons[5] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_i:
                    print("Add agent in the front of ego")
                    self.agent_generator.spawn_agent_at("front")
                elif event.key == pg.K_j:
                    print("Add agent in the left of ego")
                    self.agent_generator.spawn_agent_at("left")
                elif event.key == pg.K_b:
                    print("Trigger agents' autopilot")
                    self.agent_generator.trigger_autopilot()

    def destroy(self):
        self.agent_generator.destory()


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    rospy.init_node("key2joy")
    rate = rospy.Rate(10)

    pg.init()
    pg.display.set_mode((300, 300))
    keyboard_handler = KeyboardHandler(args)

    try:
        while not rospy.is_shutdown():
            keyboard_handler.update()
            # pg.display.update()
            rate.sleep()
    finally:
        keyboard_handler.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        pg.quit()
