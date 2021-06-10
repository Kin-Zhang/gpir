import time
import math
import argparse
import carla
import random
import logging

from transforms3d.euler import euler2quat


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
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    args = argparser.parse_args()

    logging.basicConfig(
        format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    debug = world.debug
    nav_map = world.get_map()
    spawn_points = world.get_map().get_spawn_points()

    ego_vehicle = None
    find_ego_vehicle = False
    while not find_ego_vehicle:
        actors = world.get_actors().filter('vehicle.*')
        if len(actors) > 0:
            find_ego_vehicle = True
            ego_vehicle = actors[0]
            print("find ego vehicle, id: {}".format(ego_vehicle.id))
            break
        print("wait for ego vehicle")
        time.sleep(1)

    print(ego_vehicle.get_transform())
    rotation = ego_vehicle.get_transform().rotation
    quat = euler2quat(math.radians(rotation.roll), math.radians(
        rotation.pitch), math.radians(rotation.yaw))
    waypoint = nav_map.get_waypoint(ego_vehicle.get_transform().location)
    left_waypoint = waypoint.get_left_lane()
    print(left_waypoint.lane_type)
    print("left waypoint: {}".format(left_waypoint.transform))
    left_waypoint = left_waypoint.next(20)[-1]

    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    SetVehicleLightState = carla.command.SetVehicleLightState
    FutureActor = carla.command.FutureActor

    blueprints = world.get_blueprint_library().filter(args.filterv)

    if True:
        blueprints = [x for x in blueprints if int(
            x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]

    batch = []
    if (left_waypoint):
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(
                blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(
                blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')
        tf = left_waypoint.transform
        tf.location.z = tf.location.z + 1
        # tf.rotation = ego_vehicle.get_transform().rotation
        debug.draw_point(tf.location, size=0.2, life_time=0)
        # tf.pitch = 0
        print("spawn agent at {}".format(tf))
        batch.append(SpawnActor(blueprint, tf))

    vehicles_list = []
    for response in client.apply_batch_sync(batch, False):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)

    while True:
        # world.wait_for_tick()
        time.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone')
