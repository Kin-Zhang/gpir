import pygame as pg
from pygame.locals import *
import rospy
from sensor_msgs.msg import Joy


class KeyboardHandler():
    def __init__(self):
        self.cmd_count = 0
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)

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
                    self.publish_joy(joy);


def main():
    rospy.init_node("key2joy")
    rate = rospy.Rate(10)

    pg.init()
    pg.display.set_mode((300, 300))
    keyboard_handler = KeyboardHandler()
    while not rospy.is_shutdown():
        keyboard_handler.update()
        # pg.display.update()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        pg.quit()
