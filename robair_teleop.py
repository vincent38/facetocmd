import rospy as rp
import curses
import os
from std_msgs.msg import UInt8

def core(win):
    #win.nodelay(True)
    k = ""
    win.clear()
    win.addstr("Sent command : ")
    p = rp.Publisher('verin_speed', UInt8, queue_size=10)
    rp.init_node('Verin', anonymous=True)
    rate = rp.Rate(10)
    while not rp.is_shutdown():
        # Send correct speed based on pressed key
        try:
            k = win.getkey()
            win.clear()
            win.addstr("Sent command : ")
            if k == 'u':
                p.publish(127)
                win.addstr("UP")
            elif k == 'd':
                p.publish(129)
                win.addstr("DOWN")
            elif k == 'q':
                win.addstr("BYE")
                break
            else:
                p.publish(128)
                win.addstr("STOP")
            k = ''
            rate.sleep()
        except Exception:
            win.addstr("Goodbye...")
            exit()

if __name__ == '__main__':
    try:
        curses.wrapper(core)
    except rp.ROSInterruptException:
        print("Oh no")
        pass
