#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
import threading
from cafe.msg import SiparisMsg
import time
from typing import List
import dockerinos


class Siparis():
    hazirlanmayaBasladi = 0
    siparis = ""
    masa = ""
    hazir = False

siparisler = []


#####   ros msg

def msgSubOrder(msg):
    o = Siparis()
    o.siparis = msg.siparis
    o.masa = msg.masa
    siparisler.append(o)

global sac

def actionclientlauncher():
    global sac
    sac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    while(not sac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("move_base node una baglaniliyor...")
    rospy.loginfo("move_base node una baglanildi")

def goto(pos):
    mbg = MoveBaseGoal()
    mbg.target_pose.header.frame_id="map"
    mbg.target_pose.header.stamp=rospy.Time.now()
    mbg.target_pose.pose.position.x = pos.x
    mbg.target_pose.pose.position.y = pos.y
    mbg.target_pose.pose.position.z = 0.188790
    mbg.target_pose.pose.orientation.x = 0
    mbg.target_pose.pose.orientation.y = 0
    mbg.target_pose.pose.orientation.z = 0
    mbg.target_pose.pose.orientation.w = 1
    global sac
    sac.send_goal(mbg)
    sac.wait_for_result(rospy.Duration.from_sec(3))
    return sac.get_state()


#####   movement

# goals = []

class Hedef:
    x = 0
    y = 0
    z = 0
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

baslangic = Hedef(1.13, 0.05, 0)
t1 = Hedef(2.76, -1.21, -90)
t2 = Hedef(4.64, -1.38, -90)
t3 = Hedef(6.45, -1.32, -90)
t4 = Hedef(6.67, 0.86, 90)
t5 = Hedef(4.75, 1.09, 90)
t6 = Hedef(2.84, 1.15, 90)
dock = Hedef(7.68, -0.27, 0)

masalar = [t1, t2, t3, t4, t5, t6]

def robotMantik():
    while True:
        for siparis in siparisler:
            if siparis.hazir:
                state = 0
                while not state == 3:
                    state = goto(baslangic)
                siparisler.remove(siparis)
                print("[Robot] Mutfaktan masa "+str(siparis.masa)+"'in "+siparis.siparis+" siparisi alindi.")
                goto(masalar[siparis.masa-1])
                state = 0
                while not state == 3:
                    state = goto(masalar[siparis.masa-1])
                print("[Robot] "+siparis+" siparisi masa "+str(siparis.masa)+"'ya getirildi. Mutfaga geri donuluyor.")
                state = 0
                while not state == 3:
                    state = goto(baslangic)
        # for goal in goals:
        #     if goal.active:
        #         result = goto(goal)
        #         print(result)
        #         if result == 3:
        #             goals.remove(goal)

def mutfakMantik():
    while True:
        for siparis in siparisler:
            if not siparis.hazir:
                print("[Mutfak] Masa "+str(siparis.masa)+"'nin "+siparis.siparis+" siparisi hazirlanacak.")
                while time.time()-siparis.hazirlanmayaBasladi < 5:
                    pass
                print("[Mutfak] Masa "+str(siparis.masa)+"'nin "+siparis.siparis+" siparisi hazir.")
                siparis.hazir = True
            

# def newGoal(x, y):
#     g = Hedef()
#     g.x = x
#     g.y = y
#     g.active = True
#     goals.append(g)

if __name__ == "__main__":
    dockerinos.launch()
    rospy.init_node("cafe_node")
    actionclientlauncher()
    rospy.Subscriber("/karpuz_cafe", SiparisMsg, msgSubOrder)
    loop = threading.Thread(target=robotMantik)
    loop.start()
    loop2 = threading.Thread(target=mutfakMantik)
    loop2.start()
    print("aaaaaaaaaaaaaaaaa")
    state = 0
    while not state == 3:
        state = goto(baslangic)
        print(state)
    state = 0
    while not state == 3:
        state = goto(baslangic)
        print(state)
    # newGoal(2, 2)
    # o = Siparis()
    # o.siparis = "msg.siparis"
    # o.masa = 1
    # siparisler.append(o)
    rospy.spin()
