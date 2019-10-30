#!/usr/bin/env python
from __future__ import division
from decimal import *
import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface

rospy.init_node("eurobot_task_handler", anonymous=False)
rospy.loginfo("initializing the robot...")
robot = AdvancedRobotInterface()
robot.initialize()

# Different Quadrilaterals
# ---------------------------------------------------------------------------------------------------------------------


#
def drive_square(dista):

    robot.move_linear(dista, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)
    robot.move_linear(dista, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)
    robot.move_linear(dista, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)
    robot.move_linear(dista, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)


# ZADANIE 2 - TASK TWO - ODKMENTOWAC W RAZIE ZADANIA
def task2(dista):

    robot.move_linear(1, should_avoid_obstacles=True, obstacle_backup_distance=0.3, clearing_distance=0.32, move_timeout=8)
    robot.move_angular(90)
    robot.move_angular(30)
    robot.move_linear(dista, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(120)
    robot.move_linear(dista, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(120)
    robot.move_linear(dista, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)


def drive_rectangle(sidea, sideb):
    robot.move_linear(sidea, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)
    robot.move_linear(sideb, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)
    robot.move_linear(sidea, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)
    robot.move_linear(sideb, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)


def drive_rhombus(sidea, anglesame):
    robot.move_angular(anglesame, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_linear(sidea)
    robot.move_angular(180-anglesame)
    robot.move_linear(sidea, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(anglesame)
    robot.move_linear(sidea, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(180-anglesame)
    robot.move_linear(sidea, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)


def drive_parallelogram(sidea, sideb, anglesamesmlr):
    robot.move_angular(90-anglesamesmlr)
    robot.move_linear(sidea, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(180-anglesamesmlr)
    robot.move_angular(sideb, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(anglesamesmlr)
    robot.move_linear(sidea, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(180-anglesamesmlr)
    robot.move_linear(sideb, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)

   
# Different Types of Triangles
# ---------------------------------------------------------------------------------------------------------------------

def drive_triangle_equilateral(sidesame):
    robot.move_angular(30)
    robot.move_linear(sidesame, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(120)
    robot.move_linear(sidesame, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(120)
    robot.move_linear(sidesame, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(120)




def drive_traingle_isosceles(lenghtbase, lenghtsame, anglesame, angleb):
    robot.move_angular(anglesame)
    robot.move_linear(lenghtsame, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(2 * angleb)
    robot.move_linear(lenghtsame, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(anglesame)
    robot.move_linear(lenghtbase, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(anglesame)


def drive_triangle_tangled(frst_trarg, second_trsttrarg):
    robot.move_linear(frst_trarg, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)
    # # remove timeout: move_timeout=6
    # robot.move_linear(second_trsttrarg, move_timeout=6, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
    #                   clearing_distance=0.32)
    robot.move_linear(second_trsttrarg, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_to(0, 0, 0)


# Different Types of Trapeziums
# -----------------------------------------------------------------------------------------------------------------


def drive_trapezium_angled(sidea, sideb, sidec, sided, angletoprg, anglebtrg):
    robot.move_linear(sidea, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(90)
    robot, move_linear(sideb, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.movbe_angular(90 + angletoprg)
    robot.move_linear(sided, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)
    robot.move_angular(anglebtrg)
    robot.move_linear(sidec, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)


def drive_trapezium_isosceles(lenghtsamesides, lenght_lngr_btmside, lenght_shrt_topside, anglesame):
    robot.move_angular(90 - anglesame)
    robot.move_linear(lenghtsamesides, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular((360 - (anglesame * 2) / 2))
    robot.move_linear(lenght_shrt_topside, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular((360 - (anglesame * 2) / 2))
    robot.move_linear(lenghtsamesides, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(anglesame)
    robot.move.linear(lenght_lngr_btmside, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)

# Different Types of Polygons
# -----------------------------------------------------------------------------------------------------------------


def drive_pentagram(lenghtsamesidesp):
    robot.move_angular(-72)
    robot.move_linear(lenghtsamesidesp, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(72)
    robot.move_linear(lenghtsamesidesp, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(72)
    robot.move_linear(lenghtsamesidesp, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(72)
    robot.move_linear(lenghtsamesidesp, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(72)
    robot.move_linear(lenghtsamesidesp, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)


def drive_hexagon(lenghtsamesidesh):
    robot.move_angular(-60)
    robot.move_linear(lenghtsamesidesh, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(60)
    robot.move_linear(lenghtsamesidesh, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(60)
    robot.move_linear(lenghtsamesidesh, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(60)
    robot.move_linear(lenghtsamesidesh, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(60)
    robot.move_linear(lenghtsamesidesh, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(60)
    robot.move_linear(lenghtsamesidesh, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)


def drive_heptagon(lenghtsamesideshe):
    robot.move_angular(-51.43)
    robot.move_linear(lenghtsamesideshe, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(51.43)
    robot.move_linear(lenghtsamesideshe, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(51.43)
    robot.move_linear(lenghtsamesideshe, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(51.43)
    robot.move_linear(lenghtsamesideshe, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(51.43)
    robot.move_linear(lenghtsamesideshe, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(51.43)
    robot.move_linear(lenghtsamesideshe, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(51.43)
    robot.move_linear(lenghtsamesideshe, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)


def drive_octagon(lenghtsamesidesoc):
    robot.move_angular(-45)
    robot.move_linear(lenghtsamesidesoc, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(45)
    robot.move_linear(lenghtsamesidesoc, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(45)
    robot.move_linear(lenghtsamesidesoc, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(45)
    robot.move_linear(lenghtsamesidesoc, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(45)
    robot.move_linear(lenghtsamesidesoc, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(45)
    robot.move_linear(lenghtsamesidesoc, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(45)
    robot.move_linear(lenghtsamesidesoc, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)
    robot.move_angular(45)
    robot.move_linear(lenghtsamesidesoc, should_avoid_obstacles=True, obstacle_backup_distance=0.2,
                      clearing_distance=0.32)


while True:
    print('Team Brainstrom WorldSkills 2019 Robot Drawing Control Centre Developed by Szymon Sebastian Malecki')
    print()
    print('Quadraterials: Square = s or S , Rectangle = r or R, Rhombus: rb or RB, Parallelogram: prlg or PRLG')
    print()
    print('Triangles: Equilateral Triangle: tq or TQ, Isosceles Triangle: ts or TS, Right-Angled Triangle: tra or TRA')
    print()
    print('Trapeziums: Isosceles Trapezium: tssc or TSSC, Right-Angled Trapezium: tsra or TSRA')
    print()
    print('Polygons: Pentagram: pentgm or PENTGM, Hexagon: hexgn or HEXGN, Heptagon: heptgn or HEPTGN, Octagon: octagn')
    inpt = str(raw_input('Please choose(s, r, rb, prlg tq, ts, tra, tsra, tssc, pentgm, hexgn, heptgn, octagn ): '))

    if inpt == 's' or inpt == 'S':
        print("You've chosen option %s from Quadraterials(Square)" % inpt)
    elif inpt == 't' or inpt == 'T':
        print("Task 2")

    elif inpt == "r" or inpt == "R":
        print("You've chosen option %s from Quadraterials(Rectangle)" % inpt)
    elif inpt == 'rb' or inpt == 'RB':
        print("You've chosen option %s from Quadraterials(Rhombus)" % inpt)
    elif inpt == 'prlg' or inpt == 'PRLG':
        print("You've chosen option %s from Quadraterials(Parallelogram)" % inpt)
    elif inpt == 'tq' or inpt == 'TQ':
        print("You've chosen option %s from Triangles(Equaliteral Triangle)" % inpt)
    elif inpt == 'ts' or inpt == 'TS':
        print("You've chosen option %s from Triangles(Isosceles Triangle)" % inpt)
    elif inpt == 'tra' or inpt == 'TRA':
        print("You've chosen option %s from Triangles(Right-Angled Triangle)" % inpt)
    elif inpt == 'trsa' or inpt == 'TRSA':
        print("You've chosen option %s from Trapeziums(Right-Angled Trapezium)" % inpt)
    elif inpt == 'tssc' or inpt == 'TSSC':
        print("You've chosen option %s from Trapeziums(Isosceles Trapezium)" % inpt)
    elif inpt == 'pentgm' or inpt == 'PENTGM':
        print("You've chosen option %s from Polygons(Pentagram)")
    elif inpt == 'hexgn' or inpt == 'HEXGN':
        print("You've chosen option %s from Polygons(Hexagons)")
    elif inpt == 'heptgn' or inpt == 'HEPTGN':
        print("You've chosen option %s from Polygons(Heptagon)")
    elif inpt == 'octagn' or inpt == 'OCTAGN':
        print("You've chosen option %s from Polygons(OCTAGON)")

    if inpt == 's' or inpt == 'S':
        dista = float(input("Please give a side length value: "))
        print("Your side lenght value is: %.2f" % dista)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_square(dista)

    elif inpt == "r" or inpt == "R":
        sidea = float(input("Please give a length of sideA of rectangle: "))
        print("%.2f value was saved under sideA" % sidea)
        sideb = float(input("Please give a length of sideB of rectangle: "))
        print("%.2f value was saved under sideB" % sideb)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_rectangle(sidea, sideb)

    elif inpt == "t" or inpt == "T":
        dista = float(input("Please give a length of sideA of rectangle: "))
        print("%.2f value was saved under sideA" % dista)


        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        task2(dista)

    elif inpt == 'rb' or inpt == 'RB':
        sidea = float(input("Please give a length of sideA of your rhombus: "))
        print("%.2f value was saved under sideA" % sidea)

        anglesame = int(input("Please give a value of the equal angles: "))
        print("%i angle value was saved for equal base angles" % anglesame)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_rhombus(sidea, anglesame)

    elif inpt == 'prlg' or inpt == 'PRLG':
        sidea = float(input("Please give a length of sideA of parallelogram: "))
        print("%.2f value was saved under sideA" % sidea)
        sideb = float(input("Please give a length of sideB of parallelogram: "))
        print("%.2f value was saved under sideB" % sideb)

        anglesamesmlr = int(input("Please give a value of the equal angles: "))
        print("%i angle value was saved for equal base angles" % anglesamesmlr)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_parallelogram(sidea, sideb, anglesamesmlr)

    elif inpt == 'tq' or inpt == 'TQ':
        sidesame = float(input("Please give a length of the side of equilateral triangle: "))
        print("%.2f value was saved as the side lenght" % sidesame)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_triangle_equilateral(sidesame)

    elif inpt == 'ts' or inpt == 'TS':
        lenghtbase = float(input("Please give a length of the base: "))
        print("%.2f value was saved as the lenght of the base" % lenghtbase)
        lenghtsame = float(input("Please give a length of same sides of isosceles triangle: "))
        print("%.2f value was saved as the lenght of two opposite isosceles sides: " % lenghtsame)

        anglesame = int(input("Please give a value of the equal base angles: "))
        print("%i angle value was saved for equal base angles" % anglesame)
        angleb = int(input("Please give value of the top angle: "))
        print("%i angle value was saved for short angle tangle" % angleb)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_traingle_isosceles(lenghtbase, lenghtsame, anglesame, angleb)

    elif inpt == 'tra' or inpt == 'TRA':
        frst_trarg = float(input("Please give a length of the first side of your right-angled triangle: "))
        print("Lenght of the first side is: %.2f" % frst_trarg)
        second_trsttrarg = float(input("Please give a length of the second side of your right-angled triangle: "))
        print("Your side lenght is: %.2f" % second_trsttrarg)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_triangle_tangled(frst_trarg, second_trsttrarg)

    elif inpt == 'trsa' or inpt == 'TRSA':
        sidea = float(input("Please give a length of sideA of right-angled trapezium: "))
        print("%.2f value was saved under SideA of right-angled trapezium" % sidea)
        sideb = float(input("Please give a length of sideB of right-angled trapezium: "))
        print("%.2f value was saved under SideBof right-angled trapezium" % sideb)
        sidec = float(input("Please give a length of sideC of right-angled trapezium: "))
        print("%.2f value was saved under SideB of right-angled trapezium" % sidec)
        sided = float(input("Please give a length of sideD of right-angled trapezium: "))
        print("%.2f value was saved under SideBof right-angled trapezium:" % sided)

        angletoprg = int(input("Please give value of the top-right-angle: "))
        print("%i angle value was saved for top-right-angle" % angletoprg)
        anglebtrg = int(input("Please give value of bottom-right-angle: "))
        print("%i angle value was saved for bottom-right-angle(shorter)" % anglebtrg)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_trapezium_angled(sidea, sideb, sidec, sided, angletoprg, anglebtrg)

    elif inpt == 'tssc' or inpt == 'TSSC':
        lenghtsamesides = float(input("Please give a length of same opposite sides of isosceles trapezium: "))
        print("%.2f value was saved for the lenght of opposite isosceles sides of isosceles trapezium: " % lenghtsamesides)
        lenght_lngr_btmside = float(input("Please give a length of longer bottom side of isosceles trapezium: "))
        print("%.2f value was saved for longer bottom side of isosceles trapezium:  " % lenght_lngr_btmside)
        lenght_shrt_topside = float(input("Please give a length of shorter bottom side of isosceles trapezium: "))
        print("%.2f value was saved for shorter top side of isosceles trapezium" % lenght_shrt_topside)

        anglesame = int(input("Please give value of the same base opposite angles of isosceles trapezium: "))
        print("%i value was saved for value of same exact angles on opposite sides of isosceles trapezium " % anglesame)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_trapezium_isosceles(lenghtsamesides, lenght_lngr_btmside, lenght_shrt_topside, anglesame)

    elif inpt == 'pentgm' or inpt == 'PENTGM':
        lenghtsamesidesp = float(input("Please give a length of same opposite sides of pentagram: "))
        print("%.2f value was saved for the lenght of all sides of pentagram" % lenghtsamesidesp)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_pentagram(lenghtsamesidesp)

    elif inpt == 'hexgn' or inpt == 'HEXGN':
        lenghtsamesidesh = float(input("Please give a length of same opposite sides of hexagon: "))
        print("%.2f value was saved for the lenght of all sides of hexagon" % lenghtsamesidesh)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_hexagon(lenghtsamesidesh)

    elif inpt == 'heptgn' or inpt == 'HEPTGN':
        lenghtsamesideshe = float(input("Please give a length of same opposite sides of heptagon: "))
        print("%.2f value was saved for the lenght of all sides of heptagon" % lenghtsamesideshe)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_heptagon(lenghtsamesideshe)

    elif inpt == 'octagn' or inpt == 'OCTAGN':
        lenghtsamesidesoc = float(input("Please give a length of same opposite sides of octagon: "))
        print("%.2f value was saved for the lenght of all sides of octagon" % lenghtsamesidesoc)

        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_octagon(lenghtsamesidesoc)

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()
