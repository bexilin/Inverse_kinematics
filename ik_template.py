#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy

#### YOUR IMPORTS GO HERE ####

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


def tuckarms(env, robot):
    with env:
        jointnames = ['torso_lift_joint', 'l_shoulder_lift_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint',
                      'r_shoulder_lift_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0.24, 1.29023451, -2.32099996, -0.69800004, 1.27843491, -2.32100002, -0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


# set active DOF values from a numpy matrix
def SetActiveDOFValuesNPMatrix(robot, qmat):
    qo = [q.item(i) for i in range(0, qmat.shape[1])]
    robot.SetActiveDOFValues(qo)


# returns the end effector transform in the world frame
def GetEETransform(robot, activedofvalues=None):
    if activedofvalues != None:
        robot.SetActiveDOFValues(activedofvalues);
    manip = robot.GetActiveManipulator()
    return manip.GetEndEffectorTransform()


# returns the joint axis in the world frame
def GetJointAxis(robot, jointname):
    return robot.GetJoint(jointname).GetAxis(0)


# returns the joint position in the world frame
def GetJointPosition(robot, jointname):
    return robot.GetJoint(jointname).GetAnchor()


def GetTranslationJacobian(robot, jointnames):
    J = numpy.zeros((3, robot.GetActiveDOF()))
    ### YOUR CODE HERE ###
    activedofvalues = robot.GetActiveDOFValues()
    EETtransform = GetEETransform(robot, activedofvalues)
    s = EETtransform[0:3, 3]
    v = numpy.array([GetJointAxis(robot, i) for i in jointnames])
    p = numpy.array([GetJointPosition(robot, i) for i in jointnames])
    jacob = numpy.array([numpy.cross(v[i], s-p[i]) for i in range(len(v))])

    J = numpy.transpose(jacob)
    ### YOUR CODE HERE ###

    return J


def GetJpinv(J):
    ### YOUR CODE HERE ###
    JT = numpy.transpose(J)
    Jpinv = numpy.matmul(JT, numpy.linalg.inv(numpy.matmul(J, JT)+0.0001 * numpy.identity(len(numpy.matmul(J, JT)))))

    ### YOUR CODE HERE ###
    return Jpinv


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env, 'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    # load a scene from environment XML file
    env.Load('pr2only.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms and raise torso
    tuckarms(env, robot);

    # set start config
    robot.SetActiveManipulator('leftarm')
    jointnames = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint',
                  'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

    targets = [[-0.15070158, 0.47726995, 1.56714123],
               [-0.36535318, 0.11249, 1.08326675],
               [-0.56491217, 0.011443, 1.2922572],
               [-1.07012697, 0.81909669, 0.47344636],
               [-1.11050811, 0.97000718, 1.31087581]]
    doflimits = robot.GetActiveDOFLimits()  # make sure q doesn't go past these limits
    q = numpy.zeros((1, robot.GetActiveDOF()))  # start at this configuration
    with env:
        start = time.clock()
        handles = []  # graphics handles for plotting
        SetActiveDOFValuesNPMatrix(robot, q)

        ### YOUR CODE HERE ###
    for i in range(5):
        print '\n\n\ntest target ', i, ':', '\n'
        target = targets[i]  ###pick your target here
        # draw the target point in blue
        handles = []
        handles.append(env.plot3(points=array(target), pointsize=15.0, colors=array((0, 0, 1))))

        q = [0., 0., 0., 0., 0., 0., 0.]
        robot.SetActiveDOFValues(q)
        robot.GetController().SetDesired(robot.GetDOFValues())
        waitrobot(robot)
        raw_input("place robot configuration back to start configuration q = [0,0,0,0,0,0,0], press enter to continue\n")

        while 1:
            for i in range(len(q)):
                if q[i] < doflimits[0][i]:
                    q[i] = doflimits[0][i]
                elif q[i] > doflimits[1][i]:
                    q[i] = doflimits[1][i]

            EETtransform = GetEETransform(robot, q)
            x = [EETtransform[0][3], EETtransform[1][3], EETtransform[2][3]]
            delta_x = array([target[i] - x[i] for i in range(len(target))])

            if numpy.linalg.norm(delta_x) < 0.01:
                break

            robot.SetActiveDOFValues(q)
            J = GetTranslationJacobian(robot, jointnames)
            Jinv = GetJpinv(J)
            delta_q = numpy.matmul(Jinv, delta_x)

            if numpy.linalg.norm(delta_q) > 0.5:
                d = delta_q / numpy.linalg.norm(delta_q)
                delta_q = 0.5 * d

            q = q + delta_q

        print 'joint limits are:\n', 'minimum: ', doflimits[0], '\n', 'maximum: ', doflimits[1], '\n'
        print 'target configuration is:\n', q

        ### YOUR CODE HERE ###

        robot.GetController().SetDesired(robot.GetDOFValues())
        waitrobot(robot)
        raw_input("\nPress enter and test next one")

    raw_input("Press enter to exit...")
    env.Destroy()
