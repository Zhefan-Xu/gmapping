import numpy as np
import math


def wrapToPi(theta):
    while (theta > np.pi):
        theta -= np.pi * 2

    while (theta < -np.pi):
        theta += np.pi * 2
    return theta


def motion_model(pose, odom):
    rot1 = wrapToPi(np.arctan2(odom[1], odom[0]) - pose[2])
    trans = np.sqrt(odom[0]**2 + odom[1]**2)
    rot2 = wrapToPi(odom[2] - rot1)

    x = pose[0] + trans * np.cos(pose[2] + rot1)
    y = pose[1] + trans * np.sin(pose[2] + rot1)
    theta = wrapToPi(pose[2] + rot1 + rot2)
    return np.array([x, y, theta])


def prob_normal_distribution(a, b):
    p=np.exp(-a**2/(2*b))/np.sqrt(2*np.pi*b)
    return p


def sample(var):
    return np.random.normal(loc=0.0, scale=var)


def motion_model_odometry(xt,ut,xt_1,a):
    # xt_1: x at time t-1: (x1, y1, theta1)
    # xt: x at time t: (x2, y2, theta2)
    # ut: (uxt_1, uxt), with uxt_1=(ux1,uy1,utheta1), uxt_2=(ux2,uy2,utheta2)
    # a: alphas: (a1, a2, a3, a4)

    x1,y1,theta1=xt_1
    x2,y2,theta2=xt

    uxt_1,uxt=ut
    ux1,uy1,utheta1=uxt_1
    ux2,uy2,utheta2=uxt

    a1,a2,a3,a4=a

    rot1=wrapToPi(math.atan2((uy2-uy1),(ux2-ux1))-utheta1)
    trans=np.sqrt((ux1-ux2)**2 + (uy1-uy2)**2)
    rot2=wrapToPi(utheta2-utheta1-rot1)

    rot1_h=wrapToPi(math.atan2((y2-y1),(x2-x1))-theta1)
    trans_h=np.sqrt((x1-x2)**2 + (y1-y2)**2)
    rot2_h=wrapToPi(theta2-theta1-rot1_h)

    p1=prob_normal_distribution(rot1-rot1_h,a1*np.square(rot1_h)+a2*np.square(trans_h))
    p2=prob_normal_distribution(trans-trans_h,a3*np.square(trans_h)+a4*(np.square(rot1_h)+np.square(rot2_h)))
    p3=prob_normal_distribution(rot2-rot2_h,a1*np.square(rot2_h)+a2*np.square(trans_h))

    return np.log(p1) + np.log(p2) + np.log(p3)


def sample_motion_model(pose, odom, a=[1e-4, 0.01, 0.01, 1e-4]):
    # xt_1: x at time t-1: (x1, y1, theta1)
    # ut: (uxt_1, uxt), with uxt_1=(ux1,uy1,utheta1), uxt_2=(ux2,uy2,utheta2)
    # a: alphas: (a1, a2, a3, a4)

    [x1,y1,theta1]=pose

    [odom_x, odom_y, odom_theta] = odom

    # uxt_1,uxt=ut
    # ux1,uy1,utheta1=uxt_1
    # ux2,uy2,utheta2=uxt

    a1,a2,a3,a4=a

    rot1=wrapToPi(math.atan2((odom_y),(odom_x))-odom_theta)
    trans=np.sqrt((odom_x)**2 + (odom_y)**2)
    rot2=wrapToPi(odom_theta-rot1)

    rot1_hat = rot1- sample(a1 * np.square(rot1) + a2 * np.square(trans))
    trans_hat = trans-sample(a3 * np.square(trans) + a4 * (np.square(rot1) + np.square(rot2)))
    rot2_hat = rot2-sample(a1 * np.square(rot2) + a2 * np.square(trans))

    x2=x1+trans_hat*np.cos(theta1+rot1_hat)
    y2=y1+trans_hat*np.sin(theta1+rot1_hat)
    theta2=wrapToPi(theta1+rot1_hat+rot2_hat)

    xt=np.array([x2, y2, theta2])
    return xt



