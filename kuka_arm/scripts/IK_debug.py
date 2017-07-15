#IK_debug file provided by Udacity
from sympy import *
from time import time
from mpmath import radians
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from IK_server import rot_x, rot_y, rot_z
import tf


'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generate test cases can be added to the test_cases dictionary
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}



def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    ########################################################################################
    ## Insert IK code here starting at: Define DH parameter symbols


    # Define DH param symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            
    # Joint angle symbols
        # Defined above with DH param symbols
      
    # Modified DH params
    s = {alpha0:     0, a0:      0, d1:  0.75,
         alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
         alpha2:     0, a2:   1.25, d3:     0, 
         alpha3: -pi/2, a3: -0.054, d4:   1.5,
         alpha4:  pi/2, a4:      0, d5:     0,
         alpha5: -pi/2, a5:      0, d6:     0,
         alpha6:     0, a6:      0, d7: 0.303, q7:       0}
            
    # Define Modified DH Transformation matrix
        # Total transform defined after individual transforms

    # Create individual transformation matrices
    T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
                   [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                   [                  0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
                   [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                   [                  0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
                   [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   [                  0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
                   [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                   [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                   [                  0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
                   [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                   [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                   [                  0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)
            
    T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
                   [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                   [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                   [                  0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)

    T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
                   [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                   [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                   [                  0,                   0,            0,               1]])
    T6_G = T6_G.subs(s)

    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_G = simplify(T0_6 * T6_G)

    # Correct orientation of gripper link in URDF vs. DH convention
    # 180 degree rotation about the z-axis
    R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                  [sin(pi),  cos(pi), 0, 0],
                  [      0,        0, 1, 0],
                  [      0,        0, 0, 1]])
    # 90 degree rotation about the y-axis
    R_y = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
                  [          0, 1,          0, 0],
                  [-sin(-pi/2), 0, cos(-pi/2), 0],
                  [          0, 0,          0, 1]])
    R_corr = simplify(R_z * R_y)
	
    # Total transform with gripper orientation corrected
    T_total = simplify(T0_G * R_corr) 


    for x in xrange(0, len(req.poses)):
        # IK code starts here
        joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z
        
        print("px = %f, py = %f, pz = %f", px, py, pz)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x, req.poses[x].orientation.y,
                req.poses[x].orientation.z, req.poses[x].orientation.w])

        print("roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw)
     
        # Calculate joint angles using Geometric IK method

        # 0. Useful Constants
        ee_length = d7
        ee_length = ee_length.subs(s)
        # a = l2
        l2_3 = a2
        l2_3 = l2_3.subs(s)
        # b = l3 + l4 w/ adjustment
        l3_4 = 0.96 # from URDF file
        l4_5 = 0.54 # from URDF file
        l3_4_offset = abs(a3)
        l3_4_offset = l3_4_offset.subs(s)
        l3_4_angle = pi - asin(l3_4_offset / l3_4)
        # Cosine rule
        dist3_5 = sqrt(l3_4**2 + l4_5**2 - 2*l3_4*l4_5*cos(l3_4_angle))
        print("dist3_5 = %f", dist3_5)
            
        # 1. Find total rotation matrix from roll-pitch-yaw data
        R_total = simplify(rot_x(roll) * rot_y(pitch) * rot_z(yaw))

        # 2. Find wrist center position using the end effector position and orientation
            # d6 = 0
            # wx = px - (d6 + l) * nx
            # wy = py - (d6 + l) * ny
            # wz = pz - (d6 + l) * nz

        ee_position = Matrix([[px],
                              [py],
                              [pz]])

        ee_adj = Matrix([[0],
                         [0],
                         [ee_length]])

        w_c = ee_position - R_total * ee_adj 

        # 3. theta1 calc
        theta1 = atan2(w_c[1,0], w_c[0,0])
        print("theta1 = ", theta1)

        # 4. theta2 calc
        J2_x = a1 * cos(theta1)
        J2_x = J2_x.subs(s)
        J2_y = a1 * sin(theta1)
        J2_y = J2_y.subs(s)
        J2_z = d1 
        J2_z = J2_z.subs(s)
        J5_x = w_c[0,0]
        J5_y = w_c[1,0]
        J5_z = w_c[2,0]

        dist_J2_J5 = sqrt((J5_x - J2_x)**2 + (J5_y - J2_y)**2 + (J5_z - J2_z)**2)
        dist_J2_J5_xy = sqrt((J5_x - J2_x)**2 + (J5_y - J2_y)**2)
        xc = w_c[0,0] - J2_x
        yc = w_c[2,0] - J2_z
        print("xc = ", xc)
        print("yc = ", yc)
        acos_innards = (dist3_5**2 - l2_3**2 - dist_J2_J5**2)/(-2*l2_3*dist_J2_J5)
        print("acos innards = ", acos_innards)
        theta2 = pi/2 - (acos((dist3_5**2 - l2_3**2 - dist_J2_J5**2)/(-2*l2_3*dist_J2_J5))) + acos(dist_J2_J5_xy/dist_J2_J5) 
        print("theta2 = ", theta2)
           
        # 5. theta3 calc
        theta3 = pi/2 - atan2(sqrt(1 - ((dist_J2_J5**2 - l2_3**2 - dist3_5**2) / (-2*l2_3*dist3_5))),
                                (dist_J2_J5**2 - l2_3**2 - dist3_5**2) / (-2*l2_3*dist3_5))
        print("theta3 = ", theta3)

        # Choose other theta3 calc
        # theta3_b = pi/2 - atan2(-sqrt(1 - ((xc**2 + yc**2 - l2_3**2 - dist3_5**2) / (-2*l2_3*dist3_5))),
        #                          (xc**2 + yc**2 - l2_3**2 - dist3_5**2) / (-2*l2_3*dist3_5))
		
        # 6. Find R3_6 from orientation data

        # R_rpy = R_total
        R0_3 = Matrix([[T0_3[0,0], T0_3[0,1], T0_3[0,2]],
                       [T0_3[1,0], T0_3[1,1], T0_3[1,2]],
                       [T0_3[2,0], T0_3[2,1], T0_3[2,2]]])
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            
        # for a valid rotation matrix the transpose is == to the inverse 
        R3_6 = simplify(R0_3.T * R_total)

        # 7. Find alpha, beta, gamma euler angles as done in lesson 2 part 8.

        # Method using euler_from_matrix assuming an xyx rotation rather than an xyz rotation
        alpha, beta, gamma = tf.transformations.euler_from_matrix(R3_6.tolist(), 'rxyx')
        theta4 = alpha
        theta5 = beta
        theta6 = gamma
            
        # alpha
        # theta4 = atan2(R3_6[1,0], R3_6[0,0])
        # beta
        # theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0] * R3_6[0,0] + R3_6[1,0] * R3_6[1,0]))
        # gamma
        # theta6 = atan2(R3_6[2,1], R3_6[2,2])




    ## Ending at: Populate response for the IK request
    ########################################################################################
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple posisiotns. It is best to add your forward kinmeatics to \
           \nlook at the confirm wether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

test_code(test_cases[test_case_number])
