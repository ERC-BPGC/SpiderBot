# import rospy
# from std_msgs.msg import Float64

from cpg import *
import numpy as np


class JointCmds:
    """
    The class provides a dictionary mapping joints to command values.
    """
    def __init__( self ) :
        
        self.joints_list = []
        for i in range(6) :
            for j in range(3) :
                leg_str='L' + str(i+1) + '_' + str(j+1)
                self.joints_list += [leg_str]

        self.cpg_joints = ['L6_2', 'L1_2', 'L4_2', 'L3_2','L2_2','L5_2']
        self.group_joints = [['L1_ 1', 'L4_1'], ['L6_1', 'L3_1'],
                             ['L2_2', 'L5_2'], ['L2_3', 'L5_3']]
        self.group_joints_flat = [item for sublist in self.group_joints \
                                  for item in sublist]
        self.cpg = CPG()
        self.jnt_cmd_dict = {}

    def update( self, dt ) :
        s = self.cpg.simulate(dt)
        x = s[:2]
        y = s[2:]

        for i, jnt in enumerate(self.cpg_joints) :
            self.jnt_cmd_dict[jnt] = max(0.5*y[i],0)
            # This takes care of joint 2 of moving legs
        self.jnt_cmd_dict['L1_1'] = 0
        self.jnt_cmd_dict['L2_1'] = 0
        self.jnt_cmd_dict['L3_1'] = 0
        self.jnt_cmd_dict['L4_1'] = 0
        self.jnt_cmd_dict['L5_1'] = 0
        self.jnt_cmd_dict['L6_1'] = 0


        self.jnt_cmd_dict['L1_1'] += .1*(np.cos(np.arctan2(y[0],x[0])+np.pi)+1)
        self.jnt_cmd_dict['L2_1'] -= .1*(np.cos(np.arctan2(y[1],x[1])+np.pi)+1)
        self.jnt_cmd_dict['L3_1'] += .1*(np.cos(np.arctan2(y[2],x[2])+np.pi)+1)
        self.jnt_cmd_dict['L4_1'] -= .1*(np.cos(np.arctan2(y[3],x[3])+np.pi)+1)
        self.jnt_cmd_dict['L5_1'] -= .1*(np.cos(np.arctan2(y[4],x[4])+np.pi)+1)
        self.jnt_cmd_dict['L6_1'] -= .1*(np.cos(np.arctan2(y[5],x[5])+np.pi)+1)

        for jnt in self.group_joints[2] :
            self.jnt_cmd_dict[jnt] = 1.57
        for jnt in self.group_joints[3] :
            self.jnt_cmd_dict[jnt] = -1.57
            # fixed the position of leg 2 and 5 which is not going to be used
        for jnt in self.joints_list :
            if jnt not in self.cpg_joints and jnt not in self.group_joints_flat :
                self.jnt_cmd_dict[jnt] = 0.0
                 # This says if a joint is not defined keep at 0 state
        return self.jnt_cmd_dict


# def publish_commands( hz ):
#     pub={}
#     ns_str = '/snake_monster'
#     cont_str = 'eff_pos_controller'
#     for i in range(6) :
#         for j in range(3) :
#             leg_str='L' + str(i+1) + '_' + str(j+1)
#     #         pub[leg_str] = rospy.Publisher( ns_str + '/' + leg_str + '_'
#     #                                         + cont_str + '/command',
#     #                                         Float64, queue_size=10 )
#     # rospy.init_node('walking_controller', anonymous=True)
#     # rate = rospy.Rate(hz)
#     jntcmds = JointCmds()
#     while not rospy.is_shutdown():
#         jnt_cmd_dict = jntcmds.update(1./hz)
#         for jnt in jnt_cmd_dict.keys() :
#             pub[jnt].publish( jnt_cmd_dict[jnt] )
#         rate.sleep()



    cpg = CPG()
    hz=30
    dt=1./hz
    # cpg.plot(100,dt)
