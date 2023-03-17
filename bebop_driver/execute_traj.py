

from std_msgs.msg import String, Bool, UInt8, Float32



class traj_executer():
    def __init__(self, routine_name):
        # smach.State.__init__(self, outcomes=['done'])

        self.pose_pub = rospy.Publisher("/control/position", Pose, queue_size=1)
        self.traj_name_sub = rospy.Subscriber("traj_name", String,
        self.traj_name_sub= rospy.Subscriber("traj_name",String, self.traj_name_callback)
         queue_size=1)

        self.running_control_pub = rospy.Publisher("/control/set_running_state", Bool, queue_size=1)
        # self.running_control_pub = rospy.Publisher("/control/aligned", Bool, queue_size=1)
        self.routine_name = routine_name

    def traj_name_callback(self, msg):
        self.traj_name = msg.data

    def run(self, userdata):
        # rospy.sleep(2)
        self.running_control_pub.publish(True)
        # self.pose_pub.publish()

        if self.routine_name in moving_routines:
            for position in moving_routines[self.routine_name]:
                print("------------- pose ----------")
                new_pose = ros_numpy.msgify(Pose,np.array(position))
                print(new_pose)
                self.pose_pub.publish(new_pose)
                rospy.wait_for_message("/control/aligned", Bool)
        else:
            print("no routine named: "+self.routine_name)
            print(moving_routines)
        self.running_control_pub.publish(False)
        return 'done'
    

te = traj_executer()
te.run()