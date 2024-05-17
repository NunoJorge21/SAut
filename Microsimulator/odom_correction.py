import rosbag
import rospy
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
# Open the input bag file
euler_angles=[]
with rosbag.Bag('piso5.bag', 'r') as in_bag:
    # Create a new bag file for the output
    with rosbag.Bag('noice.bag', 'w') as out_bag:
        # Iterate over the messages in the input bag
        for topic, msg, t in in_bag.read_messages(topics=['/tf','/scan']):
            # Check if the message is of the correct type
            if msg._type == 'geometry_msgs/TransformStamped':
                # Modify the message values here
                euler_angles=euler_from_quaternion(msg.transform.rotation)
                euler_angles[0]+=2
                euler_angles[1]+=5
                euler_angles[2]+=1
                msg.transform.rotation=quaternion_from_euler(euler_angles)
                # Write the message to the output bag
                out_bag.write(topic, msg, t)
            else:
                # Write the message to the output bag without modifying it
                out_bag.write(topic, msg, t)