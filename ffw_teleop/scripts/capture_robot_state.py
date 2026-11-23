import rclpy,json,time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from controller_manager_msgs.srv import ListControllers
rclpy.init();n=rclpy.create_node('human_replay_inspect');out={}
n.create_subscription(JointState,'/joint_states',lambda m:out.update(joints=dict(zip(m.name,m.position))),qos_profile_sensor_data)
n.create_subscription(String,'/robot_description',lambda m:out.update(urdf=m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
c=n.create_client(ListControllers,'/controller_manager/list_controllers');assert c.wait_for_service(timeout_sec=4);f=c.call_async(ListControllers.Request());end=time.monotonic()+8
while time.monotonic()<end and not (f.done() and 'urdf' in out and 'joints' in out):rclpy.spin_once(n,timeout_sec=.1)
assert f.done() and 'urdf' in out and 'joints' in out
out['controllers']=[dict(name=x.name,state=x.state,interfaces=list(x.claimed_interfaces)) for x in f.result().controller]
out['publishers']={c:n.count_publishers('/'+c+'/joint_trajectory') for c in ['arm_l_controller','arm_r_controller','hand_l_controller','hand_r_controller']}
from pathlib import Path
out['ros_time_ns']=n.get_clock().now().nanoseconds
board=Path('/tmp/human_demo_robot_board.json')
out['board']=json.loads(board.read_text()) if board.exists() else {'valid':False,'reason':'no observer state'}
import sys
if '--wait-board' in sys.argv:
 deadline=time.monotonic()+15.
 while time.monotonic()<deadline:
  sample=json.loads(board.read_text()) if board.exists() else {'valid':False}
  if sample.get('valid') and 0 <= (n.get_clock().now().nanoseconds-sample['image_stamp_ns'])/1e9 <= .5:
   out['board']=sample
   break
  rclpy.spin_once(n,timeout_sec=.05)
 else:
  raise RuntimeError('No stable fresh board within 15 seconds')
out['ros_time_ns']=n.get_clock().now().nanoseconds
print(json.dumps(out));n.destroy_node();rclpy.shutdown()
