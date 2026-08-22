import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymycobot.myarm import MyArm
import pigpio
import time

# --- ΡΥΘΜΙΣΕΙΣ ---
PI_IP = '192.168.123.20'
VIRTUAL_PORT = '/tmp/virtual_robot'
GRIPPER_PIN = 18

class LanDriver(Node):
    def __init__(self):
        super().__init__('lan_driver_node')
        self.get_logger().info('--- LAN Driver: ΔΙΟΡΘΩΜΕΝΗ ΕΚΔΟΣΗ ---')

        # 1. Σύνδεση με Gripper (pigpio)
        try:
            self.pi = pigpio.pi(PI_IP, 8888)
            if not self.pi.connected: raise Exception("Pigpio connection failed")
            self.get_logger().info('✅ Gripper Connected!')
        except Exception as e:
            self.get_logger().error(f'❌ Gripper Error: {e}')
            exit()

        # 2. Σύνδεση με Βραχίονα
        try:
            self.mc = MyArm(VIRTUAL_PORT, 115200)
            time.sleep(1)
            self.mc.power_on()
            self.get_logger().info('✅ Arm Connected!')
        except Exception as e:
            self.get_logger().error(f'❌ Arm Error: {e}')
            exit()

        # 3. Subscriber
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        
        self.last_gripper_val = -999

    def listener_callback(self, msg):
        try:
            # --- Α. ΒΡΑΧΙΟΝΑΣ (ΔΙΟΡΘΩΣΗ ΟΝΟΜΑΤΩΝ) ---
            # Τα ονόματα όπως φαίνονται στην εικόνα του GUI σου
            target_joints = [
                'joint1_to_base', 
                'joint2_to_joint1', 
                'joint3_to_joint2', 
                'joint4_to_joint3', 
                'joint5_to_joint4', 
                'joint6_to_joint5', 
                'joint7_to_joint6'
            ]
            
            arm_angles = []
            found_all = True
            
            # Ψάχνουμε τα σωστά ονόματα μέσα στο μήνυμα
            for name in target_joints:
                if name in msg.name:
                    idx = msg.name.index(name)
                    arm_angles.append(msg.position[idx])
                else:
                    found_all = False
            
            # Στέλνουμε εντολή μόνο αν βρήκαμε και τα 7 joints
            if found_all and len(arm_angles) == 7:
                self.mc.send_radians(arm_angles, 80)

            # --- Β. GRIPPER (ΔΙΟΡΘΩΣΗ ΦΟΡΑΣ) ---
            if 'endeffector_gripper' in msg.name:
                idx = msg.name.index('endeffector_gripper')
                val = msg.position[idx]
                
                if abs(val - self.last_gripper_val) > 0.01:
                    self.move_gripper(val)
                    self.last_gripper_val = val

        except Exception as e:
            pass

    def move_gripper(self, radian_val):
        # --- ΑΝΤΙΣΤΡΟΦΗ ΚΙΝΗΣΗΣ ---
        # Το Slider δίνει τιμές από 0.0 έως -1.0 (ή περίπου εκεί)
        # Χρησιμοποιούμε την απόλυτη τιμή (abs) για το ποσοστό
        
        percentage = abs(radian_val) / 1.57
        if percentage > 1.0: percentage = 1.0
        
        min_pulse = 500
        max_pulse = 2500
        
        # ΠΑΛΙΟΣ ΤΥΠΟΣ (Ανοίγει όταν αυξάνεται):
        # pulse = min_pulse + (percentage * (max_pulse - min_pulse))
        
        # ΝΕΟΣ ΤΥΠΟΣ (Κλείνει όταν αυξάνεται - Ανάποδα):
        pulse = max_pulse - (percentage * (max_pulse - min_pulse))
        
        self.pi.set_servo_pulsewidth(GRIPPER_PIN, pulse)

def main(args=None):
    rclpy.init(args=args)
    driver = LanDriver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
