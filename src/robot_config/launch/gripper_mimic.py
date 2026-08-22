#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class GripperMimic(Node):
    def __init__(self):
        super().__init__('gripper_mimic_node')
        
        # 1. Subscribe στο Joint States για να βλέπουμε τον "Αρχηγό"
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
            
        # 2. Publish εντολών στον Mimic Controller
        self.publisher = self.create_publisher(Float64MultiArray, '/mimic_controller/commands', 10)
        
        # 3. ΡΥΘΜΙΣΗ: Οι πολλαπλασιαστές (Multipliers)
        # ΠΡΟΣΟΧΗ: Η σειρά πρέπει να είναι ΙΔΙΑ με τη σειρά στο ros2_controllers.yaml
        self.mimic_map = {
            'endeffector_gripper': [
                -1.0, # 1. gripperbase_to_armgearleft   (Αριστερό Γρανάζι: Αντίθετο)
                 1.0, # 2. gripperbase_to_armsimpleright (Δεξί Μπράτσο: Ίδιο)
                -1.0, # 3. gripperbase_to_armsimpleleft  (Αριστερό Μπράτσο: Αντίθετο)
                -1.0, # 4. armgearright_to_fingerright   (Δεξί Δάχτυλο: Αντίστροφη περιστροφή για να μένει ίσιο)
                -1.0  # 5. armgearleft_to_fingerleft     (Αριστερό Δάχτυλο: Αντίστροφη περιστροφή)
            ]
        }

    def listener_callback(self, msg):
        try:
            # Έλεγχος αν υπάρχει το joint του αρχηγού στο μήνυμα
            if 'endeffector_gripper' in msg.name:
                index = msg.name.index('endeffector_gripper')
                current_pos = msg.position[index]
                
                # Δημιουργία μηνύματος εντολής
                commands = Float64MultiArray()
                multipliers = self.mimic_map['endeffector_gripper']
                
                # Υπολογισμός θέσεων: Θέση Αρχηγού * Πολλαπλασιαστής
                commands.data = [current_pos * m for m in multipliers]
                
                # Αποστολή
                self.publisher.publish(commands)
            
        except ValueError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Error in mimic callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperMimic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
