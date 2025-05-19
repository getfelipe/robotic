#!/usr/bin/env python3
import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtlePIDController:
    def __init__(self):
        rospy.init_node('turtle_pid_controller')
        self.input_user = int(input('1 - Quadrado / 2 - Triângulo: \n'))
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.gesture_subcriber = rospy.Subscriber("/turtle1/pose", 
                                                Pose, self.pose_callback)
        self.error_acumulated = 0
        self.error_before = 0
        self.error_acumulated_x = 0
        self.error_before_x = 0

        self.pos_y = 0.0
        self.pos_x = 0.0
        self.destino_x = 0.0 
        self.destino_y = 0.0
        self.erro_theta = 0.0

        self.i = 0
        self.square_defined = False
        self.triangle_defined = False
        self.square_x = []
        self.square_y = []
        self.try_x = []
        self.try_y = []

    def formato(self):
        if self.input_user == 1:
            self.turtle_square()
        else:
            self.turtle_triangle()


    def turtle_square(self):
        self.pos_x, self.pos_y = self.get_current_position()

        if not self.square_defined:
            self.square_x=[self.pos_x, self.pos_x+1, self.pos_x+1, self.pos_x]
            self.square_y=[self.pos_y+1, self.pos_y+1, self.pos_y, self.pos_y]
            self.square_defined = True

        idx = self.i % len(self.square_x)
        self.destino_x = self.square_x[idx]
        self.destino_y = self.square_y[idx]

        if abs(self.pos_x - self.destino_x) < 0.1 and abs(self.pos_y - self.destino_y) < 0.1:
            self.i += 1

    def turtle_triangle(self):
        self.pos_x, self.pos_y = self.get_current_position()
       

    def pose_callback(self, msg):
        current_x = msg.x
        current_y = msg.y
        theta = msg.theta

        self.pos_x = current_x
        self.pos_y = current_y
        self.formato()

        erro_x = self.destino_x - current_x
        erro_y = self.destino_y - current_y
        distancia =  math.sqrt((erro_x**2) + (erro_y**2))
        theta_destino = math.atan2(erro_y,erro_x)
        theta_erro = theta_destino - theta

        kpx =  0.6
        kix = 0.1
        kdx = 0.2

        kpy =  0.6
        kiy = 0.1
        kdy = 0.2
        hz = 0.1

        cmd = Twist()
        print('-='*10, 'theta_erro', '-='*10)
        print(theta_erro)
        print()

        if abs(theta_erro) > 0.1:
            py = kpy * theta_erro
            
            integral_y = (kiy*self.error_acumulated)*hz
            self.error_acumulated += theta_erro
            
            dy = (kdy * (theta_erro - self.error_before))/hz
            self.error_before = theta_erro

            cmd.angular.z = py + integral_y + dy
            cmd.linear.x = 0.0


            print('-='*10, 'cmd.angular.z', '-='*10)
            print(distancia)
            print(str(cmd.angular.z))
            print()

        else:
            p = kpx * distancia
            i = (kix*self.error_acumulated_x)*hz
            self.error_acumulated_x += distancia

            d = (kdx * (distancia - self.error_before_x))/hz
            self.error_before_x = distancia
            cmd.linear.x = p + i + d
            cmd.angular.z = 0.0

            print('-='*10, 'cmd.linear.x', '-='*10)
            print(str(cmd.linear.x))
            print()

        self.cmd_vel_pub.publish(cmd)

        print('-='*10, 'POSIÇÃO ATUAL', '-='*10)
        print(f"[{current_x, current_y}]")
        print(f"[{self.destino_x,self.destino_y }]")

        #rospy.loginfo("Posição atual: %.2f | Erro: %.2f | Velocidade: %.2f" % (current_x, error, output))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurtlePIDController()
        controller.run()
    except rospy.ROSInterruptException:
        pass