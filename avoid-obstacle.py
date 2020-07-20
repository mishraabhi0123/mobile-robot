import numpy as np
import matplotlib.pyplot as plt 
from matplotlib.patches import Rectangle, Circle, Wedge
import cv2, time

DEFAULT_CONE_ANGLE = 3

view = 1
trail = True

def onpress(event):
      global view, trail
      if event.key == 'v':
            view = not view
      
      if event.key == 't':
            trail = not trail


fig = plt.figure()
ax = fig.add_subplot(111)
fig.canvas.mpl_connect('key_press_event',onpress)

class Motor():

      def __init__(self, N = 1000):
            self.N = N
            self.value = 0.0
            self.prev_value = 0
            self.rpm = 0


      def run(self):
            global dt
            self.value += self.rpm * self.N * dt / 60



class IR():

      def __init__(self, x, y, angle, range = (3,20), cone_angle = DEFAULT_CONE_ANGLE):
            self.xy = np.array([x,y])
            self.range = range
            self.cone_angle = cone_angle
            self.measurment = 5
            self.angle = angle
            self.noise = 0.0
            self.green_value = 30
            self.wedge_color = 'g'


      def plot(self):
            angle = self.angle - self.cone_angle/2.0
            plt.plot([self.xy[0]],[self.xy[1]], 'k.')
            ax.add_patch(Wedge((self.xy), self.range[1], angle, angle + self.cone_angle, width = self.range[1] - self.range[0],fc = self.wedge_color))


      def set_state(self, x, y, angle):
            self.xy = np.array([x,y])
            self.angle = angle

      
      def get_measurment(self):
            N_lin = 70 
            N_ang = 3
            ds = (self.range[1] - self.range[0]) / N_lin
            dtheta = self.cone_angle / N_ang
            angles = np.arange(self.angle - self.cone_angle/2,self.angle + self.cone_angle/2, dtheta)
            for theta in angles:
                  rad_theta = np.pi * theta / 180 
                  distance = self.range[0]
                  loc = self.xy + distance * np.array([np.cos(rad_theta), np.sin(rad_theta)])
                  for i in range(N_lin):
                        if image[int(loc[1]),int(loc[0])] == 0:
                              self.measurment = distance + np.random.normal(0,self.noise)
                              self.wedge_color = 'r'
                              return self.measurment
                        loc = loc + ds * np.array([np.cos(rad_theta), np.sin(rad_theta)])
                        distance += ds
            self.measurment = self.green_value
            self.wedge_color = 'g'
            return self.measurment


class MobileRobot():

      def __init__(self,x,y, angle = 0):
            global features
            self.location = np.array([x,y])
            self.angle = angle
            self.features = features

            self.front_ir = IR(0,0,0)
            self.left_ir_1 = IR(0,0,0)
            self.left_ir_2 = IR(0,0,0)
            self.left_ir_3 = IR(0,0,0)
            self.left_ir_4 = IR(0,0,0)

            self.right_ir_1 = IR(0,0,0)
            self.right_ir_2 = IR(0,0,0)
            self.right_ir_3 = IR(0,0,0)
            self.right_ir_4 = IR(0,0,0)

            self.left_motor = Motor()
            self.right_motor = Motor()
      
            self.history = [[],[]]

      def get_ir_measurments(self):

            f = self.front_ir.get_measurment()
            l1 = self.left_ir_1.get_measurment()
            l2 = self.left_ir_2.get_measurment()
            l3 = self.left_ir_3.get_measurment()
            l4 = self.left_ir_4.get_measurment()

            r1 = self.right_ir_1.get_measurment()
            r2 = self.right_ir_2.get_measurment()
            r3 = self.right_ir_3.get_measurment()
            r4 = self.right_ir_4.get_measurment()


            return [f,l1,l2,l3,l4,r1,r2,r3,r4]

      def update(self):
            NL = self.left_motor.value
            NR = self.right_motor.value

            L = features['length']
            B = features['breadth']
            R = features['wheel_radius']
            N = features['encoder_ticks']

            DL = 2*np.pi*R*(NL - self.left_motor.prev_value) / N
            DR = 2*np.pi*R*(NR - self.right_motor.prev_value) / N
            DC = (DL + DR) / 2
            phi = (self.angle * np.pi / 180)

            self.location[0] += DC * np.cos(phi)
            self.location[1] += DC * np.sin(phi)
            self.angle = 180 * (phi + (DR - DL) / B) / np.pi 
            self.angle = self.angle % 360

            self.left_motor.prev_value = NL
            self.right_motor.prev_value = NR

      def goto_goal(self, goal = [0,0]):
            x,y = goal
            X,Y = self.location
            angle = np.arctan(abs(Y-y)/(X-x + 0.0001))
 
            print(180*angle/np.pi)
            

      def plot_robot(self):
            global image , trail, view

            L = self.features['length']
            B = self.features['breadth']
            r = self.features['wheel_radius']
            t = self.features['wheel_thickness']

            x,y = location = self.location
            theta = self.angle
            theta_rad = np.pi * theta / 180

            self.history[0].append(x)
            self.history[1].append(y)

            self.front_ir.set_state(x,y, theta)

            self.left_ir_1.set_state(x,y, theta + 25)
            self.left_ir_2.set_state(x,y, theta + 50)
            self.left_ir_3.set_state(x,y, theta + 75)
            self.left_ir_4.set_state(x,y, theta + 100)

            self.right_ir_1.set_state(x,y, theta - 25)
            self.right_ir_2.set_state(x,y, theta - 50)
            self.right_ir_3.set_state(x,y, theta - 75)
            self.right_ir_4.set_state(x,y, theta - 100)


            plt.imshow(image,cmap= 'gray')

            temp1 = np.sqrt(((B+t)/2)**2 + r*r)
            temp2 = np.sqrt(((B-t)/2)**2 + 0.16*L*L)

            theta1 = np.pi/2 + theta_rad + np.arctan(2*r/(B+t))
            theta2 = np.pi/2 + theta_rad + np.arctan(0.8*L/(B-t))

            cir_loc = location + 0.5*L*np.array([np.cos(theta_rad), np.sin(theta_rad)])
            loc1 = location + temp1 * np.array([np.cos(theta1), np.sin(theta1)])
            loc2 = location + temp2 * np.array([np.cos(theta2), np.sin(theta2)])

            if trail:
                  ax.plot(self.history[0], self.history[1], 'm-')

            ax.grid(False)
            ax.add_patch(Circle(cir_loc, radius = .25*B, fc = 'b'))
            ax.add_patch(Rectangle(loc1, width = B+t, height = 2*r, angle = theta - 90, fc = 'b'))
            ax.add_patch(Rectangle(loc2, width= B-t, height = L, angle = theta - 90, fc = 'r'))
            ax.plot([self.location[0],0],[0,self.location[1]],'r.')

            self.front_ir.plot()
            self.left_ir_1.plot()
            self.left_ir_2.plot()
            self.left_ir_3.plot()
            self.left_ir_4.plot()

            self.right_ir_1.plot()
            self.right_ir_2.plot()
            self.right_ir_3.plot()
            self.right_ir_4.plot()

            if view:
                  ax.set_xlim(-0.0001,image.shape[1])
                  ax.set_ylim(-0.0001,image.shape[0])
            else :
                  ax.set_xlim(self.location[0] - 50, self.location[0] + 50)
                  ax.set_ylim(self.location[1] - 50, self.location[1] + 50)
            

def get_count(distance):
      target = (features['encoder_ticks'] / (2*np.pi*features['wheel_radius']))*distance
      return target

def calculate(goal_angle, current_angle):
      global features
      B = features['breadth']
      r = features['wheel_radius']
      N = features['encoder_ticks']
      nd = 0.5*B*(goal_angle - current_angle) *N / ( 720*r )
      return -nd, nd


features = {

      'length'          :      14,
      'breadth'         :      10,
      'wheel_radius'    :       4,
      'wheel_thickness' :       2,
      'encoder_ticks'   :    1000,

}

image = plt.imread('obstacle.jpg',0)
image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


image = image // 255.0
image = image > 0.5

dt = 0.1
sim_time = 0.0

plt.ion()

r1 = MobileRobot(126.0,56.0,120.0)
rpm = 50
kp = 1

while True:
      sim_time += dt
      plt.cla()

      r1.plot_robot()

      Z1 = np.array(r1.get_ir_measurments())

      x1 = r1.angle
      angles1 = np.array([x1,x1+25,x1+50,x1+75,x1+100,x1-25,x1-50,x1-75,x1-100])
      goal_angle1 = Z1.dot(angles1)/ sum(Z1)
      n11, n21 = calculate(goal_angle1, r1.angle)

      r1.left_motor.rpm = rpm + kp * (n11)
      r1.right_motor.rpm = rpm + kp * (n21)

      r1.left_motor.run()
      r1.right_motor.run()

      r1.update()
      plt.title(f'simulation time :{sim_time//1}')
      plt.show()
      plt.pause(0.001)