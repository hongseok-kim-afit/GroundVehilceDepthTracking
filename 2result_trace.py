import json
import matplotlib.pyplot as plt
import numpy as np
import math as m
from scipy.interpolate import splprep, splev
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import random
from collections import deque
import matplotlib.animation as animation
import cv2
from time import sleep
import matplotlib as mpl 



## you have to change folder_name 
# set the file path

i=0
time=[]
x=[]
y=[]
z=[]
vx=[]
vy=[]
vz=[]
ax=[]
ay=[]
az=[]
depth=[]
vehicle_heading=[]
x_goal=[]
z_goal=[]
x_object=[]
z_object=[]

'''
total_folder=['3m/single_box','3m/multi_box','3m/single_triangle','3m/multi_triangle',
              '3m/single_box_avoid','3m/multi_box_avoid','3m/single_triangle_avoid','3m/multi_triangle_avoid', 
              '5m/single_box','5m/multi_box','5m/single_triangle','5m/multi_triangle',
              '5m/single_box_avoid','5m/multi_box_avoid','5m/single_triangle_avoid','5m/multi_triangle_avoid']
best_pick=[1,4,2,3,
            1,4,4,3,
            5,5,3,3,
            4,5,5,4]
'''
##################################################
root_data = './test/data/'
test_data = './test/data/'
folder_data = 'D:/final_data/'
test_data = 'D:/final_data/3m/multi_box_avoid/'
#change folder_name and add '/' at the end
#folder_name = '2021-10-12(1200_left_turn)' # max left turn 1200
folder_name = '5'
# stop1, stpt_1_15

# if you do test put the 
#file_path = root_data+folder_name
file_path = test_data+folder_name

# read file
with open(file_path+'/result_data.json', 'r') as myfile:
    load_data = myfile.read()

# parse file
ex_result = json.loads(load_data)

# 
for result in enumerate (ex_result):
    #print(result)
    time.append(result[1]['time'])
    #postion[0]= x(forward), position[1] = y(altitude), z= position
    x.append(result[1]['position'][0])
    y.append(result[1]['position'][1])
    z.append(-result[1]['position'][2])
    depth.append(result[1]['object'][1])
    vehicle_heading.append(result[1]['vehicle_heading'])
    x_goal.append(result[1]['goal'][0])
    z_goal.append(result[1]['goal'][2])
    x_object.append(result[1]['object'][0][0])
    z_object.append(result[1]['object'][0][2])
####################################################

# test for d_t_t
#a = [pose_data.translation.x, pose_data.translation.y, pose_data.translation.z, 1]
pixel_location=[0,320]
#pixel_location=[640,320]
width = 640
depth = 2.
fov = 87. * m.pi/180. # Depth Field of View (FOV): 87° × 58°, RGB sensor FOV (H × V): 69° × 42°
#####################################################
# for trace

dt = 0.5/1.55/67*118 # 1 / frame per second
ddt = 1/dt /3/67*118# for realtime 

#t_stop = 5 # how many seconds to simulate
history_len = len(x)  # how many trajectory points to display
max_x = max(x)
max_z = max(z)
large_xz = max(max_x, max_z) + 0.5

min_x = min(x)
min_z = min(z)
small_xz = min(min_x, min_z) - 0.5
#####################################################
# for error
err = 0.1

# calculate normals via derivatives of splines
tck, u = splprep([x,z], s=0)
dx, dz = splev(u, tck, der=1)
l = np.hypot(dx,dz) # get the sqrt(dx**2,dz**2)

nx = dz / l
nz = -dx / l

# end points of errors
xp = x + nx * err
zp = z + nz * err
xn = x - nx * err
zn = z - nz * err

vertices = np.block([[xp, xn[::-1]], [zp, zn[::-1]]]).T
codes = Path.LINETO * np.ones(len(vertices), dtype=Path.code_type)
codes[0] = codes[len(xp)] = Path.MOVETO
path = Path(vertices, codes)

patch = PathPatch(path, facecolor='C0', edgecolor='none', alpha=0.3)


####################################################
#plot part
############################3
#print(x_object)
#print(z_object)
#print(x_goal)
fig = plt.figure(figsize=(5, 4))
ax = fig.add_subplot(autoscale_on=False, xlim=(small_xz, large_xz), ylim=(small_xz, large_xz))

########################################################
# for trace
# https://matplotlib.org/stable/gallery/animation/double_pendulum.html#sphx-glr-gallery-animation-double-pendulum-py

ax.grid()
spot, = ax.plot([], [], 'o', lw=1)
trace, = ax.plot([], [], ',-', lw=2,color='C0')
detect, =ax.plot([], [], 'o', lw=2, color='red') 

######################################################################
# arrow
#https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.quiver.html
q_u, q_v = np.zeros(len(z)),np.zeros(len(z))
if x_object[i] != 0:

    arrow = ax.quiver(x[0],z[0],x_object[0] - x[0], z_object[0] - z[0])

# time stamp
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

# trace
history_x, history_z = deque(maxlen=history_len), deque(maxlen=history_len)

def animate(i):
    global arrow
    # current position
    thisx = [0, x[i]]
    thisz = [0, z[i]]

    if i == 0:
        history_x.clear()
        history_z.clear()
    # for daw arrow when detect target
    if x_object[i] != 0:
        #####################################################################################
        # wait keyboard 'Enter' input
        #input("Press Enter to continue...") # if you press enter the next process going on
        #sleep(0.5)
        # draw arrow from current position to detected object
        arrow.remove()
       
        #arrow = ax.quiver(x[i], z[i],q_u, q_v, color='red', units='xy', scale=1)
        arrow = ax.quiver(x[i], z[i], x_object[i] - x[i], z_object[i] - z[i], color='b', units='xy', scale=1)

        # draw detected object
        detect.set_data(x_object[i],z_object[i])

    # trace
    history_x.appendleft(thisx[1])
    history_z.appendleft(thisz[1])

    # current position
    spot.set_data(thisx, thisz)

    # trace
    trace.set_data(history_x, history_z)
    # time stamp
    time_text.set_text(time_template % (i*dt*ddt)) # mutiply by 10 for time sequence
    return spot, trace, time_text, arrow, detect

ani = animation.FuncAnimation(
    fig, animate, len(z), interval=dt*1000, blit=True)
# f = r"D:/final_data/3m/multi_box_avoid/animation.avi" 
# writervideo = animation.FFMpegWriter(fps=60)
# ani.save(f, writer=writervideo)
###########################################################3
#ax.plot(x,z)
#ax.add_patch(patch)

#plt.axis('equal')
plt.plot(x_goal,z_goal,'o', color='b')
#plt.scatter(tracking_pose[0],  -tracking_pose[2])
plt.text(x[0], z[0], 'start', color='g',
        horizontalalignment='center', verticalalignment='top')
plt.scatter(x_object,z_object, color='lightpink')
'''
# vector from position to object
for i in range(len(x)):
    if x_object[i] != 0:
        plt.quiver(x[i],z[i], x_object[i]-x[i], z_object[i]-z[i], color='b', units='xy', scale=1)
'''     
#plt.text(x_object,z_object)
plt.xlabel('x')
plt.ylabel('z')

plt.show()
