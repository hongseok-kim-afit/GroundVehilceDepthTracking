import json
import matplotlib.pyplot as plt
import numpy as np
import math as m
from scipy.interpolate import splprep, splev
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import random


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
y_goal=[]
z_goal=[]
x_object=[]
z_object=[]

total_folder=['3m/single_box','3m/multi_box','3m/single_triangle','3m/multi_triangle',
              '3m/single_box_avoid','3m/multi_box_avoid','3m/single_triangle_avoid','3m/multi_triangle_avoid', 
              '5m/single_box','5m/multi_box','5m/single_triangle','5m/multi_triangle',
              '5m/single_box_avoid','5m/multi_box_avoid','5m/single_triangle_avoid','5m/multi_triangle_avoid']
best_pick=[1,4,2,3,1,4,2,3,5,5,3,3,4,5,5,4]
##################################################
root_data = './test/data/'
folder_data = 'D:/final_data/'
test_data = 'D:/final_data/3m/multi_triangle_avoid/'
file_path =[]
#change folder_name and add '/' at the end
#folder_name = '2021-10-12(1200_left_turn)' # max left turn 1200
num = 16  # test times

for i in range(0,num,1): # 12 is last box +1
    folder_name = str(best_pick[i])
    test_data=folder_data+total_folder[i]+'/'
    # stop1, stpt_1_15
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
    y_goal=[]
    z_goal=[]
    x_object=[]
    z_object=[]
    # if you do test put the 
    #file_path = root_data+folder_name
    file_path=test_data+folder_name

    # read file
    #with open(file_path+'/result_data.json', 'r') as myfile:
    #    load_data = myfile.read()
    load_data=open(file_path+'/result_data.json', 'r')
    load_data=load_data.read()
    # parse file
    ex_result = json.loads(load_data)


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
        y_goal.append(result[1]['goal'][1])
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
    # for error
    # https://matplotlib.org/stable/gallery/lines_bars_and_markers/curve_error_band.html#sphx-glr-gallery-lines-bars-and-markers-curve-error-band-py
    # err = random.random()
    err = 0.3
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

    vertices = np.block([[xp, xn[::-1]], [zp, zn[::-1]]]).T # <start>:<stop>:<step>,::-1 = reverse whole list
    codes = Path.LINETO * np.ones(len(vertices), dtype=Path.code_type)
    codes[0] = codes[len(xp)] = Path.MOVETO
    path = Path(vertices, codes)

    patch = PathPatch(path, facecolor='C0', edgecolor='none', alpha=0.3)


    ####################################################
    # for final distance
    x_dis = x[len(x)-1]
    y_dis = y[len(y)-1]
    z_dis = z[len(z)-1]

    goal_x = x_goal[len(x_goal)-1]
    goal_y = y_goal[len(y_goal)-1]
    goal_z = z_goal[len(z_goal)-1]

    final_distance = 0.3 - m.sqrt((goal_x - x_dis)**2 + (goal_y - y_dis)**2 + (goal_z - z_dis)**2)
    #print('distance',final_distance)


    #print(x_object)
    #print(z_object)
    #print(x_goal)
    fig, ax = plt.subplots()
    ax.plot(x,z)
    ax.add_patch(patch)
    plt.axis('equal')
    plt.plot(x_goal,z_goal,'o', color='b')
    #plt.scatter(tracking_pose[0],  -tracking_pose[2])
    plt.text(x[0], z[0], 'start', color='g',
            horizontalalignment='center', verticalalignment='top')
        
    plt.text(x[len(x)-1], z[len(z)-1], str(final_distance), color='red',
            horizontalalignment='center', verticalalignment='top')
    plt.scatter(x_object,z_object, color='r')
    # vector from position to object
    for i in range(len(x)):
        if x_object[i] != 0:
            plt.quiver(x[i],z[i], x_object[i]-x[i], z_object[i]-z[i], color='b', units='xy', scale=1)
            
    #plt.text(x_object,z_object)
    plt.xlabel('x')
    plt.ylabel('z')
    plt.show()
