import numpy as np
import matplotlib.pyplot as plt
DEV_LEN = 1

# Target is length 10 at 45 deg from each axis
# TARGET = np.array([
#                     [np.sqrt(10/3)],
#                     [np.sqrt(10/3)],
#                     [np.sqrt(10/3)]
# ])
TARGET = np.array([
                    [np.sqrt(1)],
                    [np.sqrt(0)],
                    [np.sqrt(0)]
])

OFFSET = np.array([
                    [np.sqrt(1/2)],
                    [0],
                    [np.sqrt(1/2)]
])


# Rotation 1 (rotate about onto xz plane: rotate about z-axis)
phi = np.arctan(TARGET[0][0]/TARGET[1][0]) # atan(x/y)
print(phi)
m1 = np.array([
            [np.cos(phi),-np.sin(phi),0],
            [np.sin(phi),np.cos(phi),0],
            [0,0,1]
]) # Rotate about z axis
stage1_rotation = np.matmul(m1,TARGET)

# Rotation 3 (rotate onto x axis: y=z=0) I.e. rotate about y axis
phi = np.arctan(stage1_rotation[2][0]/stage1_rotation[0][0]) # atan(z/x)
print(phi)
m2 = np.array([
            [np.cos(phi),0,np.sin(phi)],
            [0,1,0],
            [-np.sin(phi),0,np.cos(phi)]
]) # Rotate about y axis

rotation_mat = np.matmul(m1,m2)
inv_rotation_mat = np.linalg.inv(rotation_mat)


# 45 deg in xz plane
line_to_rotate = np.array([
            [np.sqrt(1/1.1716)],
            [0],
            [np.sqrt(0.414/1.1716)]
])
deg = 45
phi = np.deg2rad(deg)
x_rotation_45_deg = np.array([
            [1,0,0],
            [0,np.cos(phi),-np.sin(phi)],
            [0,np.sin(phi),np.cos(phi)]
            
])

ax = plt.axes(projection='3d')
lim = 2
ax.axes.set_xlim3d(left=-lim, right=lim) 
ax.axes.set_ylim3d(bottom=-lim, top=lim) 
ax.axes.set_zlim3d(bottom=-lim, top=lim) 

# g_vec = [line_to_rotate]
# for phi in range(0,360,deg):
#     g_vec.append(np.matmul(x_rotation_45_deg, g_vec[-1]))
#     ax.scatter3D(g_vec[-1][0], g_vec[-1][1], g_vec[-1][2], c="green")

# g_vec = np.matmul(inv_rotation_mat,g_vec)
# for item in g_vec:
#     ax.scatter3D(item[0], item[1], item[2], c="red")

# ax.plot([0 ,1],[0,0],[0,0],c="green")
# ax.plot([0 ,1],[0,1],[0,1],c="red")

# print(g_vec)





g_vec = np.array([[0],[0],[0]])

# Now try to roatate g vector and find component in x, y, and z directions
phi = np.deg2rad(-10)
y_45_deg = np.array([
            [np.cos(phi),0,np.sin(phi)],
            [0,1,0],
            [-np.sin(phi),0,np.cos(phi)]
]) # Rotate about y axis
x_axis = np.matmul(y_45_deg,np.array([[1],[0],[0]]))
y_axis = np.matmul(y_45_deg,np.array([[0],[1],[0]]))
z_axis = np.matmul(y_45_deg,np.array([[0],[0],[1]]))
g_axis = np.array([[1],[-1],[1]]).transpose()

for phi in range(0,360,deg):
    theta = np.deg2rad(phi)
    x_rotation = np.array([
            [1,0,0],
            [0,np.cos(theta),-np.sin(theta)],
            [0,np.sin(theta),np.cos(theta)]])


    x_axis_new = np.matmul(x_rotation,x_axis)
    y_axis_new = np.matmul(x_rotation,y_axis)
    z_axis_new = np.matmul(x_rotation,z_axis)
    g_vec_new = np.array([np.dot(g_axis,x_axis_new)[0],
    np.dot(g_axis,y_axis_new)[0],
    np.dot(g_axis,z_axis_new)[0]])
    g_vec = np.concatenate((g_vec,g_vec_new),axis = 1)
    print(g_vec)
    ax.scatter3D(g_vec[:,-1][0], g_vec[:,-1][1], g_vec[:,-1][2], c="orange")

print("\nFINISHED GVEC:\n",g_vec)
svd = np.linalg.svd(g_vec - np.mean(g_vec, axis=1, keepdims=True))
left = svd[0]
line = left[:, -1]
print("Normal:",line)
ax.plot3D([0,line[0]],[0,line[1]],[0,line[2]],c="red")
print(left[:, -1] @ g_vec)

print(f"Initial vector: {g_axis}")
if np.dot(g_vec[:,0],line) < 0 :
     line = -line
print("Normal:",line)
    

plt.show()

