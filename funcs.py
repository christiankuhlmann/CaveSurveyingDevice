import numpy as np

def calculate_normal_vec(g_vec: np.ndarray):
    svd = np.linalg.svd(g_vec - np.mean(g_vec, axis=1, keepdims=True))
    left = svd[0]
    line = left[:, -1]
    print("SVD magnitude:",np.linalg.norm(line))
    return line

def calc_true_vec(normal_vec: np.ndarray, laser_distances: np.ndarray, disto_len: float):
    laser_length = np.mean(laser_distances)
    x_axis = np.array([1,0,0]).T

    b1 = normal_vec
    b2 = np.array([[1],[0],[0]])
    alpha = np.arccos(np.dot(b1,b2) / np.linalg.norm(b1)*np.linalg.norm(b2))
    if alpha > np.pi/2:
        alpha = np.pi-alpha
        b1 = -b1
    print("alpha:",np.rad2deg(alpha))
    print("normal_vec:", b1)


    l_0 = disto_len
    l_1 = laser_length
    l_2 = l_0 * np.sin(alpha)
    l_3 = l_0 * np.cos(alpha) + np.sqrt(l_1**2-l_2**2)

    true_vec = l_3 * x_axis + l_0 * normal_vec
    return true_vec

if __name__ == "__main__":

    g_vec = np.array([[0],[0],[0]])
    deg = 45

    # Now try to roatate g vector and find component in x, y, and z directions
    phi = np.deg2rad(-20)
    y_45_deg = np.array([
                [np.cos(phi),0,np.sin(phi)],
                [0,1,0],
                [-np.sin(phi),0,np.cos(phi)]
    ]) # Rotate about y axis
    x_axis = np.matmul(y_45_deg,np.array([[1],[0],[0]]))
    y_axis = np.matmul(y_45_deg,np.array([[0],[1],[0]]))
    z_axis = np.matmul(y_45_deg,np.array([[0],[0],[1]]))
    g_axis = np.array([[0],[0],[1]]).transpose()


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


    normal_vec = calculate_normal_vec(g_vec)
    true_vec = calc_true_vec(normal_vec,np.array([20,20,20]),5)
    print(true_vec)