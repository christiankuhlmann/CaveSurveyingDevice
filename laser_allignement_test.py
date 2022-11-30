import numpy as np

tilt_arr = [np.deg2rad(20),np.deg2rad(230)]
central_point = np.mean(tilt_arr) # or fit a circle through the points?

correction_dir = abs(tilt_arr-central_point)/(tilt_arr-central_point)
print("Correction dir:", correction_dir)

alpha = np.mean(np.abs(tilt_arr-central_point))
beta = np.pi/2 - alpha/2
gamma = np.pi-beta

l_0 = 0.1
l_1 = 20
l_2 = 2*l_0 * np.cos(alpha/2)

eeta = np.arcsin((l_2/l_1) * np.sin(gamma))
phi = np.pi - gamma - eeta
theta = np.pi - phi - beta

correction_vector = np.array([
                                [np.sin(theta)*np.cos(phi)],
                                [np.sin(theta)*np.sin(phi)],
                                [np.cos(theta)]
                            ])
print(alpha+2*beta)
print(eeta+gamma+phi)
print(beta+phi+theta)
print("Alpha:",alpha)
print("Beta:",beta)
print("Gamma:",gamma)
print("Eeta:",eeta)
print("Theta:",theta)
print("Phi",phi)
print("Opposite theta:",np.pi/2-theta)