import matplotlib.pyplot as plt 
import numpy as np

f = open("comparison_house.txt", "r")
lines = f.readlines()

max_x_list = []
max_y_list = []
max_theta_list = []

true_x_list = []
true_y_list = []
true_theta_list = [] 


for line in lines:
	line = line.split(" ")
	l_type, x, y, theta = line[0], float(line[1]), float(line[2]), float(line[3]) 
	if (l_type == "max"):
		max_x_list.append(x)
		max_y_list.append(y)
		max_theta_list.append(theta)
	else:
		true_x_list.append(x)
		true_y_list.append(y)
		true_theta_list.append(theta)
	# print(l_type, x, y,  theta)
	# print(line)

max_x_list = np.array(max_x_list)
max_y_list = np.array(max_y_list)
max_theta_list = np.array(max_theta_list)

true_x_list = np.array(true_x_list)
true_y_list = np.array(true_y_list)
true_theta_list = np.array(true_theta_list)

# Make plots
steps = [i for i in range(len(true_x_list))]

# x
# plt.plot(steps, max_x_list, label="particle x")
# plt.plot(steps, true_x_list, label="true x")
# plt.xlabel("steps")
# plt.ylabel("x (m)")
# plt.legend()
# plt.show()

# dx
# dx = np.abs(max_x_list - true_x_list)
# plt.plot(steps, dx, label="Absolute Error x")
# plt.xlabel("steps")
# plt.ylabel("dx (m)")
# plt.legend()
# plt.show()

# dx percent 
# dx = np.abs(max_x_list - true_x_list)
# dx_percent = dx / true_x_list * 100
# plt.plot(steps, dx_percent, label="Percentage Error x")
# plt.xlabel("steps")
# plt.ylabel("dx (%)")
# plt.legend()
# plt.show()



# y
# plt.plot(steps, max_y_list, label="particle y")
# plt.plot(steps, true_y_list, label="true y")
# plt.xlabel("steps")
# plt.ylabel("y (m)")
# plt.legend()
# plt.show()

# dy
# dy = np.abs(max_y_list - true_y_list)
# plt.plot(steps, dy, label="Absolute Error y")
# plt.xlabel("steps")
# plt.ylabel("dy (m)")
# plt.legend()
# plt.show()

# dy percent 
# dy = np.abs(max_y_list - true_y_list)
# dy_percent = dy / true_y_list * 100
# plt.plot(steps, dy_percent, label="Percentage Error y")
# plt.xlabel("steps")
# plt.ylabel("dy (%)")
# plt.legend()
# plt.show()


# theta
# plt.plot(steps, max_theta_list, label="particle theta")
# plt.plot(steps, true_theta_list, label="true theta")
# plt.legend()
# plt.xlabel("steps")
# plt.ylabel("theta (rad)")
# plt.show()

# dtheta
dtheta = np.abs(max_theta_list - true_theta_list)
for i, dt in enumerate(dtheta):
	if dt >= np.pi:
		while (dt >= np.pi):
			dt -= 2*np.pi
	dtheta[i] = dt

plt.plot(steps, dtheta, label="Absolute Error theta")
plt.xlabel("steps")
plt.ylabel("dtheta (rad)")
plt.legend()
plt.show()

# dtheta percent 
# dtheta = np.abs(max_theta_list - true_theta_list)
# dtheta_percent = dtheta / (true_theta_list+1e-6) * 100
# plt.plot(steps, dtheta_percent, label="Percentage Error theta")
# plt.xlabel("steps")
# plt.ylabel("dtheta (%)")
# plt.legend()
# plt.show()