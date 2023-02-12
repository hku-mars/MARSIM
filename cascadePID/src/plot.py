import matplotlib.pyplot as plt
import numpy as np

file = '../data/log_0.txt'
a = np.loadtxt(file)

pos_des_x = a[:,0]
pos_des_y = a[:,1]
pos_des_z = a[:,2]
pos_fb_x = a[:,3]
pos_fb_y = a[:,4]
pos_fb_z = a[:,5]
vel_des_x = a[:,6]
vel_des_y = a[:,7]
vel_des_z = a[:,8]
vel_fb_x = a[:,9]
vel_fb_y = a[:,10]
vel_fb_z = a[:,11]
att_des_x = a[:,12]
att_des_y = a[:,13]
att_des_z = a[:,14]
att_fb_x = a[:,15]
att_fb_y = a[:,16]
att_fb_z = a[:,17]
att_error_x = a[:,18]
att_error_y = a[:,19]
att_error_z = a[:,20]
acc_des_x = a[:,21]
acc_des_y = a[:,22]
acc_des_z = a[:,23]
torque_des_x = a[:,24]
torque_des_y = a[:,25]
torque_des_z = a[:,26]
z_body_vec_des_x = a[:,27]
z_body_vec_des_y = a[:,28]
z_body_vec_des_z = a[:,29]
RPM_1 = a[:,30]
RPM_2 = a[:,31]
RPM_3 = a[:,32]
RPM_4 = a[:,33]

# plt.plot(att_des_x)
# plt.plot(att_fb_x)
# plt.plot(att_des_y)
# plt.plot(att_fb_y)
# plt.plot(att_des_z)
# plt.plot(att_fb_z)
# plt.show()

# #position error
plt.plot(pos_des_x,'-')
plt.plot(pos_fb_x,'*-')
plt.plot(pos_des_y,'-')
plt.plot(pos_fb_y,'*-')
plt.plot(pos_des_z,'-')
plt.plot(pos_fb_z,'*-')
plt.show()

# # #vel error
# plt.plot(vel_des_x,'-')
# plt.plot(vel_fb_x,'*-')
# plt.plot(vel_des_y,'-')
# plt.plot(vel_fb_y,'*-')
# plt.plot(vel_des_z,'-')
# plt.plot(vel_fb_z,'*-')
# plt.show()

# # #acc error
# plt.plot(acc_des_x,'-')
# plt.plot(acc_des_y,'-')
# plt.plot(acc_des_z,'-')
# plt.show()

#torque desire
# plt.plot(torque_des_x,'r-')
# plt.plot(torque_des_y,'g-')
# plt.plot(torque_des_z,'b-')
# plt.show()

# # RPM output
# plt.plot(RPM_1,'r-')
# plt.plot(RPM_2,'g-')
# plt.plot(RPM_3,'b-')
# plt.plot(RPM_4,'-')
# plt.show()

#z body desire
# plt.plot(z_body_vec_des_x,'r-')
# plt.plot(z_body_vec_des_y,'g-')
# plt.plot(z_body_vec_des_z,'b-')
plt.show()

#att error
plt.plot(att_error_x*57.3,'r-')
plt.plot(att_error_y*57.3,'g-')
# plt.plot(att_error_z*57.3,'o-')
# plt.plot(att_des_x*57.3)
# plt.plot(att_fb_x*57.3,'*-')
# plt.plot(att_des_y*57.3)
# plt.plot(att_fb_y*57.3,'*-')
plt.plot(att_des_z*57.3)
plt.plot(att_fb_z*57.3,'o-')
plt.show()