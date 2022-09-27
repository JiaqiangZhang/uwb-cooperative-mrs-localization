from cProfile import label
from distutils.log import error
import numpy as np
import matplotlib.pyplot as plt


# pos_u = np.loadtxt('./pos/pos_u.csv')
# pos_u_v = np.loadtxt('./pos/pos_u_v.csv')
# pos_uv = np.loadtxt('./pos/pos_uv.csv')

errors_uwb = np.loadtxt('./errors/error_uwb.csv')
errors_uwb_1 = np.loadtxt('./errors/error_u_3.csv')
errors_uwb_2 = np.loadtxt('./errors/error_u_13.csv')
errors_uwb_3 = np.loadtxt('./errors/error_u_134.csv')
# errors_u = np.loadtxt('./errors/error_u.csv')
# errors_u_v = np.loadtxt('./errors/error_u_v.csv')
# errors_uv = np.loadtxt('./errors/error_uv.csv')

errors = [errors_uwb, errors_uwb_1, errors_uwb_2, errors_uwb_3]

plt.style.use('seaborn-whitegrid')

# fig = plt.figure()
# ax = plt.axes()

# print(errors_uwb.size)
# print(errors_uwb_1.size)
# print(errors_uwb_2.size)
# print(errors_uwb_3.size)


# plt.boxplot(errors)

fig = plt.figure(figsize =(10, 7))
ax = fig.add_subplot(111)

# Creating axes instance
# bp = ax.boxplot(errors, patch_artist = True,
#     notch ='True', vert = 0)


colors = ['#0000FF', '#00FF00',
  '#FFFF00', '#FF00FF', '#32a846']

labels = ['real uwb ranges', '1 uwb ranges meas',
     '2 uwb ranges meas','3 uwb range meas', '3 uwb + 1 vis range meas']

for  i, e in enumerate(errors):
  plt.plot(np.arange(len(e)), e, color=colors[i], label=labels[i])
 
# x-axis labels
# ax.set_yticklabels(['real uwb ranges', '1 uwb ranges meas',
#      '2 uwb ranges meas','three uwb range meas', ])

plt.legend()

# Adding title
plt.title("uwb range error line plot")

# Removing top axes and right axes
# ticks
# ax.get_xaxis().tick_bottom()
# ax.get_yaxis().tick_left()
 
# show plot
plt.show()



# x = np.arange(errors_uwb.shape[0])
# plt.plot(x, errors_uwb, label='only uwb ranges')

# y = np.arange(errors_u.shape[0])
# plt.plot(y, errors_u, label='uwb ranges with pf')

# z = np.arange(errors_u_v.shape[0])
# plt.plot(z, errors_u_v, label='uwb ranges integrating spatial (one meas) with pf')

# j = np.arange(errors_uv.shape[0])
# plt.plot(j, errors_uv, label='uwb ranges integrating spatial (two meas) with pf')

# plt.legend()
# plt.show()
# print(errors[0])