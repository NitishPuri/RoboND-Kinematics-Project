import matplotlib.pyplot as plt
import numpy as np
import time
import sys

if(len(sys.argv) == 1):
    print ("Error file name required.")
    exit(1)

errors = np.loadtxt(sys.argv[1])
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.bar(range(0, len(errors)), errors)
ax.set_xlabel("poses")
ax.set_ylabel("errors")
# ax.set_ylim(0, 10)
fig.savefig("{}.png".format(sys.argv[1]))

    # plt.savefig("{} errors.png".format(time.ctime()))

# fig, ax = plt.subplots()
# ax.bar(x_s, y_s)
# ax.set_xlabel("poses")
# ax.set_ylabel("errors")
# ax.set_ylim(0, 10)
# filename =  
# plt.savefig("{} errors.png".format(time.ctime()))
# plt.show()