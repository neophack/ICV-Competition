import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import sys

a=np.loadtxt("out.txt")
f=plt.figure()
ax=f.add_subplot(111)

i=1
while i<len(sys.argv):
    x, y, l, w=[float(_) for _ in sys.argv[i:i+4]]
    ax.add_patch(patches.Rectangle((x-l/2,y-w/2),l,w))
    i=i+4
	
plt.plot(a[0,0],a[0,1],'ro')
plt.arrow(a[0,0], a[0,1], 1.0 * math.cos(a[0,2]), 1.0 * math.sin(a[0,2]),
                  fc="r", ec="k", head_width=0.5, head_length=0.5)
plt.plot(a[len(a)-1,0],a[len(a)-1,1],'yo')
plt.arrow(a[len(a)-1,0], a[len(a)-1,1], 1.0 * math.cos(a[len(a)-1,2]), 1.0 * math.sin(a[len(a)-1,2]),fc="r", ec="k", head_width=0.5, head_length=0.5)
plt.plot(a[:,0],a[:,1],'-')
plt.show()
