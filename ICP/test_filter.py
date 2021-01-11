import numpy as np
from scipy.spatial.distance import cdist

dat=np.array([[1,1],[1.1,1],[1.3,1],[1.5,1],[1.6,1]])
dat=np.transpose(dat)
dist_thres=0.15
num_points = dat.shape[1]
is_key_point=[False for i in range(num_points)]
print("Before filter, total {} points".format(num_points))
dat_t = np.transpose(dat)
dist = cdist(dat_t, dat_t, 'euclidean')


i=0
for it in range(dat_t.shape[0]):
    len_next = list(np.where(dist[i, i:] > dist_thres))[0]
    is_key_point[i] = True
    if len_next.shape[0]>0:
        i = len_next[0] + i
    else:
        break

dat_filt = dat[:, is_key_point]
print("After filter, total {} points".format(dat_filt.shape[1]))
print(dat_filt)