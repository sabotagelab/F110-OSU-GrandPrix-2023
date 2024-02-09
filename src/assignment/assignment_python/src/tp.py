
import numpy
from scipy.ndimage.filters import uniform_filter1d

x = [1, 2, 3, 4, 5, 6, 7]
N = 3
# cumsum, moving_aves = [0], []

# for i, x in enumerate(mylist, 1):
#     cumsum.append(cumsum[i-1] + x)
#     if i>=N:
#         moving_ave = (cumsum[i] - cumsum[i-N])/N
#         #can do stuff with moving_ave here
#         moving_aves.append(moving_ave)

# print(moving_aves)

# def running_mean(x, N):
#     cumsum = numpy.cumsum(numpy.insert(x, 0, 0)) 
#     return (cumsum[N:] - cumsum[:-N]) / float(N)

# ret = running_mean(x, N)
# print(ret)

y = uniform_filter1d(x, size=N)
print(y)