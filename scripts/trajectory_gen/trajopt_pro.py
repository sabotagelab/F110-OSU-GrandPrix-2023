# -*- coding: utf-8 -*-
"""trajopt_pro.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1ubjOYWqwMK3e1wCh42DIcJnTLgJ3H-vH
"""

import csv, math
from matplotlib import pyplot as plt
import numpy as np
from sklearn.cluster import KMeans
from scipy.spatial import cKDTree
from os.path import expanduser

def dist(p, q):
    "Return the Euclidean distance between points p and q."
    return math.hypot(p[0] - q[0], p[1] - q[1])

def simplify_points(points, r):
    """Return a maximal list of elements of points such that no pairs of
    points in the result have distance less than r.
    """
    result = []
    for p in points:
        if all(dist(p, q) >= r for q in result):
            result.append(p)
    return result

def order_pts(pts, starter):
    """no pairs of points in the result have distance less than r.
    """
    r = 0.1#0.08
    # print(pts)
    #new_pts = simplify_points(pts, r)
    # print(new_pts)
    new_pts = pts
    ordered_pts = []
    ind = starter #try zero if not working
    for i in range(len(new_pts)-1):
        p = new_pts.pop(ind)
        ctree = cKDTree(new_pts)
        dis, ind =  ctree.query(p, k=1)
        if dis < 10*r:
          ordered_pts.append(p)
    #ordered_pts.append(new_pts[-1])
    ordered_pts.append(ordered_pts[0])
    ordered_pts = np.array(ordered_pts)
    ox = ordered_pts[:,0].flatten()
    oy = ordered_pts[:,1].flatten()
    #smx, smy = smoothener(ordered_pts)
    return ox, oy #, smx, smy

# def tf_points(x, y):
#     #look out for the role of resolution here 0.5 or 0.05 thingy
#     rmx = (x*0.05 - 14.9)*-1
#     rmy = (y*0.05 - 13.8)
#     return rmx, rmy

# def map_inflate(f):
#     size = 6
#     with open(f, 'rb') as pgmf:
#         im = plt.imread(pgmf)
#     data = np.array(im)
#     # data = np.pad(dat, size, pad_with)
#     w, h = np.shape(data)
#     data = data.flatten().tolist()
#     new_data = []
#     for i in data:
#         if i == 0: k = 100
#         elif i > 206: k = 0
#         else: k = -1
#         new_data.append(k)
#     map_ = np.reshape(np.array(new_data), (w, h))
#     map2 = np.reshape(map_, (w, h)) *1
#     k = int(size/2)
#     b = np.ones((size, size))*100
#     for x in range(w):
#         for y in range(h):
#             if (map_[x][y] > 0):
#                 map2[x-k:x+k,y-k:y+k]= b           
#     xs = []
#     ys = []
#     for x in range(w):
#         for y in range(h):
#             if (map2[x][y] > 0):
#                 rmx, rmy = tf_points(x, y)
#                 #print(x, y, rmx, rmy)
#                 xs.append(rmx)
#                 ys.append(rmy)
#                 #map2[x-k:x+k,y-k:y+k]= b
#     return xs, ys

# def map_inflate(f):
#     size = 6
#     with open(f, 'rb') as pgmf:
#         im = plt.imread(pgmf)
#     data = np.array(im)
#     # data = np.pad(dat, size, pad_with)
#     w, h = np.shape(data)
#     data = data.flatten().tolist()
#     new_data = []
#     for i in data:
#         if i == 0: k = 100
#         elif i > 206: k = 0
#         else: k = -1
#         new_data.append(k)
#     map_ = np.reshape(np.array(new_data), (w, h))
#     xs = []
#     ys = []
#     for x in range(w):
#         for y in range(h):
#             if (map_[x][y] > 0):
#                 rmx = (x*0.05 - 11.7)*-1
#                 rmy = (y*0.05 - 1)
#                 #print(x, y, rmx, rmy)
#                 xs.append(rmx)
#                 ys.append(rmy)
#                 #map2[x-k:x+k,y-k:y+k]= b
#     return xs, ys

def file_reader(file):
  with open (file , 'r') as p:
      point_data = list(csv.reader(p))
      p.close()

  traj_ = []
  xs = []
  ys = []
  for i in range(len(point_data)):
      traj_.append((float(point_data[i][0]), float(point_data[i][1])))
      xs.append(float(point_data[i][0]))
      ys.append(float(point_data[i][1]))
  return xs, ys, traj_

# fileleft = "/content/drive/MyDrive/f1ten-scripts/csv/leftwp.csv"
# filemid = "/content/drive/MyDrive/f1ten-scripts/waypoints.csv"
# fileright = "/content/drive/MyDrive/f1ten-scripts/csv/rightwp.csv"
# mapfile = "/content/drive/MyDrive/f1ten-scripts/map.pgm"
home = expanduser('~')
fileleft = home + "/f1ten-scripts/csv/leftwp.csv"
filemid = home + "/f1ten-scripts/csv/refwp.csv"
fileright = home + "/f1ten-scripts/csv/rightwp.csv"
mapfile = home + "/f1ten-scripts//map.pgm"
#ox, oy = map_inflate(mapfile) #inflated map
rx_, ry_, rts = file_reader(fileright) #right border
rx, ry = order_pts(rts, -1)
lx_, ly_, lts = file_reader(fileleft) # left border
lx, ly = order_pts(lts, -1)
xs, ys, pts = file_reader(filemid) #old traj
plt.plot(xs, ys, marker="o", color="black")
plt.plot(rx, ry, marker="s")
plt.plot(lx, ly, marker="s")
#plt.scatter(oy, ox)
# for i in range(len(ox)):
#   plt.gcf().gca().add_artist(plt.Circle((oy[i], ox[i]), 0.05))
plt.show()

from scipy.interpolate import splprep, splev
tr, ur = splprep([rx, ry], s=0)
unew = np.arange(0, 1.00, 0.001)
new_points_right = splev(unew, tr)
plt.plot(new_points_right[0], new_points_right[1], marker='o')

tl, ul = splprep([lx, ly], s=0)
unew2 = np.arange(0, 1.00, 0.001)
new_points_left = splev(unew2, tl)
plt.plot(new_points_left[0], new_points_left[1], marker='o')
# for i in range(len(ox)):
#   plt.gcf().gca().add_artist(plt.Circle((oy[i], ox[i]), 0.05))
print("before pairing")
plt.show()

left_points = [(new_points_left[0][i], new_points_left[1][i]) for i in range(len(new_points_left[0]))]
right_points = [(new_points_right[0][i], new_points_right[1][i]) for i in range(len(new_points_right[0]))]
cid = 0
pairs = []
ctree = cKDTree(left_points)
p = right_points.pop()
dis, ind =  ctree.query(p, 1)
pairs.append((p, left_points[ind]))
taken = [ind]
for i in right_points:
  if len(left_points)-2 > ind:
    if ind+1 in taken:
      ind -= 1
    elif ind-1 in taken:
      ind += 1
    else:
      dist1 = dist(i, left_points[ind + 1])
      dist2 = dist(i, left_points[ind - 1])
      if dist1 <= dist2: ind += 1
      else: ind -= 1
    pairs.append((i, left_points[ind]))
    taken.append(ind)
  elif len(pairs) != len(left_points):
    ind = 1

# cid = 0
# pairs = []
# taken = []
# ctree = cKDTree(left_points)
# p = right_points.pop(0)
# dis, ind =  ctree.query(p, 1)
# pairs.append((p, left_points[ind]))
# _ = left_points.pop(ind)
# for i in range(len(right_points)):
#   ctree = cKDTree(left_points)
#   p = right_points.pop(0)
#   dis, ind =  ctree.query(p, 1)
#   if ind not in taken:
#     pairs.append((p, left_points[ind]))
#     taken.append(ind)
#     _ = left_points.pop(ind)
    

midsx = []
midsy = []
midsx2 = []
midsy2 = []
midsx3 = []
midsy3 = []
wt, wt1, wt2 = 0.4, 0.8, 0.2
for i in pairs:
  plt.plot((i[0][0],i[1][0]), (i[0][1], i[1][1]))
  midsx.append((wt*i[0][0] + (1-wt)*i[1][0]))
  midsy.append((wt*i[0][1] + (1-wt)*i[1][1]))
  midsx2.append((wt1*i[0][0] + (1-wt1)*i[1][0]))
  midsy2.append((wt1*i[0][1] + (1-wt1)*i[1][1]))
  midsx3.append((wt2*i[0][0] + (1-wt2)*i[1][0]))
  midsy3.append((wt2*i[0][1] + (1-wt2)*i[1][1]))
  

print("after pairing")
plt.plot(new_points_right[0], new_points_right[1], marker='s')
plt.plot(new_points_left[0], new_points_left[1], marker='o')
plt.plot(midsx, midsy, color = "black", marker='s')
plt.plot(midsx2, midsy2, color = "red", marker='s')
plt.plot(midsx3, midsy3, color = "yellow", marker='s')

plt.show()


# wt = 0.5 #wt indicates weight to left 0.5 is midpoint
# midsx = (1-wt)*(final_left[0]) + wt*final_right[0]
# midsy =  (1-wt)*(final_left[1]) + wt*final_right[1]

# wt = 0.8 #wt indicates weight to left 0.5 is midpoint
# midsx2 = (1-wt)*(final_left[0]) + wt*final_right[0]
# midsy2 =  (1-wt)*(final_left[1]) + wt*final_right[1]
plt.plot(midsx, midsy, color = "black", marker='s')
# plt.plot(midsx2, midsy2, color = "red", marker='s')
# plt.plot(midsx3, midsy3, color = "yellow", marker='s')
# wt = 0.3 #wt indicates weight to left 0.3 is closer to inner lane
# midsx3 = (1-wt)*(final_left[0]) + wt*final_right[0]
# midsy3 =  (1-wt)*(final_left[1]) + wt*final_right[1]



# plt.plot(midsx, midsy, color = "black", marker='s')

# plt.show()





# for i in range(len(ox)):
#   plt.gcf().gca().add_artist(plt.Circle((oy[i], ox[i]), 0.05))

# plt.show()
# csv generator for traj opt
import csv
file = open(home + "/f1ten-scripts/opt.csv", 'w')
file.write('# x_m,y_m,w_tr_right_m,w_tr_left_m\n')
for i in pairs:
  mix, miy = 0.5*(i[0][0] + i[1][0]), 0.5*(i[0][1] + i[1][1])
  dist1 = min(0.8*dist(i[0], (mix, miy)), 0.22)
  file.write('%f, %f, %f, %f\n' % (10*mix, 10*miy, dist1*10, dist1*10))
file.close()
print('Goodbye')

# #generate parallel tracks
# import csv

f = home + "/f1ten-scripts/ways.csv"
file = open(f , 'w')
for i in range(len(midsx)):
  file.write('%f, %f, %f, %f, %f, %f\n' % (midsx[i], midsy[i], midsx2[i], midsy2[i], midsx3[i], midsy3[i]))
file.close()
print('Goodbye')

# file = home + "/f1ten-scripts/traj.csv"
# xt, yt, pts = file_reader(file)
# plt.plot(xt, yt)
# for i in range(len(ox)):
#   plt.gcf().gca().add_artist(plt.Circle((oy[i], ox[i]), 0.05))
# plt.show()