#!/usr/bin/env python

import sys
import math
from matplotlib import pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray, Marker
import visualization_msgs
from scipy.spatial.distance import cdist
from scipy.spatial import Delaunay
from scipy.spatial import cKDTree
from os.path import expanduser

def dist(p, q):
    "Return the Euclidean distance between points p and q."
    return math.hypot(p[0] - q[0], p[1] - q[1])

def running_mean(x, N):
    #x is array of points and N is window size
    cumsum = np.cumsum(np.insert(x, 0, 0)) 
    return (cumsum[N:] - cumsum[:-N]) / float(N)

def simplify_points(points, r):
    """Return a maximal list of elements of points such that no pairs of
    points in the result have distance less than r.
    """
    result = []
    print("digging")
    for p in points:
        if all(dist(p, q) >= r for q in result):
            result.append(p)
    return result

def filter_small_triangles(points, tri, coeff):
    """
    Filter out triangles that have an edge > coeff * median(edge)
    Inputs:
        tri: scipy.spatial.Delaunay object
        coeff: triangles with an edge > coeff * median(edge) will be filtered out
    Outputs:
        valid_slice: boolean array that selects "normal" triangles
    """
    edge_lengths = np.zeros(tri.vertices.shape)
    seen = {}
    # loop over triangles
    for i, vertex in enumerate(tri.vertices):
        # loop over edges
        for j in range(3):
            id0 = vertex[j]
            id1 = vertex[(j + 1) % 3]

            # avoid calculating twice for non-border edges
            if (id0,id1) in seen:
                edge_lengths[i, j] = seen[(id0,id1)]
            else:    
                edge_lengths[i, j] = np.linalg.norm(points[id1] - points[id0])

                seen[(id0,id1)] = edge_lengths[i, j]

    valid_slice = np.all(edge_lengths > coeff , axis=1)
    kid = np.where(valid_slice)[0]
    return kid

def get_edges(sims, tol, pts):
    #filter out sall edges
    edges = []
    for  i in sims:
        d1 = dist(pts[i[0]], pts[i[1]])
        d2 = dist(pts[i[1]], pts[i[2]])
        d3 = dist(pts[i[0]], pts[i[2]])
        if d1 > tol: edges.append([i[0], i[1]])
        if d2 > tol: edges.append([i[1], i[2]])
        if d3 > tol: edges.append([i[2], i[0]])
    edgs = np.array(edges)
    mids = get_mids(edgs, pts)
    return edgs, mids

def order_pts(pts):
    """no pairs of points in the result have distance less than r.
    """
    r = 1
    new_pts = simplify_points(pts, r)
    ordered_pts = []
    ind = 0
    for i in range(len(new_pts)-1):
        p = new_pts.pop(ind)
        ctree = cKDTree(new_pts)
        dis, ind =  ctree.query(p, 1)
        ordered_pts.append(p)
    ordered_pts.append(new_pts[-1])
    #ordered_pts.append(ordered_pts[0])
    ordered_pts = np.array(ordered_pts)
    ox = ordered_pts[:,0].flatten()
    oy = ordered_pts[:,1].flatten()
    #smx, smy = smoothener(ordered_pts)
    return ox, oy #, smx, smy
    

def get_mids(edges, pts):
    #returns midpoints of edges
    mids = []
    for e in edges:
        mids.append([((pts[e[0]][0] + pts[e[1]][0])/2), ((pts[e[0]][1] + pts[e[1]][1])/2)])
    return mids

def smoothener(pts):
    from scipy.interpolate import splprep, splev
    mids = np.array(pts)
    x = mids[:,0].flatten()
    y = mids[:,1].flatten()
    tck, u = splprep([x, y], s = 0.7, k=3) 
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_new, y_new = splev(u_new, tck, der=0)
    return x_new, y_new

def triangle_solver(rmx, rmy):
    factor = 1.25
    for i in range(len(rmx)-5):
        d1 = dist((rmx[i], rmy[i]), (rmx[i+1], rmy[i+1]) )
        d2 = dist((rmx[i+2], rmy[i+2]), (rmx[i+1], rmy[i+1])) 
        d3 = dist((rmx[i+3], rmy[i+3]), (rmx[i+2], rmy[i+2]))
        d4 =  dist((rmx[i+3], rmy[i+3]), (rmx[i], rmy[i]))
        if (d1 + d2 + d3 ) > d4*factor:
            rmx = np.delete(rmx, i+1)
            rmy = np.delete(rmy, i+1)
        if i > len(rmx) - 4:
            return rmx, rmy
    
    return rmx, rmy

def tf_points(x, y):
    #look out for the role of reolution here 0.5 or 0.05 thingy
    rmx = (x*0.05 - 11.7)*-1
    rmy = (y*0.05 - 0.95)
    return rmx, rmy


def get_mean_traj(ox, oy, N):
    #calculates running mean and closes the loop
    rmx = running_mean(ox, N)
    rmy = running_mean(oy, N)
    # for i in range(int(N/2)-1):
    #     rmx = np.append(rmx, ox[-(int(N/2) - i - 1 )])
    #     rmy = np.append(rmy, oy[-(int(N/2) - i - 1)])
    # rmx = np.append(rmx, rmx[0])
    # rmy = np.append(rmy, rmy[0])
    rmx, rmy = triangle_solver(rmx, rmy)
    rmx,rmy = triangle_solver(rmx, rmy)

    
    return rmx, rmy

def publish_plan(xs, ys, publisher):
    msg = Path()
    msg.header.frame_id = "/map"
    msg.header.stamp = rospy.Time.now()
    for i in range(len(xs)):
        pose = PoseStamped()
        pose.pose.position.x = float(xs[i]) 
        pose.pose.position.y = float(ys[i]) 
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1
        msg.poses.append(pose)

    rospy.loginfo("Publishing Plan...")
    publisher.publish(msg) 

def pad_with(vector, pad_width, iaxis, kwargs):
    pad_value = kwargs.get('padder', -1)
    vector[:pad_width[0]] = pad_value
    vector[-pad_width[1]:] = pad_value

def map_inflate(f):
    size = 6
    with open(f, 'rb') as pgmf:
        im = plt.imread(pgmf)
    data = np.array(im)
    # data = np.pad(dat, size, pad_with)
    w, h = np.shape(data)
    data = data.flatten().tolist()
    new_data = []
    for i in data:
        if i == 0: k = 100
        elif i > 206: k = 0
        else: k = -1
        new_data.append(k)
    map_ = np.reshape(np.array(new_data), (w, h))
    map2 = np.reshape(map_, (w, h)) *1
    k = int(size/2)
    b = np.ones((size, size))*100
    for x in range(w):
        for y in range(h):
            if (map_[x][y] > 0):
                map2[x-k:x+k,y-k:y+k]= b           
    #rx, ry, bx, by = map_proc(map2, w, h)
    #return rx, ry, bx, by
    map_proc(map2, w, h)

def map_proc(map_, w, h):
    points = []
    map_ = np.reshape(map_, (w, h))
    for x in range(w):
        for y in range(h):
            if (map_[x][y] > 0) or (map_[x][y] < 0):
                points.append([x, y])
    points = np.array(points)
    tri = Delaunay(points)
    #Filter out triangles that have an edge > coeff * median(edge)
    print("filtering smaller triangles")
    coeff = 3
    large_triangle_ids = filter_small_triangles(points, tri, coeff)
    subset_tri = np.take(tri.simplices, large_triangle_ids, axis =0) # this doesn't preserve tri, effectively just a renaming
    edge_length_min = 20
    #Filter out small edges
    edges, mids = get_edges(subset_tri, edge_length_min, points)
    ox, oy = order_pts(mids)
    N = 3
    print("Smoothening trajectory")
    rmx, rmy = get_mean_traj(ox, oy, N)
    #rmx,rmy = get_mean_traj(rmx, rmy, N-3)
    rmx, rmy = tf_points(rmx, rmy)
    x = points[:,0].flatten()
    y = points[:,1].flatten()
    box, boy = tf_points(np.array(x[edges.T]).flatten(), np.array(y[edges.T]).flatten())
    #return rmx, rmy, box, boy
    plt.plot(ox, oy, linestyle="--")
    plt.triplot(points[:,0], points[:,1], subset_tri)
    plt.show()
    plt.plot(rmx, rmy, linestyle='-', color='r', marker='s')
    plt.scatter(box, boy, marker='s') 
    plt.show()


if __name__ == "__main__":
    home = expanduser('~')
    f = home + "/f1ten-scripts/map.pgm"
    #map_inflate(f)
    rospy.init_node("map_proc")
    map_pub = rospy.Publisher("/map_inflate", OccupancyGrid, queue_size=1, latch=True)
    waypoint_publisher = rospy.Publisher("/trajectory", Path, queue_size=1, latch=True)
    side_publisher = rospy.Publisher("/side", Path, queue_size=1, latch=True)
    map_inflate(f)
    #rmx, rmy, box, boy =  map_inflate(f)
    #publish_plan(rmy, rmx, waypoint_publisher)
   # publish_plan(boy, box, side_publisher)
    rospy.sleep(5)
    rospy.spin()