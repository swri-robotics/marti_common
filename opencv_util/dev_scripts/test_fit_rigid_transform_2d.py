import random
import numpy as np
from opencv_util import fit_rigid_transform_2d, apply_transform
from matplotlib import pyplot

def generate_random_points(num_points, x_range, y_range):
    x = np.random.uniform(x_range[0], x_range[1], (num_points, 1))
    y = np.random.uniform(y_range[0], y_range[1], (num_points, 1))
    return np.hstack([x,y])    

def move_points(points, dx, dy, angle, noise):
    c = np.cos(angle/180.0*np.pi)
    s = np.sin(angle/180.0*np.pi)
    
    transform = np.array(((c, -s, dx), (s, c, dy)), np.float32)
    pts2 = apply_transform(transform, points)

    if noise > 0.0:
        pts2 += np.random.uniform(0, noise, pts2.shape)
    return pts2

def draw_correspondence(pts1, pts2):
    assert pts1.shape[0] == pts2.shape[0]
    
    pyplot.plot(pts1[:,0], pts1[:,1], 'ro')
    pyplot.plot(pts2[:,0], pts2[:,1], 'bo')

    for i in range(pts1.shape[0]):
        pyplot.plot([pts1[i,0], pts2[i,0]], [pts1[i,1], pts2[i,1]])

    
pts1 = generate_random_points(10, (-10, 10), (-10, 10))

## pts1 = np.array((
##     ( 2.0,  1.0),
##     ( 2.0, -1.0),
##     (-2.0, -1.0),
##     (-2.0,  1.0)), np.float32)

pts2 = move_points(pts1, 2.0, 5.0, 10.0, 2.0)


np.set_printoptions(precision=4, suppress=True, linewidth=300)

transform = fit_rigid_transform_2d(pts1, pts2)
print "transform = "
print transform
print '='*60

pts3 = apply_transform(transform, pts1)

pyplot.figure('unaligned')
draw_correspondence(pts1, pts2)

pyplot.figure('aligned')
draw_correspondence(pts3, pts2)

pyplot.show()


