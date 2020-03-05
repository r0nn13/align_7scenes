import numpy as np
import cv2 
import matplotlib.pyplot as plt

# % The maximum depth used, in meters.
maxDepth = 10

# % RGB Intrinsic Parameters
fx_rgb = 5.1885790117450188e+02
fy_rgb = 5.1946961112127485e+02
cx_rgb = 3.2558244941119034e+02
cy_rgb = 2.5373616633400465e+02

# % RGB Distortion Parameters
k1_rgb =  2.0796615318809061e-01
k2_rgb = -5.8613825163911781e-01
p1_rgb = 7.2231363135888329e-04
p2_rgb = 1.0479627195765181e-03
k3_rgb = 4.9856986684705107e-01

# % Depth Intrinsic Parameters
fx_d = 5.8262448167737955e+02
fy_d = 5.8269103270988637e+02
cx_d = 3.1304475870804731e+02
cy_d = 2.3844389626620386e+02

# % RGB Distortion Parameters
k1_d = -9.9897236553084481e-02
k2_d = 3.9065324602765344e-01
p1_d = 1.9290592870229277e-03
p2_d = -1.9422022475975055e-03
k3_d = -5.1031725053400578e-01

# % Rotation
R = -np.array([ [9.9997798940829263e-01, 5.0518419386157446e-03, 
    4.3011152014118693e-03],[ -5.0359919480810989e-03, 
    9.9998051861143999e-01, -3.6879781309514218e-03],[ 
    -4.3196624923060242e-03, 3.6662365748484798e-03, 
    9.9998394948385538e-01 ]])

R = np.reshape(R, [3, 3])
R = np.linalg.inv(R.T)

# % 3D Translation
t_x = 7.5031875059141302e-02
t_z = 2.9342312935846411e-02
t_y = 6.6238747008330102e-04

# % Parameters for making depth absolute.
depthParam1 = 351.3
depthParam2 = 1092.5

def depth_rel2depth_abs(imgDepthOrig):
    H, W = imgDepthOrig.shape

    imgDepthAbs = imgDepthOrig
    
    imgDepthAbs[imgDepthAbs > maxDepth] = maxDepth
    imgDepthAbs[imgDepthAbs < 0] = 0

    return imgDepthAbs

def depth_plane2depth_world(imgDepth):
    H, W = imgDepth.shape

    xx, yy = np.meshgrid(range(0,W), range(0,H))

    x3 = (xx - cx_d) * imgDepth / fx_d
    y3 = (yy - cy_d) * imgDepth / fy_d
    z3 = imgDepth
    
    points3d = np.array([np.reshape(x3,[-1]), -np.reshape(y3,[-1]), np.reshape(z3,[-1])])
    
    return points3d

def depth_world2rgb_world(points3d):
    T = np.array([[t_x], [t_z], [t_y]])
   
    points3d = R @ points3d + T @ np.ones((1, points3d.shape[1]))
    points3d = points3d.T
    
    return points3d

# def rgb_plane2rgb_world(imgDepth):
#     H, W = imgDepth.shape

#     xx, yy = np.meshgrid(range(1,W), range(1,H))

#     x3 = (xx - cx_rgb) * imgDepth / fx_rgb
#     y3 = (yy - cy_rgb) * imgDepth / fy_rgb
#     z3 = imgDepth
    
#     points3d = [x3[:], -y3[:], z3[:]]
#     return points3d

def rgb_world2rgb_plane(points3d):
    X_world = points3d[:,0]
    Y_world = points3d[:,1]
    Z_world = points3d[:,2]
    
    X_plane = (X_world * fx_rgb / Z_world) + cx_rgb
    Y_plane = (Y_world * fy_rgb / Z_world) + cy_rgb

    return X_plane, Y_plane

def rgb2depth(rgb):
    H,W,_ = rgb.shape
    scale = fx_rgb/fx_d
    start_x = int((1.0-scale)/2.0 * W- 0*(cx_d - cx_rgb))
    end_x = int(W-(1.0-scale)/2.0 * W- 0*(cx_d - cx_rgb))
    start_y = int((1.0-scale)/2.0 * H  + 0*(cy_d - cy_rgb)-30.0)
    end_y = int(W-(1.0-scale)/2.0 * H + 0*(cy_d - cy_rgb)-30.0)

    print(start_y,end_y)
    out = cv2.resize(rgb[start_y:end_y, start_x:end_x,:], (W,H))
    
    return out

def reprojectRGB(filename = '/media/ronnie/EVO_Data/microsoft7scenes/chess/seq-01/frame-000000.depth.png'):
    imgName = filename.split('/')[-1]
    imgDepth = cv2.imread(filename,cv2.IMREAD_UNCHANGED).astype(np.float)/1000.0
    imgDepth = np.squeeze(imgDepth)
    H,W = imgDepth.shape
    
    depth2 = depth_rel2depth_abs(imgDepth)
    points3d = depth_plane2depth_world(depth2)
    points3d = depth_world2rgb_world(points3d)

    xProj, yProj = rgb_world2rgb_plane(points3d)

    xProj = np.round(xProj).astype(np.int)
    yProj = np.round(yProj).astype(np.int)
    
    goodInds = np.argwhere(np.logical_and(np.logical_and(np.logical_and(xProj > 0 ,  xProj < W) , yProj > 0) , yProj < H))

    depthOut = np.zeros_like(imgDepth)

    imgDepthFlat = np.reshape(imgDepth,[-1])
    
    # for idx in goodInds:
    #     depthOut[yProj[idx], xProj[idx]] = imgDepthFlat[idx]

    # depthOut = np.flipud(depthOut)

    #cv2.imwrite('frame-000000.depth.png', (depthOut*1000.0).astype(np.uint16))

    rgb = cv2.imread(filename.replace('depth','color'))[:,:,::-1]   
    #rgbOut = rgb2depth(rgb)
    rgb = np.flipud(rgb)

    rgbOut = np.zeros_like(rgb)
    rgbOut = np.reshape(rgbOut,[-1,3])

    #rgbFlat = np.reshape(rgb,[-1,3])
    for ii in range(len(xProj)):
        if xProj[ii] < W and yProj[ii] < H and xProj[ii]>0 and yProj[ii]>0:
            rgbOut[ii,:] = rgb[yProj[ii],xProj[ii],:]
    rgbOut = np.reshape(rgbOut,[H,W,3])
    #rgbOut = np.flipud(rgbOut)

    cv2.imwrite(imgName.replace('depth','color'), rgbOut[:,:,::-1])
    
    return rgbOut

import glob
if __name__ == '__main__':
    files = glob.glob('/media/ronnie/EVO_Data/microsoft7scenes/chess/seq-01/*depth.png')
    files.sort()

    for filename in files:
        print(filename)
        reprojectRGB(filename)
