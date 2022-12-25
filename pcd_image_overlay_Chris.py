import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
# import cv2 as cv
import cv2
import io # Needed for the buffer functionality
from PIL import Image
from ai import cs # Coordinate transformation package
import copy
import math
# Own files
import transformations
import params

# ---------------------------------------------------------------------------- #
#     Create a plot and .jpg with front camera image and pointcloud overlay    #
# ---------------------------------------------------------------------------- #
def create_point_cloud_image_overlay(pcd_points2 , cam_image):

    # -------------------- Lidar camera projection right here -------------------- #
    # ptc_numpy_record = pointcloud2_to_array(self.point_cloud_msg)
    # ptc_xyz_lidar = get_xyz_points(ptc_numpy_record)
    ptc_xyz_lidar = pcd_points2
    numpoints = ptc_xyz_lidar.shape[0]
    assert(ptc_xyz_lidar.shape[1] == 3), "PointCloud_lidar is not N x 3"
    # np.save(os.path.join(os.getcwd(), "pointcloud_raw.txt"), ptc_numpy)
    # np.savetxt(os.path.join(os.getcwd(), "pointcloud_raw.txt"), ptc_numpy, fmt="%d", delimiter=",")
    # print("Min Values:", np.min(ptc_xyz_lidar, axis=0))
    # print("Max Values: ", np.max(ptc_xyz_lidar, axis=0))

    # --------------------------- Applying the Rotation -------------------------- # Alt + x
    # ---------------------------------------------------------------------------- # Alt + Y
    # ---------------------------------------------------------------------------- # Alt + shift + x
    #                                      sdf                                     #
    # ---------------------------------------------------------------------------- #
    translation_luminar_front2_flc = np.array([(params.translation[0]+params.usr_translation[0]-1.302+1.354-0.3),
                                                (params.translation[1]+params.usr_translation[1]-1.25+1.250-0.054), 
                                                (params.translation[2]+params.usr_translation[2]+13.33-9.167+4.905)])
    RotMat_luminar_front2_flc = np.array([(params.rotation[0]+params.usr_rotation[0]+6.77), 
                                            (params.rotation[1]+params.usr_rotation[1]+1.61), 
                                            (params.rotation[2]+params.usr_rotation[2]-5.93)])
    RotMat_luminar_front2_flc = np.array(transformations.deg2RotMat(RotMat_luminar_front2_flc))
    # RotMat_luminar_front2_flc = np.array([[0.0, -1.0,  0.0],
    #                                     [0.0,  0.0, -1.0],
    #                                     [1.0,  0.0,  0.0]])
    # translation_luminar_front2_flc = np.array([0.121, -0.026, 0.007])
    print(translation_luminar_front2_flc)
    print(RotMat_luminar_front2_flc)
    translation_luminar_front2_flc = np.tile(translation_luminar_front2_flc.reshape((3, 1)), numpoints)
    assert(translation_luminar_front2_flc.shape == (3, numpoints)), "Translation is not 3 x N"
    ptc_xyz_camera = RotMat_luminar_front2_flc @ ptc_xyz_lidar.T + translation_luminar_front2_flc # This is correct
    ptc_xyz_camera = ptc_xyz_camera.T
    assert(ptc_xyz_camera.shape == (numpoints, 3)), "PointCloud_camera is not N x 3"


    # ------------------------- Applying the Camera Info ------------------------- #
    camera_info = np.array([[1732.571708, 0.000000, 549.797164/2 + 92 -20 -35+ params.usr_cx_scale], 
                            [0.000000, 1731.274561, 295.484988/2 + 100 -41+3+ params.usr_cy_scale], 
                            [0.000000, 0.000000, 1.000000]])
    ptc_xyz_camera = ptc_xyz_camera.T
    ptc_xyz_camera = camera_info @ ptc_xyz_camera
    ptc_xyz_camera = ptc_xyz_camera.T

    # ------------------------------ Do the Division ----------------------------- #
    ptc_z_camera = ptc_xyz_camera[:, 2]
    ptc_xyz_camera = np.divide(ptc_xyz_camera, ptc_z_camera.reshape(ptc_xyz_camera.shape[0], 1))

    # -------------------- Plotting everything into the Image -------------------- #
    # print("image_width, height:", image.shape)
    image_undistorted = cv2.undistort(cam_image, camera_info, np.array([-0.272455, 0.268395, -0.005054, 0.000391, 0.000000]))
    border_size=300
    image_undistorted=cv2.copyMakeBorder(image_undistorted,border_size,border_size,border_size,border_size,cv2.BORDER_CONSTANT,None,0)

    for i in ptc_xyz_camera:
        a = int(np.floor(i[0]) + border_size)
        b = int(np.floor(i[1]) + border_size)
        try:
            image_undistorted[b][a] = [0, 0, 255]
            image_undistorted[b+1][a] = [0, 0, 255]
            image_undistorted[b][a+1] = [0, 0, 255]
            image_undistorted[b+1][a+1] = [0, 0, 255]
            image_undistorted[b-1][a] = [0, 0, 255]
            image_undistorted[b][a-1] = [0, 0, 255]
            image_undistorted[b-1][a-1] = [0, 0, 255]
        except:
            continue
        # image_undistorted = cv2.circle(image_undistorted, (int(i[0]+border_size), int(i[1])+border_size), 1, (0, 0, 255), 2)

    return image_undistorted
    


