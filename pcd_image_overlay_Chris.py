import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
# import cv2 as cv
import cv2
import io # Needed for the buffer functionality
from PIL import Image
import copy
import math
# Own files
import transformations
import params
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------- #
#     Create a plot and .jpg with front camera image and pointcloud overlay    #
# ---------------------------------------------------------------------------- #
def create_point_cloud_image_overlay(pcd_points2 , cam_image):

    # -------------------- Lidar camera projection right here -------------------- #
    ptc_xyz_lidar = pcd_points2
    numpoints = ptc_xyz_lidar.shape[0]
    assert(ptc_xyz_lidar.shape[1] == 3), "PointCloud_lidar is not N x 3"

    # --------------------------- Applying the Rotation -------------------------- # Alt + x
    # ---------------------------------------------------------------------------- # Alt + Y
    # ---------------------------------------------------------------------------- # Alt + shift + x
    #                                      sdf                                     #
    # ---------------------------------------------------------------------------- #
    translation_luminar_front2_flc = np.array([(params.translation[0]),
                                                (params.translation[1]), 
                                                (params.translation[2])])
                                                
    RotMat_luminar_front2_flc = np.array([(params.rotation[0]), 
                                            (params.rotation[1]), 
                                            (params.rotation[2]),
                                            (params.rotation[3])])
    translation_luminar_front2_flc2 = np.array([(params.usr_translation[0]),
                                                (params.usr_translation[1]), 
                                                (params.usr_translation[2])])
    RotMat_luminar_front2_flc2= np.array([params.usr_rotation[0], 
                                            params.usr_rotation[1], 
                                            params.usr_rotation[2]])
    # RotMat_luminar_front2_flc = np.array(transformations.deg2RotMat(RotMat_luminar_front2_flc))
    RotMat_luminar_front2_flc2 = np.array(transformations.deg2RotMat(RotMat_luminar_front2_flc2))
    RotMat_luminar_front2_flc = R.from_quat(RotMat_luminar_front2_flc).as_matrix()
    # RotMat_luminar_front2_flc2 = R.from_quat(RotMat_luminar_front2_flc2).as_matrix()

    translation_luminar_front2_flc = translation_luminar_front2_flc + translation_luminar_front2_flc2 
    translation_luminar_front2_flc_final = translation_luminar_front2_flc
    print(translation_luminar_front2_flc_final) # Final Translation

    translation_luminar_front2_flc = np.tile(translation_luminar_front2_flc.reshape((3, 1)), numpoints)
    translation_luminar_front2_flc = (RotMat_luminar_front2_flc2 @ translation_luminar_front2_flc)
    RotMat_luminar_front2_flc = RotMat_luminar_front2_flc2 @ RotMat_luminar_front2_flc

    ptc_xyz_camera = RotMat_luminar_front2_flc @ ptc_xyz_lidar.T + translation_luminar_front2_flc

    print(RotMat_luminar_front2_flc) # Final Rotation

    # ---------------------------------------------------------------------------- #
    #                     Writing the Calibrations into a file                     #
    # ---------------------------------------------------------------------------- #
    # Save the array to a file
    np.savetxt(f'calibration/Calibrated_translation_{params.cam_type}.txt', translation_luminar_front2_flc2, fmt='%.4f')
    # Save the array to a file
    np.savetxt(f'calibration/Calibrated_rotation_{params.cam_type}.txt', RotMat_luminar_front2_flc2, fmt='%.4f')
    # with open('calibtaton.txt','wb') as f:
    #     new_translation = str(translation_luminar_front2_flc_final) + '\n'
    #     new_rotation = str(RotMat_luminar_front2_flc)
    #     f.write(new_translation + new_rotation)
    #     f.close()


    ptc_xyz_camera = ptc_xyz_camera.T

    # ------------------------- Applying the Camera Info ------------------------- #
    # camera_info = np.array([[1732.571708*0.5 * float(params.fx), 0.000000, 549.797164*0.5 + params.usr_cx_scale], 
    #                         [0.000000, 1731.274561*0.5 * float(params.fx), 295.484988*0.5 + params.usr_cy_scale], 
    #                         [0.000000, 0.000000, 1.000000]])
    camera_info=params.mp
    # camera_info = np.array([[1732.571708 * float(params.fx), 0.000000, 549.797164 + params.usr_cx_scale], 
    #                     [0.000000, 1731.274561 * float(params.fx), 295.484988 + params.usr_cy_scale], 
    #                     [0.000000, 0.000000, 1.000000]])
    ptc_xyz_camera = ptc_xyz_camera.T
    ptc_xyz_camera = camera_info @ ptc_xyz_camera
    ptc_xyz_camera = ptc_xyz_camera.T

    # ------------------------------ Do the Division ----------------------------- #
    ptc_z_camera = ptc_xyz_camera[:, 2]
    ptc_xyz_camera[:,:2] = np.divide(ptc_xyz_camera[:,:2], ptc_z_camera.reshape(ptc_xyz_camera.shape[0], 1))

    # -------------------- Plotting everything into the Image -------------------- #
    # print("image_width, height:", image.shape)
    image_undistorted = cv2.undistort(cam_image, camera_info, np.array([-0.272455, 0.268395, -0.005054, 0.000391, 0.000000]))
    border_size=300
    image_undistorted=cv2.copyMakeBorder(image_undistorted,border_size,border_size,border_size,border_size,cv2.BORDER_CONSTANT,None,0)
    
    
    z_min=np.min(ptc_z_camera)
    z_range=np.max(ptc_z_camera)-z_min
    ptc_z_camera=(ptc_z_camera-z_min)*255/z_range
    ptc_z_camera=ptc_z_camera.astype(np.uint8)
    color=cv2.applyColorMap(ptc_z_camera[:,np.newaxis],cv2.COLORMAP_HSV)
    r=color.shape[0]
    for j in range(r):
        i=ptc_xyz_camera[j]
        c=color[np.newaxis,np.newaxis,j,0]
        a = int(np.floor(i[0]) + border_size)
        b = int(np.floor(i[1]) + border_size)
        if a>0 and b>0:
            try:
                image_undistorted[b-1:b+2,a-1:a+2] = c
            except:
                continue

    return image_undistorted
    


