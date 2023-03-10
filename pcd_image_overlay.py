import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
import cv2 as cv
import io # Needed for the buffer functionality
from PIL import Image
from ai import cs # Coordinate transformation package
import copy
import math
# Own files
import transformations
import params

# ---------------------------------------------------------------------------- #
# ------------------- Convert a matplotlib figure to a jpg ------------------- #
# ------------------- image in openCV format and return it ------------------- #
# ---------------------------------------------------------------------------- #
def fig2img(fig):
    buf = io.BytesIO() # The buffer is defined as binary (needed for images)
    plt.savefig(buf, format = 'jpg', dpi = params.dpi) # Save figure as jpg to buffer (Pufferspeicher)
    buf.seek(0) # Needed in order to make cv.imread start reading at beginning of buf
    pil_img = copy.deepcopy(Image.open(buf)).convert('RGB') # Read from buffer as PIL image (read in as openCV image does not work)
    img = np.array(pil_img) # Convert PIL image format to openCV format
    img = img[:, :, ::-1].copy() # Transformations
    buf.close()
    return img

# ---------------------------------------------------------------------------- #
#     Create a plot and .jpg with front camera image and pointcloud overlay    #
# ---------------------------------------------------------------------------- #
def create_point_cloud_image_overlay(pcd_points, pcd_points2 , cam_image):

    # ---------------------------------------------------------------------------- #
    #                              Initial Calibration                             #
    # ---------------------------------------------------------------------------- #
    # Transformation of point cloud points into camera coordinate system
    # Adding user input to the initial transformation
    translation_tot = ((params.translation[0]+params.usr_translation[0]),(params.translation[1]+params.usr_translation[1]), (params.translation[2]+params.usr_translation[2]))
    rotation_tot = ((params.rotation[0]+params.usr_rotation[0]), (params.rotation[1]+params.usr_rotation[1]), (params.rotation[2]+params.usr_rotation[2]))
    calib_matrix = transformations.matrix(rotation_tot, translation_tot) # Convert to 4x4 g matrix
    
    # Checking if Pointcloud Exists
    try:
        dimensions = pcd_points.shape
        dimensions = (dimensions[0], 3)
    except:
        print("No image or pointcloud has been received. Make sure to run the image and pcd publisher before starting the main.py file and check for correct topic names.")
        print("The overlay publisher has been stopped.")
        exit()

    # ---------------------------------------------------------------------------- #
    #                  Transforming Points from Lidar2Camera frame                 #
    # ---------------------------------------------------------------------------- #
    pcd_points_cam = np.zeros((dimensions))
    # print("Shape of PCD Points: ", pcd_points.shape)
    # print("Shape of PCD Points: ", pcd_points2.shape)

    # ---------------------------------------------------------------------------- #
    # ------- Applying Transformation Matrix to Each Point from Lidar Frame ------ # (Checked)
    # ---------------------------------------------------------------------------- #
    for i in range (0, dimensions[0]):
        # print(pcd_points2[i,:])
        # print(pcd_points[i,:])
        # Only the first 3 items are "x", "y", "z" coordinates in respective order ([i,:3])
        # print("Calibration  Matrix: \n", np.array(calib_matrix))
        # print("pcd_points[i,:]: \n", pcd_points[i,:])
        # print("transformations.transform(pcd_points[i,:], calib_matrix) : ", transformations.transform(pcd_points[i,:], calib_matrix) )
        pcd_points_cam [i,:] = transformations.transform(pcd_points[i,:], calib_matrix) 
        # print("pcd_points_cam [i,:3]: \n", pcd_points_cam [i,:3])
    
    
    # ---------------------------------------------------------------------------- #
    #         Transformation of cartesian ptc array to spherical ptc array         #
    # ---------------------------------------------------------------------------- #
    spherical_pcd_points = np.zeros(dimensions)
    points_out_of_cam_perspective = []

    for i in range (0, dimensions[0]):
        # Cartesian --> spherical
        spherical_pcd_points[i, 0], spherical_pcd_points[i, 1], spherical_pcd_points[i, 2] = \
            cs.cart2sp(pcd_points_cam[i,0],pcd_points_cam[i,1], pcd_points_cam[i,2])  #xyz --> r theta phi
        # Degrees --> radians
        spherical_pcd_points[i, 1] = math.degrees(spherical_pcd_points[i, 1])
        spherical_pcd_points[i, 2] = math.degrees(spherical_pcd_points[i, 2])

        # ---------------------------------------------------------------------------- #
        #              Getting points which are out of FOV of the camera              #
        # ---------------------------------------------------------------------------- #
        if spherical_pcd_points[i,2] < -params.horizontal_FOV_cam/2 or spherical_pcd_points [i,2]>params.horizontal_FOV_cam/2:
            points_out_of_cam_perspective.append(i)
        elif spherical_pcd_points[i,1] <-(params.vertical_FOV_cam/2) or spherical_pcd_points[i,1] >(params.vertical_FOV_cam/2): 
            points_out_of_cam_perspective.append(i)
        elif spherical_pcd_points[i,0]<params.pcd_dis_min or spherical_pcd_points [i,0]>params.pcd_dis_max:
            points_out_of_cam_perspective.append(i)

    # ---------------------------------------------------------------------------- #
    #            Deleting of points which are out of FOV of front camera           #
    # ---------------------------------------------------------------------------- #
    print("spherical_pcd_points.shape: ", spherical_pcd_points.shape)
    print("points_out_of_cam_perspective.shape: ", np.array(points_out_of_cam_perspective).shape)
    spherical_pcd_points_cam_perspective = np.delete(spherical_pcd_points, points_out_of_cam_perspective, 0) 
    print("spherical_pcd_points_cam_perspective.shape: ", spherical_pcd_points_cam_perspective.shape)

    # ---------------------------------------------------------------------------- #
    #         Transformation of spherical ptc array to cartesian ptc array         #
    # ---------------------------------------------------------------------------- #
    new_dimensions = spherical_pcd_points_cam_perspective.shape 
    cartesian_pcd_points_cam_perspective = np.zeros((new_dimensions))
    for i in range (0, new_dimensions[0]):
        # Dergees --> radians
        spherical_pcd_points_cam_perspective[i, 1] = math.radians(spherical_pcd_points_cam_perspective[i, 1])
        spherical_pcd_points_cam_perspective[i, 2] = math.radians(spherical_pcd_points_cam_perspective[i, 2])
        # Spherical --> cartesian
        cartesian_pcd_points_cam_perspective[i, 0], cartesian_pcd_points_cam_perspective[i, 1], cartesian_pcd_points_cam_perspective[i, 2] = \
            cs.sp2cart(spherical_pcd_points_cam_perspective[i,0],spherical_pcd_points_cam_perspective[i,1], spherical_pcd_points_cam_perspective[i,2])  #xyz --> r theta phi


    # cv.projectPoints requires rotation and translation vector, but this has already been done
    rvec = np.array([0,0,0], np.float) # Rotation vector
    tvec = np.array([0,0,0], np.float) # Translation vector
    cx = params.width_pixels/2 # Center of image horizontal
    cy = params.height_pixels/2 # Center of image vertical
    
    # Calculation of fu and fv in pixel units
    if params.sim_data == "yes":    # Using projection matrix 
        fu = params.mp[0,0]*cx  
        fv = params.mp[1,1]*cy
    else:                           # Using camera matrix 
        fu = params.mp[0,0]*params.width_pixels/params.width_sensorsize
        fv = params.mp[1,1]*params.height_pixels/params.height_sensorsize

    # New arrangement of the colums of the point cloud matrix to fit it for the projectpoints() function
    cartesian_pcd_points_cam_perspective_new_oder = np.zeros(new_dimensions)
    cartesian_pcd_points_cam_perspective_new_oder[:,0] = cartesian_pcd_points_cam_perspective[:,1]
    cartesian_pcd_points_cam_perspective_new_oder[:,1] = cartesian_pcd_points_cam_perspective[:,2]
    cartesian_pcd_points_cam_perspective_new_oder[:,2] = -cartesian_pcd_points_cam_perspective[:,0]
    
    # Initialize camera matrix
    cameraMatrix =  np.array([[fu,0,cx],[0,fv,cy],[0,0,1]], np.float)
    
    # ----------------------------------- Error ---------------------------------- #
    # Checking Matrix here
    print("cartesian_pcd_points_cam_perspective_new_oder Shape: ", cartesian_pcd_points_cam_perspective_new_oder)
    print("rvec Shape: ", rvec.shape)
    print("tvec Shape: ", tvec.shape)
    print("cameraMatrix Shape: ", cameraMatrix.shape)

    # ---------------------------------------------------------------------------- #
    # ------------- 2D points which can be projected to camera image ------------- #
    # ---------------------------------------------------------------------------- #
    projected_3D_points_to_2D_project_points, jacobian_pcd_3d_2d = \
        cv.projectPoints(cartesian_pcd_points_cam_perspective_new_oder, rvec, tvec, cameraMatrix, None)
    # Changing size of the array
    projected_3D_points_to_2D_project_points = projected_3D_points_to_2D_project_points.reshape(-1,2)
    
     # ---------------------------------------------------------------------------- #
     #           Create plot with image and overlaying point cloud points           #
     # ---------------------------------------------------------------------------- #
    try:    
        implot = plt.imshow(cam_image)
    except:
        print("No image or pointcloud has been received. Make sure to run the image and pcd publisher before starting the main.py file and check for correct topic names.")
        print("The overlay publisher has been stopped.")
        exit()
        
    scat = plt.scatter(projected_3D_points_to_2D_project_points[:,0], projected_3D_points_to_2D_project_points[:,1],params.point_size, \
        c = spherical_pcd_points_cam_perspective[:,0], cmap = cm.gist_ncar)
    plt.axis("off")
    image_pointcloud_overlay_front = fig2img(scat)
    implot.remove()
    scat.remove()

    return image_pointcloud_overlay_front
    


