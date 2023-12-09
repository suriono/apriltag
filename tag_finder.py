import mycamera, apriltag, cv2, math
import numpy as np
import dt_apriltags
from transforms3d.euler import mat2euler

# ======================== Camera class==================================
class Detector:
   def __init__(self, tag_size):
      self.tag_size = tag_size
      self.camera_obj = mycamera.Camera()
      options = apriltag.DetectorOptions(families="tag36h11") 
      self.detector = apriltag.Detector(options)
      self.dt_detector = dt_apriltags.Detector( searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],nthreads=3, quad_decimate=2, quad_sigma=0.4, refine_edges=1, decode_sharpening=1, debug=0)
      if self.camera_obj.start_reader():
         self.img = self.camera_obj.image
         self.camera_width,self.camera_height = self.img.shape[1],self.img.shape[0]

   # --------------------------------------------------------
   def capture_Camera(self):
      self.camera_obj.read()
      self.camera_obj.read()
      self.camera_obj.read()
      self.camera_obj.read()
      if self.camera_obj.start_reader():
         self.img = self.camera_obj.image
   # --------------------------------------------------------
   def release_Camera(self):
      self.camera_obj.release()

   # --------------------------------------------------------
   def getPose(self):
      self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
      self.dt_results = self.dt_detector.detect(self.gray,True,self.camera_obj.camera_params, self.tag_size)

      self.Yaw, self.Pitch, self.Roll,self.Translation = [],[],[],[]
      for result in self.dt_results:
         radian, degree = self.get_Euler(result.pose_R)
         #print("R matrix: ", result.pose_t)
         X,Y,Z = result.pose_t*1000.0
         self.X, self.Y, self.Z = X[0],Y[0],Z[0]
         position = (X[0],Y[0],Z[0])
         self.Pitch.append(degree[0])
         self.Yaw.append(degree[1])
         self.Roll.append(degree[2])
         self.Translation.append(position)
         self.tag_family = result.tag_family.decode("utf-8")
         self.tag_id = result.tag_id
         print("TAG FAMILY: ", self.tag_family, " tag id: ", self.tag_id)

#      self.getCamera_Pose(self.gray)
      return (len(self.dt_results)>0)

   # --------------------------------------------------------
   def getCamera_Pose(self):
      results = self.detector.detect(self.gray)
      for result in results:
         self.pose, e0, e1 = self.detector.detection_pose(result, self.camera_obj.camera_params)
#         self.draw_Cube(1)
         np_pose = np.array(self.pose)
         # Find where the camera is if the tag is at the origin
         camera_pose = np.linalg.inv(self.pose) # relative to the ta
      #   camera_pose = invert(self.pose) # relative to the ta
         camera_pose[0][3] *= self.tag_size * 1000
         camera_pose[1][3] *= self.tag_size * 1000
         camera_pose[2][3] *= self.tag_size * 1000
         self.camera_X = camera_pose[0][3]
         self.camera_Y = camera_pose[1][3] 
         self.camera_Z = camera_pose[2][3] 
#         print("Relative to Camera:\n", camera_pose)

   # --------------------------------------------------------
   def get_Euler(self, Rmatrix):
      radian = np.array(mat2euler(Rmatrix[0:3][0:3], 'sxyz')) 
      degree = np.array(np.rad2deg(mat2euler(Rmatrix[0:3][0:3], 'sxyz'))) 
      # Pitch, Yaw, and Roll
      return radian, degree

   # --------------------------------------------------------
   def get_All_Tag_Info(self):
      X,Y,Z = self.X, self.Y, self.Z
      yaw,pitch,roll = self.Yaw[0],self.Pitch[0],self.Roll[0]
      return self.tag_family, self.tag_id, X,Y,Z, yaw,pitch,roll

   # --------------------------------------------------------
   def get_Destination(self, loc):
#      print("Destination: ", loc)
      dx = loc[0] - self.Translation[0][0]
      dy = loc[1] - self.Translation[0][1]
      radian = math.atan2(dx,dy)
#      self.draw_Arrow(radian)

   # --------------------------------------------------------
   def showImage(self, image, length, scale=1):
      resize_picture = cv2.resize(image,(image.shape[1]*scale, image.shape[0]*scale))
      #cv2.imshow('img', image)
      cv2.imshow('img', resize_picture)
      keypress = cv2.waitKey(length)
      return keypress

   # --------------------------------------------------------
   def destroyAllWindows(self):
      cv2.destroyAllWindows()

# ======================== End of Camera class=========================


#camera_obj = mycamera.Camera()
#print (camera_obj.camera)
#print (camera_obj.camera_info)
#camera_obj.start_reader()
