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
      gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
      self.dt_results = self.dt_detector.detect(gray,True,self.camera_obj.camera_params, self.tag_size)

      self.Yaw, self.Pitch, self.Roll,self.Translation = [],[],[],[]
      for result in self.dt_results:
         radian, degree = self.get_Euler(result.pose_R)
         #print("R matrix: ", result.pose_t)
         X,Y,Z = result.pose_t*1000.0
         position = (X[0],Y[0],Z[0])
         self.Pitch.append(degree[0])
         self.Yaw.append(degree[1])
         self.Roll.append(degree[2])
         self.Translation.append(position)
         self.tag_family = result.tag_family.decode("utf-8")
         self.tag_id = result.tag_id
         print("TAG FAMILY: ", self.tag_family, " tag id: ", self.tag_id)

         self.getCamera_Pose(gray)
      return (len(self.dt_results)>0)

   # --------------------------------------------------------
   def getCamera_Pose(self,gray):
      results = self.detector.detect(gray)
      for result in results:
         self.pose, e0, e1 = self.detector.detection_pose(result, self.camera_obj.camera_params)
#         self.draw_Bounding_box(result)
         self.draw_Cube(1, self.pose)
         np_pose = np.array(self.pose)
#         np_pose[0][3] *= self.tag_size
#         np_pose[1][3] *= self.tag_size
#         np_pose[2][3] *= self.tag_size
#         self.pose[0][3] *= self.tag_size
#         self.pose[1][3] += 0.5
#         self.pose[2][3] *= self.tag_size
         # Find where the camera is if the tag is at the origin
         camera_pose = np.linalg.inv(self.pose) # relative to the ta
      #   camera_pose = invert(self.pose) # relative to the ta
#         unitmat = np.matmul(self.pose,camera_pose)
         camera_pose[0][3] *= self.tag_size * 1000
         camera_pose[1][3] *= self.tag_size * 1000
         camera_pose[2][3] *= self.tag_size * 1000
#         print("relative camera pose: ", camera_pose)
         X_cam,Y_cam,Z_cam = self.pose[0][3]*1000, self.pose[1][3]*1000, self.pose[2][3]*1000
         print("Relative XYZ: ", X_cam,Y_cam,Z_cam)
         print("Relative Camera:\n", camera_pose)
#         print("Unity Matrix:\n", unitmat)

   # --------------------------------------------------------
   def get_Euler(self, Rmatrix):
      radian = np.array(mat2euler(Rmatrix[0:3][0:3], 'sxyz')) 
      degree = np.array(np.rad2deg(mat2euler(Rmatrix[0:3][0:3], 'sxyz'))) 
      # Pitch, Yaw, and Roll
      return radian, degree

   # --------------------------------------------------------
   def annotage_Image(self):
      text_translation = '(X,Y,Z) in mm:' + f'({self.Translation[0][0]:.0f}' + ',' + f'{self.Translation[0][1]:.0f}' + ',' + f'{self.Translation[0][2]:.0f})' 
      location_translation = (10,30)

      text_rotation = '(Yaw,Pitch,Roll):' + f'({self.Yaw[0]:.1f}' + ',' + f'{self.Pitch[0]:.1f}' + ',' + f'{self.Roll[0]:.1f})' 
      location_rotation = (10,80)

      font,location,fontScale = cv2.FONT_HERSHEY_SIMPLEX,(10, 30),1 
      color,thickness = (55, 0, 255),4 
      annote_img = cv2.putText(self.img, text_translation, 
                   location_translation, font, fontScale, color, 
                   thickness, cv2.LINE_AA)
      annote_img = cv2.putText(self.img, text_rotation, location_rotation,
                   font, fontScale, color, thickness, cv2.LINE_AA)
      return annote_img

   # --------------------------------------------------------
   def get_Destination(self, loc):
      print("Destination: ", loc)
      dx = loc[0] - self.Translation[0][0]
      dy = loc[1] - self.Translation[0][1]
      radian = math.atan2(dx,dy)
      self.draw_Arrow(radian)

   # --------------------------------------------------------
   def draw_Arrow(self, radian):
      arrow_length = self.camera_width / 5
      center = (int(self.camera_width-arrow_length*0.6),int(self.camera_height-arrow_length*0.6))
      x1 = int(-arrow_length*math.sin(radian)/2 + center[0])
      x2 = int(arrow_length*math.sin(radian)/2 + center[0])
      y1 = int(arrow_length*math.cos(radian)/2 + center[1])
      y2 = int(-arrow_length*math.cos(radian)/2 + center[1])
      image = cv2.arrowedLine(self.img, (x1,y1), (x2,y2),(0,255,0),15,tipLength=0.5)  
      image = cv2.circle(self.img,center, int(arrow_length*0.6), (0,255,255), 3)

   # --------------------------------------------------------
   def draw_Bounding_box(self, detect_result):
      (ptA, ptB, ptC, ptD) = detect_result.corners
      ptB = (int(ptB[0]), int(ptB[1]))
      ptC = (int(ptC[0]), int(ptC[1]))
      ptD = (int(ptD[0]), int(ptD[1]))
      ptA = (int(ptA[0]), int(ptA[1]))
      # draw the bounding box of the AprilTag detection
      cv2.line(self.img, ptA, ptB, (0, 255, 0), 2)
      cv2.line(self.img, ptB, ptC, (0, 255, 0), 2)
      cv2.line(self.img, ptC, ptD, (0, 255, 0), 2)
      cv2.line(self.img, ptD, ptA, (0, 255, 0), 2)

   # --------------------------------------------------------
   def draw_Cube(self, tag_size, pose, z_sign=1):
#      print("Cube: ", tag_size,pose)
      opoints =  np.array([-1, -1, 0, 1, -1, 0, 1,  1, 0,
                 -1,  1, 0, -1, -1, -2*z_sign, 1, -1, -2*z_sign,
                 1,  1, -2*z_sign, -1,  1, -2*z_sign,
                 ]).reshape(-1, 1, 3) * 0.5*tag_size
      edges = np.array([ 0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 1, 5,
              2, 6, 3, 7, 4, 5, 5, 6, 6, 7, 7, 4 ]).reshape(-1, 2)
      fx, fy, cx, cy = self.camera_obj.camera_params
      K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
      rvec, _ = cv2.Rodrigues(pose[:3,:3])
      tvec = pose[:3, 3]
      dcoeffs = np.zeros(5)
      ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
      ipoints = np.round(ipoints).astype(int)
      ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]
      for i, j in edges:
         cv2.line(self.img, ipoints[i], ipoints[j], (0, 0, 255), 3, 16)

   # --------------------------------------------------------
   def showImage(self, image, length):
      cv2.imshow('img', image)
      keypress = cv2.waitKey(length)
      return keypress

   # --------------------------------------------------------
   def destroyAllWindows(self):
      cv2.destroyAllWindows()

# ======================== End of Camera class=========================

# ============================ Invert 4x4 matrix ======================
def invert(m):
   m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33 = np.ravel(m)
   a2323 = m22 * m33 - m23 * m32
   a1323 = m21 * m33 - m23 * m31
   a1223 = m21 * m32 - m22 * m31
   a0323 = m20 * m33 - m23 * m30
   a0223 = m20 * m32 - m22 * m30
   a0123 = m20 * m31 - m21 * m30
   a2313 = m12 * m33 - m13 * m32
   a1313 = m11 * m33 - m13 * m31
   a1213 = m11 * m32 - m12 * m31
   a2312 = m12 * m23 - m13 * m22
   a1312 = m11 * m23 - m13 * m21
   a1212 = m11 * m22 - m12 * m21
   a0313 = m10 * m33 - m13 * m30
   a0213 = m10 * m32 - m12 * m30
   a0312 = m10 * m23 - m13 * m20
   a0212 = m10 * m22 - m12 * m20
   a0113 = m10 * m31 - m11 * m30
   a0112 = m10 * m21 - m11 * m20

   det = m00 * (m11 * a2323 - m12 * a1323 + m13 * a1223) \
	- m01 * (m10 * a2323 - m12 * a0323 + m13 * a0223) \
	+ m02 * (m10 * a1323 - m11 * a0323 + m13 * a0123) \
	- m03 * (m10 * a1223 - m11 * a0223 + m12 * a0123)
   det = 1 / det

   return np.array([ \
		det *  (m11 * a2323 - m12 * a1323 + m13 * a1223), \
		det * -(m01 * a2323 - m02 * a1323 + m03 * a1223), \
		det *  (m01 * a2313 - m02 * a1313 + m03 * a1213), \
		det * -(m01 * a2312 - m02 * a1312 + m03 * a1212), \
		det * -(m10 * a2323 - m12 * a0323 + m13 * a0223), \
		det *  (m00 * a2323 - m02 * a0323 + m03 * a0223), \
		det * -(m00 * a2313 - m02 * a0313 + m03 * a0213), \
		det *  (m00 * a2312 - m02 * a0312 + m03 * a0212), \
		det *  (m10 * a1323 - m11 * a0323 + m13 * a0123), \
		det * -(m00 * a1323 - m01 * a0323 + m03 * a0123), \
		det *  (m00 * a1313 - m01 * a0313 + m03 * a0113), \
		det * -(m00 * a1312 - m01 * a0312 + m03 * a0112), \
		det * -(m10 * a1223 - m11 * a0223 + m12 * a0123), \
		det *  (m00 * a1223 - m01 * a0223 + m02 * a0123), \
		det * -(m00 * a1213 - m01 * a0213 + m02 * a0113), \
		det *  (m00 * a1212 - m01 * a0212 + m02 * a0112)  \
	]).reshape(4, 4)

#camera_obj = mycamera.Camera()
#print (camera_obj.camera)
#print (camera_obj.camera_info)
#camera_obj.start_reader()
