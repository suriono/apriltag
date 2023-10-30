import cv2, math
import numpy as np

# --------------------------------------------------------
class Draw:

   def __init__(self):
      self.img = None

   # --------------------------------------------------------
   def annotate_Image(self,loc_x,loc_y,X,Y,Z,yaw,pitch,roll,color=(255,0,0)):
      text_translation = '(X,Y,Z) in mm:' + f'({X:.0f}' + ',' + f'{Y:.0f}' + ',' + f'{Z:.0f})' 
      location_translation = (loc_x,loc_y)

      text_rotation = '(Yaw,Pitch,Roll):' + f'({yaw:.1f}' + ',' + f'{pitch:.1f}' + ',' + f'{roll:.1f})' 
      location_rotation = (loc_x,loc_y+50)

      font,location,fontScale = cv2.FONT_HERSHEY_SIMPLEX,(10, 30),1 
      #color,thickness = (255, 0, 0),4 
      thickness = 4 
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

