import tag_finder, location, drawing
import json
import numpy as np

fjson = open("tag_transform.json")
tag_transform = json.load(fjson)
fjson.close()

# ======================== Camera class==================================

def Run_eachPressKey(tagfinder_obj, draw_obj):
   tagfinder_obj.capture_Camera()
   if not tagfinder_obj.getPose(): 
      print("===== No tag found")
      return 0
#     raise SystemExit("!!! Error: no tag found")

   #translate = tagfinder_obj.dt_results[0].pose_t
   #tagfinder_obj.annotate_Image(X,Y,Z)

   tag_family = tagfinder_obj.tag_family
   tag_id     = tagfinder_obj.tag_id
   transform_t = np.array(tag_transform[tag_family][str(tag_id)]['tmatrix'])

   # tag from camera
   X,Y,Z = tagfinder_obj.X,tagfinder_obj.Y,tagfinder_obj.Z
   yaw,pitch,roll = tagfinder_obj.Yaw[0],tagfinder_obj.Pitch[0],tagfinder_obj.Roll[0]

   # camera from tag
   tagfinder_obj.getCamera_Pose()
   cam_X,cam_Y,cam_Z = tagfinder_obj.camera_X,tagfinder_obj.camera_Y,tagfinder_obj.camera_Z
   cam_position = np.array([ cam_X, cam_Y, cam_Z ])

   cam_transform = np.matmul(transform_t , cam_position)
   print("Camera transform: ", cam_transform)

   draw_obj.img = tagfinder_obj.img
   draw_obj.annotate_Image(10,30,X,Y,Z,yaw,pitch,roll,(250,0,120))
   draw_obj.annotate_Image(10,140,cam_X,cam_Y,cam_Z,yaw,pitch,roll,(0,255,0))
   draw_obj.draw_Cube()

   #X,Y,Z = translate[0][0],translate[1][0], translate[2][0]
   #print(" ==== Translation: (", X,",",Y,",",Z,")")
   radian, degree = tagfinder_obj.get_Euler(tagfinder_obj.dt_results[0].pose_R)
   #print(" ==== Euler degree (Yaw,Pitch,Roll):", degree)
   #print(" ==== Pose:\n", tagfinder_obj.pose)

   tagfinder_obj.get_Destination((-47,20))
   return  tagfinder_obj.showImage(tagfinder_obj.img,100,2)

# ============================================


tagfinder_obj = tag_finder.Detector(0.047)
draw_obj      = drawing.Draw(tagfinder_obj)

keypress = ord('*') # arbitrary

while keypress != ord('a'):
   keypress = Run_eachPressKey(tagfinder_obj, draw_obj)
tagfinder_obj.release_Camera()
tagfinder_obj.destroyAllWindows()
