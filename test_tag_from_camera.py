import tag_finder, location, drawing

# ======================== Camera class==================================

def Run_eachPressKey(tagfinder_obj):
   tagfinder_obj.capture_Camera()
   #if not tagfinder_obj.getPose(): raise SystemExit("!!! Error: no tag found")
   if not tagfinder_obj.getPose(): 
      print("===== No tag found")
      return 0
#     raise SystemExit("!!! Error: no tag found")

   X,Y,Z = tagfinder_obj.X,tagfinder_obj.Y,tagfinder_obj.Z
   yaw,pitch,roll = tagfinder_obj.Yaw[0],tagfinder_obj.Pitch[0],tagfinder_obj.Roll[0]

   draw_obj.img = tagfinder_obj.img
   draw_obj.annotate_Image(10,30,X,Y,Z,yaw,pitch,roll)
   #draw_obj.draw_Bounding_box()
   draw_obj.draw_Cube()

#   tagfinder_obj.annotate_Image()
#   translate = tagfinder_obj.dt_results[0].pose_t
#   X,Y,Z = translate[0][0],translate[1][0], translate[2][0]
   print(" ==== Translation: (", X,",",Y,",",Z,")")
   radian, degree = tagfinder_obj.get_Euler(tagfinder_obj.dt_results[0].pose_R)
   print(" ==== Euler degree (Yaw,Pitch,Roll):", degree)
#   print(" ==== Pose:\n", tagfinder_obj.pose)

   tagfinder_obj.get_Destination((-47,20))
   return  tagfinder_obj.showImage(tagfinder_obj.img,100, 2)


#print (camera_obj.camera)
#print (camera_obj.camera_info)
#camera_obj.start_reader()
#locate_obj = location.Locator(tagfinder_obj.pose)

# ============================================

tagfinder_obj = tag_finder.Detector(0.047)
draw_obj      = drawing.Draw(tagfinder_obj)

keypress = ord('*') # arbitrary

while keypress != ord('a'):
   keypress = Run_eachPressKey(tagfinder_obj)

tagfinder_obj.release_Camera()
tagfinder_obj.destroyAllWindows()
