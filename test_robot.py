import mySerial, tag_finder, location, drawing
import json,cv2

myserial      = mySerial.MySerial()
myserial.serOpen()

tagfinder_obj = tag_finder.Detector(0.047) # small tag
draw_obj      = drawing.Draw(tagfinder_obj)

keypress = ord('*')     # initialization


# ======================== Camera class==================================

def Run_eachPressKey(tagfinder_obj, draw_obj):
   tagfinder_obj.capture_Camera()
   if not tagfinder_obj.getPose():
      print("===== No tag found")
      return 0
#     raise SystemExit("!!! Error: no tag found")

#   transform_t = np.array(tag_transform[tag_family][str(tag_id)]['tmatrix'])

   # tag from camera
   tag_family, tag_id,X,Y,Z,yaw,pitch,roll = tagfinder_obj. get_All_Tag_Info()
   draw_obj.annotate_Image(10,30,X,Y,Z,yaw,pitch,roll,(250,0,120))
   draw_obj.draw_Arrow(roll*3.14/180.0)
#   draw_obj.draw_Bounding_box()
#   draw_obj.draw_Cube()

   # motor control
   # spin toward the tag first
   timeeach, waittime = 500,0
   if ( abs(roll) > 10 ):
      amp = str(4*int(abs(roll) % 180))
      theta = str(int(-90*roll/abs(roll)))
      cmd = '{"mag":' + amp + ', "theta":' + theta + ', "delay": ' + str(timeeach) + '}'
      print("Return msg: ", myserial.serWrite_and_Read(cmd, waittime))
   elif abs(Y) > 5:
      amp = str(min(2*int(Y),200))
      theta = "180" if Y > 0 else "0"
      cmd = '{"mag":' + amp + ', "theta":' + theta + ', "delay": ' + str(timeeach) + '}'
      print("Robot cmd: ", cmd)
      print("Return msg: ", myserial.serWrite_and_Read(cmd, waittime))

   tagfinder_obj.get_Destination((-47,20))
   #return  tagfinder_obj.showImage(tagfinder_obj.img,5,1)
   return  cv2.waitKey(5)

# ============================================

while keypress != ord('a'):
   keypress = Run_eachPressKey(tagfinder_obj, draw_obj)
tagfinder_obj.release_Camera()
tagfinder_obj.destroyAllWindows()

myserial.serClose()
