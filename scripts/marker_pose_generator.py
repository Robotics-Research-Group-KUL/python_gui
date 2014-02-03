#!/usr/bin/env python


import roslib
roslib.load_manifest("interactive_markers")
import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose
from std_msgs.msg import String
#roslib.load_manifest("python_gui")
def make6DofMarker( int_marker,fixed ):
    
    # insert a box
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.2
    box_marker.scale.y = 0.2
    box_marker.scale.z = 0.2
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 0.5

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.interaction_mode=InteractiveMarkerControl.BUTTON
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )


    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED

    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)
    
     # This will add an ``Option'' with the menu on it
    #control = InteractiveMarkerControl()
    #control.interaction_mode = InteractiveMarkerControl.MENU
    #control.description="Options"
    #control.name = "menu_only_control"
    #int_marker.controls.append((control))

    


def processFeedback(feedback):
    #p = feedback.pose.position
    #print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)
    pub_pose.publish(feedback.pose)
      
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        pub_event.publish(event_on_click)
        rospy.loginfo( ": button click"   "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo(  ": menu item " + str(feedback.menu_entry_id) + " clicked" )
        print(feedback.menu_entry_id)
        if feedback.menu_entry_id==1:
            rospy.loginfo( ": reset pose" )
            p=Pose();
            p=feedback.pose;
            p.orientation=starting_pose.orientation
            server.setPose( feedback.marker_name, p )
            server.applyChanges()
        elif feedback.menu_entry_id==2:
            rospy.loginfo( ": reset orientation" )
            server.setPose( feedback.marker_name, starting_pose )
            server.applyChanges()
        if feedback.menu_entry_id==3:
            print(feedback.menu_entry_id)
            pub_event.publish(event_on_click)
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo(  ": new Pose is  " + str(feedback.pose))



if __name__=="__main__":
    rospy.init_node("simple_marker")
    # read parameters
    ref_frame_name=rospy.get_param('ref_name', "/base_link")
    event_on_click=rospy.get_param('event_on_click', "e_go_new_pos")
    
    p=Pose()
    p.position.z=0.5;
    p.orientation.w=1.0;
    starting_pose=rospy.get_param('starting_pose', p)
    pub_pose=rospy.Publisher('desired_pose',Pose)
    pub_event=rospy.Publisher('events',String)
    
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("desired_pose_marker")
    
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = ref_frame_name
    int_marker.name = "target_marker"
    int_marker.description = "Pose for trajectory generator"
    int_marker.pose=starting_pose
    # create marker
    make6DofMarker( int_marker,False )
    int_marker.scale = 0.3
    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)
    
    #creates the menu
    menu_handler = MenuHandler()
    menu_handler.insert( "Reset Initial Orientation", callback=processFeedback )
    menu_handler.insert( "Reset Initial Pose", callback=processFeedback )
    menu_handler.insert( "Go to Pose", callback=processFeedback )
    #sub_menu_handle = menu_handler.insert( "Submenu" )
    #menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    #menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )
    
    menu_handler.apply( server, int_marker.name )
    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()
