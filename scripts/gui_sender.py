#!/usr/bin/python3
from PyQt5 import QtGui, QtCore, QtWidgets
from lxml import etree
import sys

import roslib
from roslib.packages import get_pkg_dir
import rospy
from std_msgs.msg import String


roslib.load_manifest("python_gui")

    

class EventSender(QtWidgets.QWidget):
    
    def __init__(self):
        super(EventSender, self).__init__()
        self.initUI()
      
        
    def initUI(self):      


        #ros related stuff
        topic_name=rospy.get_param('topic_name', "/events")
        self.pub=rospy.Publisher(topic_name,String,queue_size=10)
        self.string_map={}
        self.buttons={}
        self.create_buttons()



        
    def create_buttons(self):
        QtWidgets.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        file_name=rospy.get_param('xml_button_file', None)
        file_name_pkg=rospy.get_param('xml_button_file_pkg', None)
        if file_name==None:
           file_name =(get_pkg_dir("python_gui"))+"/xml/default.xml"
        elif file_name_pkg != None:
           file_name = (get_pkg_dir(file_name_pkg))+file_name
            
        #print "file name:\n"+file_name
        file_map = etree.parse(file_name)
        
        list_event =  file_map.getroot()
        grid = QtWidgets.QGridLayout()
        i=0
        for child  in list_event:
            self.string_map[child.get('name')]=child.get('event')
            btn=QtWidgets.QPushButton(child.get('name'), self)
            grid.addWidget(btn,i,0)
            btn.clicked.connect(self.buttonClicked) 
            tooltip_text=child.get('tooltip')
            if tooltip_text!=None:
                btn.setToolTip(tooltip_text)
            
            i=i+1
 
        #layout: a vbox divided in 3 parts
        # the grid, a Stretch to fill in the space and a status bar
        
        vbox = QtWidgets.QVBoxLayout()
        vbox.addLayout(grid)
        self.statBar = QtWidgets.QStatusBar(self)
        vbox.addStretch(1)
        vbox.addWidget(self.statBar)
        self.statBar.showMessage("Event sender by GB")
        
        self.setLayout(vbox)
        self.setGeometry(300, 300, 290, 150)
        self.move(1500, 150)
        self.setWindowTitle('Event sender')
        self.setWindowIcon(QtGui.QIcon((get_pkg_dir("python_gui")) + '/resources/es_ico.png'))
        self.show()
        
            
    def buttonClicked(self):
      
        sender = self.sender()
        string_key=(sender.text()) 
        event_to_send=self.string_map[string_key]
        self.statBar.showMessage(string_key + ' pressed.\n->sent: '+
                                    event_to_send)
        self.pub.publish(event_to_send)

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtWidgets.QApplication.quit()
import signal 
def main():
    rospy.init_node("sender")
    signal.signal(signal.SIGINT, sigint_handler)
    app = QtWidgets.QApplication(sys.argv)
    timer = QtCore.QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    ex = EventSender()
     
    status = app.exec_()
    sys.exit(status)
   

if __name__ == '__main__':
    main()
