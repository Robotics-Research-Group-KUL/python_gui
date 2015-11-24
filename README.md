GUI in python
================================

Collection of small programs to use with ros.

gui_sender.py
-------------------------
A simple interface that reads from an xml file with the format

```
<list>
    <button name='Start' event='e_start' tooltip='start the robot'/>
    <button name='Stop' event='e_stop'/>
</list>
```

Each button group produces a button that, if pressed sends the named event.
The default topic is /events, but can be altered using the ros parameter 'topic_name', see gui_sender.launch for an example on how to change this.
The tooltip is an optional field (text when hovering over a button).
The xml file is stored in the rosparam "xml_button_file", that defauts to default.xml in the xml directory of this package.

usage
-----
```
roscore
roslaunch python_gui gui_sender.launch
```
or
```
roscore
rosrun python_gui gui_sender.py
```
or
```
roscore
cd scripts
./gui_sender.py
```

parameters
----------
See xml/gui_sender.launch for an example with the default values.

 * topic\_name: name of the topic to publish the events on
 * xml\_button\_file\_pkg: package where to find the xml\_button\_file, its location is apended to the xml\_button\_file value
 * xml\_button\_file: location of the xml file with the event buttons to create: full path or path from xml\_button\_file\_pkg location (start with /)

orocos integration
-----
The file 'lua_components/signal_echo.lua'  is a an orocos component realized in lua, that reads a std_msgs/String from a topic and echos as a normal string.
to connect easily with components running rFSM.  

for loading the component (in indigo) try something like
```
require "rttlib"

tc = tc or rtt.getTC()
depl = depl or tc:getPeer("Deployer")
depl:import("rtt_rosnode")
depl:import("rtt_ros")
depl:import("rtt_std_msgs")
depl:import("rtt_rospack")-- this for using the find

depl:loadComponent("eventEcho", "OCL::LuaComponent")
eventEcho = depl:getPeer("eventEcho")
eventEcho:exec_file( rtt.provides("ros"):find("python_gui").."/lua_components/signal_echo.lua")
eventEcho:configure()

--stream data to component
cp_ros=rtt.Variable("ConnPolicy")
cp_ros.transport=3
cp_ros.name_id="/events"
depl:stream("eventEcho.event_in",cp_ros)
--here, connect "eventEcho.event_out" to some other component
cp=rtt.Variable("ConnPolicy")
cp.type=1-- a buffer
cp.size=10
--depl:connect("...","eventEcho.event_out",cp)
```

