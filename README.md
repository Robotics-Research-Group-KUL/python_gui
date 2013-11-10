GUI in python
================================

Collection of small program to use with ros.

gui_sender.py
-------------------------
A simple interface that reading from an xml file with the format
<pre>
<list>
<button name='Start' event='e_start' tooltip='start the robot'/>
<button name='Stop' event='e_stop'/>
</list>
</pre>
Each button group produces a button that if pressed send the named event.
the tooltip is an optinoal field (text of a tooltip box)
