import os

import sys
import select

import curses

stdscr = curses.initscr()
curses.cbreak()
#stdscr.keypad(1)

#stdscr.addstr(0,10,"Hit 'q' to quit")
#stdscr.refresh()

key = ''
#filename = "group-"
count = 0
while key != ord('q'):
    key = stdscr.getch()
   # stdscr.addch(20,25,key)
   # stdscr.refresh()
   # if key == curses.KEY_UP: 
   #     stdscr.addstr(2, 20, "Up")
   # elif key == curses.KEY_DOWN: 
   #     stdscr.addstr(3, 20, "Down")
    if key == ord('z'):
        count += 1
        filename = "group-" + str(count) + ".bag"
        command = "rosbag record chatter -O " + filename + " --d 3.5s"
        os.system(command)
        print "Finished: " + filename

curses.endwin()
