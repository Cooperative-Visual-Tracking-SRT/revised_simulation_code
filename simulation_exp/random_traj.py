from numpy import random
import numpy as np
cur_x = 0
cur_y = 0
cur_t = 0
for i in range(1,12):
    nex_x = cur_x+5*random.randn()
    nex_y = cur_x+5*random.randn()
    if(i==11):
        nex_x = 0
        nex_y = 0
    nex_t = cur_t + (abs(nex_x-cur_x)+abs(nex_y-cur_y))/0.8
    print("<waypoint>")
    print("  <time>{:.2f}</time>".format(cur_t))
    print("  <pose>{:.2f} {:.2f} 0 0 0 0</pose>".format(cur_x,cur_y))
    print("</waypoint>")
    cur_t = nex_t
    cur_x = nex_x
    cur_y = nex_y

print("<waypoint>")
print("  <time>{:.2f}</time>".format(cur_t))
print("  <pose>{:.2f} {:.2f} 0 0 0 0</pose>".format(cur_x,cur_y))
print("</waypoint>")