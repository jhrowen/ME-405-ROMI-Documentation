import pyb
from task_share import Share, Queue
from time import ticks_us, ticks_diff
from ulab import numpy as np

class generate_curve:
    def __init__(self,estS,estPsi,estOmegaL,estOmegaR,start_path,xPos,yPos,xdot, ydot,ax,bx,cx,dx,ay,by,cy,dy,segment_id,follow_path,segment_done,changex,changey):
        self._arclength : Share = estS
        self._heading : Share = estPsi
        self._OmegaL : Share = estOmegaL
        self._OmegaR: Share = estOmegaR
        self._start : Share = start_path
        self._xPos : Share = xPos
        self._yPos : Share = yPos
        self._xdot : Share = xdot
        self._ydot : Share = ydot
        self._ax : Share = ax
        self._bx : Share = bx
        self._cx : Share = cx
        self._dx : Share = dx
        self._ay : Share = ay
        self._by : Share = by
        self._cy : Share = cy
        self._dy : Share = dy
        self._segment_done : Share = segment_done
        self._follow_path : Share = follow_path
        self._state = 0
        self._dt = 0
        self._time_prev = ticks_us()
        self._path1 = np.array([
                                    [0,0,78.54,0],
                                    [31.29,-2.46,77.57,-12.29],
                                    [61.8,-9.79,74.7,-24.27],
                                    [90.8,-21.8,69.98,-35.66],
                                    [117.56,-38.2,63.54,-46.16],
                                    [141.42,-58.58,55.54,-55.54],
                                    [161.80,-82.44,46.16,-63.54],
                                    [178.2,-109.2,35.66,-69.98],
                                    [190.21,-138.2,24.27,-74.7],
                                    [197.54,-168.71,12.29,-77.57],
                                    [200,-200,0,-78.54],
                                ])
        self._path2 = np.array([    [200, -200, 0, -49.09],
                                    [198.46,-219.55,-7.68,-48.48],
                                    [193.88,-238.63,-15.17,-46.68],
                                    [186.38,-256.35,-22.29,-43.74],
                                    [176.13,-273.47,-28.85,-39.71],
                                    [163.39,-288.39,-34.71,-34.71],
                                    [148.47,-301.13,-39.71,-28.85],
                                    [131.75,-311.38,-43.74,-22.89],
                                    [113.63,-318.88,-46.68,-15.17],
                                    [94.55,-323.46,-48.48,-7.68],
                                    [75,-325,-49.09,0]
                                ])
        self._changex : Queue = changex
        self._changey : Queue = changey
        self._segment_id : Share = segment_id
        self._i = 0
    def run(self):
        while True:
            if self._state == 0:
                if self._start.get():
                    self._follow_path.put(True)
                    self._segment_done.put(True)
                    self._i = 1
                    self._state = 1
                    
            elif self._state == 1:
                # first curve
                if self._segment_done.get():
                    self._segment_done.put(False)
                    if self._i < len(self._path1):
                        x_act = self._xPos.get()
                        y_act = self._yPos.get()
                        print(str(x_act) + "," + str(y_act))
                        print(self._segment_id.get())
                        dx_act = self._xdot.get()
                        dy_act = self._ydot.get()
                        # print(str(dx_act) + "," + str(dy_act))
                        q1x = self._path1[self._i,0]
                        q1y = self._path1[self._i,1]
                        q1xdot = self._path1[self._i,2]
                        q1ydot = self._path1[self._i,3]
                        #print(q1x , q1y, q1xdot , q1ydot)
                        # a,b,c,d for x direction
                        self._ax.put(x_act)
                        self._bx.put(dx_act)
                        cx = -18.75*x_act - 5* dx_act + 18.75 * q1x - 2.5 * q1xdot
                        self._cx.put(cx)
                        dx = 31.25* x_act + 6.25 * dx_act - 31.25 * q1x + 6.25 * q1xdot
                        self._dx.put(dx)
                        # a,b,c,d for y direction
                        self._ay.put(y_act)
                        self._by.put(dy_act)
                        cy = -18.75* y_act - 5 * dy_act + 18.75 * q1y - 2.5 * q1ydot
                        self._cy.put(cy)
                        dy = 31.25* y_act + 6.25 * dy_act - 31.25 * q1y + 6.25 * q1ydot
                        self._dy.put(dy)
                        self._segment_id.put(self._i)
                        self._i += 1
                        yield self._state
                    else:
                        self._state = 2
                        self._changex.put(200.0)
                        self._changey.put(-200.0)
                        print(str(self._xPos.get()) + "," + str(self._yPos.get()))
                        self._i = 1
                        self._segment_done.put(True)
            elif self._state == 2:
                # first curve
                if self._segment_done.get():
                    self._segment_done.put(False)
                    if self._i < len(self._path2):
                        x_act = self._xPos.get()
                        y_act = self._yPos.get()
                        print(str(x_act) + "," + str(y_act))
                        print(self._segment_id.get())
                        dx_act = self._xdot.get()
                        dy_act = self._ydot.get()
                        # print(str(dx_act) + "," + str(dy_act))
                        q1x = self._path2[self._i,0]
                        q1y = self._path2[self._i,1]
                        q1xdot = self._path2[self._i,2]
                        q1ydot = self._path2[self._i,3]
                        #print(q1x , q1y, q1xdot , q1ydot)
                        # a,b,c,d for x direction
                        self._ax.put(x_act)
                        self._bx.put(dx_act)
                        cx = -18.75*x_act - 5* dx_act + 18.75 * q1x - 2.5 * q1xdot
                        self._cx.put(cx)
                        dx = 31.25* x_act + 6.25 * dx_act - 31.25 * q1x + 6.25 * q1xdot
                        self._dx.put(dx)
                        # a,b,c,d for y direction
                        self._ay.put(y_act)
                        self._by.put(dy_act)
                        cy = -18.75* y_act - 5 * dy_act + 18.75 * q1y - 2.5 * q1ydot
                        self._cy.put(cy)
                        dy = 31.25* y_act + 6.25 * dy_act - 31.25 * q1y + 6.25 * q1ydot
                        self._dy.put(dy)
                        self._segment_id.put(self._i)
                        self._i += 1
                        yield self._state
                    else:
                        self._state = 0
                        print(str(self._xPos.get()) + "," + str(self._yPos.get()))
                        self._follow_path.put(False)
                        self._start.put(False)
            yield self._state




                

