from task_share import Share, Queue
from time import ticks_ms, ticks_diff
from ulab import numpy as np
from micropython import const

S0_wait = const(0)
S1_run = const(1)


class follow_path_task:
    def __init__(
        self,
        vL,
        vR,
        ax,
        bx,
        cx,
        dx,
        ay,
        by,
        cy,
        dy,
        follow_path,
        segment_id,
        next_segment,
        xPos,
        yPos,
        psi
    ):
        self._vL_ref: Share = vL
        self._vR_ref: Share = vR

        self._ax: Share = ax
        self._bx: Share = bx
        self._cx: Share = cx
        self._dx: Share = dx

        self._ay: Share = ay
        self._by: Share = by
        self._cy: Share = cy
        self._dy: Share = dy

        self._follow_path: Share = follow_path
        self._next_segment: Share = next_segment

        self._xPos = xPos
        self._yPos = yPos
        self._Psi = psi

        self._segment_id: Share = segment_id

        self._state = S0_wait
        self._segment = 0
        self._request_next = False

        self._t0 = 0
        self._prev_time = 0

        # Robot parameters
        self._w = 141            # wheel track width [mm]
        self._a_max = 500        # max wheel accel [mm/s^2]

        # Commanded wheel speeds
        self._vL_cmd = 0
        self._vR_cmd = 0

        # Tracking gains during spline-following phase
        self._kx = 0.6
        self._ky = 0.008
        self._kpsi = 0.6

        # Endpoint capture gains after spline time has elapsed
        self._k_rho = 0.8        # forward gain toward endpoint
        self._k_alpha = 2.0      # heading gain toward endpoint

        # Segment timing
        self._T = 0.4            # nominal spline segment duration [s]
        self._timeout = 1.2      # hard timeout for a segment [s]
        self._tol = 20.0         # endpoint tolerance [mm]

        # Endpoint capture limits
        self._v_capture_max = 80.0
        self._v_capture_turn = 20.0
        self._turn_only_thresh = 0.5   # [rad]

        # Wheel speed saturation
        self._wheel_speed_limit = 250.0

    def _wrap_to_pi(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def run(self):
        while True:
            if self._state == S0_wait:
                if self._follow_path.get():
                    self._t0 = ticks_ms()
                    self._prev_time = self._t0
                    self._vL_cmd = 0
                    self._vR_cmd = 0
                    self._segment = self._segment_id.get()
                    self._request_next = False
                    self._next_segment.put(False)
                    self._state = S1_run

            elif self._state == S1_run:
                if not self._follow_path.get():
                    self._vL_ref.put(0)
                    self._vR_ref.put(0)
                    self._state = S0_wait
                    yield self._state
                    continue

                # If a new segment has been loaded, reset timer and handoff flag
                if self._segment != self._segment_id.get():
                    self._segment = self._segment_id.get()
                    self._t0 = ticks_ms()
                    self._prev_time = self._t0
                    self._request_next = False
                    self._next_segment.put(False)

                # Timing
                tnow = ticks_ms()
                t = ticks_diff(tnow, self._t0) / 1000.0
                dt = ticks_diff(tnow, self._prev_time) / 1000.0
                self._prev_time = tnow

                if dt < 0:
                    dt = 0

                # Read spline coefficients
                ax = self._ax.get()
                bx = self._bx.get()
                cx = self._cx.get()
                dx = self._dx.get()

                ay = self._ay.get()
                by = self._by.get()
                cy = self._cy.get()
                dy = self._dy.get()

                # Read actual pose
                x_act = self._xPos.get()
                y_act = self._yPos.get()
                psi_act = self._Psi.get()

                # Endpoint of current segment
                x_end = ax + bx * self._T + cx * (self._T ** 2) + dx * (self._T ** 3)
                y_end = ay + by * self._T + cy * (self._T ** 2) + dy * (self._T ** 3)

                dx_end = x_end - x_act
                dy_end = y_end - y_act
                dist_end = np.sqrt(dx_end ** 2 + dy_end ** 2)

                vcmd = 0.0
                omega_cmd = 0.0

                # -------------------------------
                # Phase 1: Follow time-parameterized spline
                # -------------------------------
                if t < self._T:
                    qx = ax + bx * t + cx * t ** 2 + dx * t ** 3
                    qy = ay + by * t + cy * t ** 2 + dy * t ** 3

                    qdotx = bx + 2 * cx * t + 3 * dx * t ** 2
                    qdoty = by + 2 * cy * t + 3 * dy * t ** 2

                    qddx = 2 * cx + 6 * dx * t
                    qddy = 2 * cy + 6 * dy * t

                    psi_d = np.arctan2(qdoty, qdotx)

                    # Position error in ACTUAL robot body frame
                    ex_w = qx - x_act
                    ey_w = qy - y_act

                    ex = np.cos(psi_act) * ex_w + np.sin(psi_act) * ey_w
                    ey = -np.sin(psi_act) * ex_w + np.cos(psi_act) * ey_w

                    epsi = self._wrap_to_pi(psi_d - psi_act)

                    v_ff = np.sqrt(qdotx ** 2 + qdoty ** 2)
                    den = qdotx ** 2 + qdoty ** 2

                    if den < 1e-6:
                        psidot_ff = 0.0
                    else:
                        psidot_ff = (qdotx * qddy - qdoty * qddx) / den

                    vcmd = v_ff + self._kx * ex
                    omega_cmd = psidot_ff + self._ky * ey + self._kpsi * epsi

                    self._next_segment.put(False)

                # -------------------------------
                # Phase 2: Endpoint capture mode
                # -------------------------------
                else:
                    psi_to_end = np.arctan2(dy_end, dx_end)
                    epsi_end = self._wrap_to_pi(psi_to_end - psi_act)

                    if dist_end < self._tol:
                        if not self._request_next:
                            self._next_segment.put(True)
                            self._request_next = True

                        vcmd = 0.0
                        omega_cmd = 0.0

                    elif t >= self._timeout:
                        # Hard stop if endpoint capture fails
                        self._vL_ref.put(0)
                        self._vR_ref.put(0)
                        self._next_segment.put(False)
                        self._follow_path.put(False)
                        print("Segment timeout:", self._segment)
                        print("dist_end =", dist_end)
                        print("epsi_end =", epsi_end)
                        self._state = S0_wait
                        yield self._state
                        continue

                    else:
                        self._next_segment.put(False)

                        # Point stabilizer toward endpoint
                        vcmd = self._k_rho * dist_end
                        if vcmd > self._v_capture_max:
                            vcmd = self._v_capture_max

                        omega_cmd = self._k_alpha * epsi_end

                        # If badly misaligned, mostly turn in place
                        if abs(epsi_end) > self._turn_only_thresh:
                            vcmd = self._v_capture_turn

                # Convert body commands to wheel commands
                vL_ref = vcmd - (self._w * omega_cmd / 2.0)
                vR_ref = vcmd + (self._w * omega_cmd / 2.0)

                # Saturate wheel speed commands
                if vL_ref > self._wheel_speed_limit:
                    vL_ref = self._wheel_speed_limit
                elif vL_ref < -self._wheel_speed_limit:
                    vL_ref = -self._wheel_speed_limit

                if vR_ref > self._wheel_speed_limit:
                    vR_ref = self._wheel_speed_limit
                elif vR_ref < -self._wheel_speed_limit:
                    vR_ref = -self._wheel_speed_limit

                # Slew-rate limit wheel speeds
                dv_max = self._a_max * dt

                dL = vL_ref - self._vL_cmd
                if abs(dL) > dv_max:
                    if dL > 0:
                        dL = dv_max
                    else:
                        dL = -dv_max
                self._vL_cmd += dL

                dR = vR_ref - self._vR_cmd
                if abs(dR) > dv_max:
                    if dR > 0:
                        dR = dv_max
                    else:
                        dR = -dv_max
                self._vR_cmd += dR

                # Output commands
                self._vL_ref.put(self._vL_cmd)
                self._vR_ref.put(self._vR_cmd)

            yield self._state
