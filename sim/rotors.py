# Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
# Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
# Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
# Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
# Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
# Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0   (also in licenses/Apache-2.0.txt of this repository)
#
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
# an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
#
# This file is ported from RotorS (https://github.com/ethz-asl/rotors_simulator, as in the fork
# https://github.com/ntnu-arl/rotors_simulator at commit e8c6f170de51620f572664656f10603334bf5057):
#   LeeController <- rotors_control/src/library/lee_position_controller.cpp and the allocation matrix of
#                    rotors_control/include/rotors_control/common.h
#   Rotors        <- rotors_gazebo_plugins/src/gazebo_motor_model.cpp
# Changes (for sim/drone_sim.py of this project): rewritten in Python/NumPy; parameters read from config/robots.yaml;
# the motor model reduced to the first-order speed lag, thrust, rotor drag against the sideways velocity and the drag
# torque, summed into one wrench on the body (no rolling moment, no wind, no rotor spin-up visualisation).
"""RotorS's Lee geometric position controller and motor model for sim/drone_sim.py (Apache-2.0, see above)."""
import math

import numpy as np

G = 9.81


class LeeController:
    """RotorS LeePositionController: position, velocity -> desired acceleration -> desired attitude (yaw given) ->
    angular acceleration -> rotor speeds through the allocation matrix. Gains normalised by mass and inertia, as there."""

    def __init__(self, r):
        self.m, self.J = r["mass"], np.array(r["inertia"], float)
        kf, km = r["rotor_force_constant"], r["rotor_moment_constant"]
        # allocation: roll, pitch, yaw torque and thrust per (rotor speed)^2
        alloc = np.array([[math.sin(x["angle"]) * x["arm"] * kf, -math.cos(x["angle"]) * x["arm"] * kf,
                           -x["direction"] * kf * km, kf] for x in r["rotors"]]).T
        self.to_speed2 = alloc.T @ np.linalg.inv(alloc @ alloc.T) @ np.diag(list(self.J) + [1.0])
        g = r["lee_gains"]
        self.kp, self.kv = np.array(g["position"]), np.array(g["velocity"])
        self.kr, self.kw = np.array(g["attitude"]) / self.J, np.array(g["angular_rate"]) / self.J

    def rotor_speeds(self, p, v, rot, w, p_ref, v_ref, yaw_ref, yaw_rate_ref):
        """p, v: position and velocity (world); rot: body-to-world rotation; w: angular velocity (body)."""
        acc = ((p - p_ref) * self.kp + (v - v_ref) * self.kv) / self.m - np.array([0.0, 0.0, G])
        b1 = np.array([math.cos(yaw_ref), math.sin(yaw_ref), 0.0])
        b3 = -acc / np.linalg.norm(acc)
        b2 = np.cross(b3, b1)
        b2 /= np.linalg.norm(b2)
        rot_des = np.column_stack([np.cross(b2, b3), b2, b3])
        e = 0.5 * (rot_des.T @ rot - rot.T @ rot_des)
        e_rot = np.array([e[2, 1], e[0, 2], e[1, 0]])
        e_rate = w - rot_des.T @ rot @ np.array([0.0, 0.0, yaw_rate_ref])
        ang_acc = -e_rot * self.kr - e_rate * self.kw
        thrust = -self.m * acc @ rot[:, 2]
        return np.sqrt(np.maximum(self.to_speed2 @ np.append(ang_acc, thrust), 0.0))


class Rotors:
    """Motor lag and the wrench on the body (body frame) from the rotor speeds, as RotorS's gazebo_motor_model."""

    def __init__(self, r):
        self.kf, self.km, self.drag = r["rotor_force_constant"], r["rotor_moment_constant"], r["rotor_drag_coefficient"]
        self.max, (self.tau_up, self.tau_down) = r["rotor_max_speed"], r["rotor_time_constants"]
        self.pos = np.array([[x["arm"] * math.cos(x["angle"]), x["arm"] * math.sin(x["angle"]), 0.0]
                             for x in r["rotors"]])
        self.dirs = np.array([x["direction"] for x in r["rotors"]])
        self.speed = np.zeros(len(r["rotors"]))

    def step(self, speed_cmd, v_body, dt):
        cmd = np.clip(speed_cmd, 0.0, self.max)
        tau = np.where(cmd > self.speed, self.tau_up, self.tau_down)
        self.speed += (cmd - self.speed) * (1.0 - np.exp(-dt / tau))
        thrust = self.kf * self.speed ** 2
        force = np.array([-self.drag * self.speed.sum() * v_body[0], -self.drag * self.speed.sum() * v_body[1],
                          thrust.sum()])
        torque = np.cross(self.pos, np.outer(thrust, [0.0, 0.0, 1.0])).sum(0)
        torque[2] += (-self.dirs * self.km * thrust).sum()
        return force, torque
