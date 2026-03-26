#!/usr/bin/env python3

from math import pi, atan2, cos, sin, sqrt, hypot
from dataclasses import dataclass


@dataclass
class EndEffectorWorkingSpace:
    Pee_x: float
    Pee_y: float
    Pee_z: float


class ComputeIk():
    def __init__(self, DH_parameters):
        self.DH_parameters_ = DH_parameters

    def get_dh_param(self, name):
        if name in self.DH_parameters_:
            return self.DH_parameters_[name]
        raise KeyError("Asked for non-existent DH param: " + str(name))

    def wrap_pi(self, a):
        """Wrap angle to [-pi, pi]."""
        return (a + pi) % (2.0 * pi) - pi

    def compute_ik(self, end_effector_pose, elbow_config="plus-plus", verbose=False):
        """
        Compute IK for a 3-joint anthropomorphic arm.

        elbow_config encodes two independent choices as 'theta2_config-theta3_config':
          theta2_config 'plus'  -> front solution (arm faces target directly)
          theta2_config 'minus' -> back solution  (base rotated ~180 deg)
          theta3_config 'plus'  -> elbow up   (sin(theta3) >= 0)
          theta3_config 'minus' -> elbow down (sin(theta3) <= 0)
        """
        Px = end_effector_pose.Pee_x
        Py = end_effector_pose.Pee_y
        Pz = end_effector_pose.Pee_z

        r2 = self.get_dh_param('r2')
        r3 = self.get_dh_param('r3')

        theta2_config, theta3_config = elbow_config.strip().lower().split('-')

        # Step 1 — theta1: rotate base to face the target in the XY plane.
        #   Front: point directly at target.
        #   Back:  point away (add -pi), effectively flipping the arm around.
        R_mag = hypot(Px, Py)           # horizontal distance to target

        if theta2_config == "plus":
            theta1   = self.wrap_pi(atan2(Py, Px))
            R_signed = +R_mag           # positive rho for forward-facing arm
        else:
            theta1   = self.wrap_pi(atan2(Py, Px) - pi)
            R_signed = -R_mag           # negative rho mirrors the geometry for back solution

        # Step 2 — theta3: law of cosines on the triangle (r2, r3, L).
        #   L = straight-line distance from shoulder to target.
        #   cos(theta3) = (L^2 - r2^2 - r3^2) / (2 * r2 * r3)
        L2 = R_mag**2 + Pz**2
        C3 = (L2 - r2**2 - r3**2) / (2.0 * r2 * r3)   # cos(theta3)

        # Reachability check
        if C3 < -1.0 or C3 > 1.0:
            raise ValueError("Target position not reachable with given r2, r3")

        S3 = sqrt(1.0 - C3**2)                          # sin(theta3), always >= 0
        if theta3_config == "minus":
            S3 = -S3                                     # flip for elbow-down

        theta3 = self.wrap_pi(atan2(S3, C3))

        # Step 3 — theta2: angle subtraction.
        #   The target direction in the (R_signed, Pz) plane is atan2(Pz, R_signed).
        #   The second link contributes angle atan2(r3*S3, r2 + r3*C3).
        #   Subtracting gives the shoulder angle theta2.
        k1 = r2 + r3 * C3
        k2 = r3 * S3
        theta2 = self.wrap_pi(atan2(Pz, R_signed) - atan2(k2, k1))

        # Joint limit checks
        theta2_ok = (-pi/4      <= theta2 <= 3*pi/4)
        theta3_ok = (-3*pi/4    <= theta3 <= 3*pi/4)
        possible  = theta2_ok and theta3_ok

        if verbose:
            print("Input Data===== theta_2_config =", theta2_config)
            print("Input Data===== theta_3_config =", theta3_config)
            print("Pee_x =", Pee_x)
            print("Pee_y =", Pee_y)
            print("Pee_z =", Pee_z)
            print("r2 =", r2)
            print("r3 =", r3)
            print(f"theta1={theta1:.4f}, theta2={theta2:.4f}, theta3={theta3:.4f}")
            if not theta2_ok:
                print(f">>> theta2 NOT POSSIBLE: {theta2:.4f} outside [{-pi/4:.4f}, {3*pi/4:.4f}]")
            if not theta3_ok:
                print(f">>> theta3 NOT POSSIBLE: {theta3:.4f} outside [{-3*pi/4:.4f}, {3*pi/4:.4f}]")

        return [theta1, theta2, theta3], possible


def calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, elbow_config="plus-plus", verbose=False):
    ik = ComputeIk(DH_parameters=DH_parameters)
    end_effector_pose = EndEffectorWorkingSpace(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z)
    thetas, possible = ik.compute_ik(end_effector_pose=end_effector_pose,
                                     elbow_config=elbow_config, verbose=verbose)
    print(f"Angles thetas solved ={thetas} , solution possible = {possible}")
    return thetas, possible


if __name__ == '__main__':
    DH_parameters = {"r2": 1.0, "r3": 1.0}

    Pee_x, Pee_y, Pee_z = 0.5, 0.6, 0.7

    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, elbow_config="plus-plus", verbose=True)
    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, elbow_config="plus-minus", verbose=True)
    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, elbow_config="minus-plus", verbose=True)
    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, elbow_config="minus-minus", verbose=True)