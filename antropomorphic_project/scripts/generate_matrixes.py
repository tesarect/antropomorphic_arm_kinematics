#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp, preview, pi, lambdify
import numpy as np

class RobotArmDH:
    def __init__(self):
        theta_i = Symbol("theta_i")
        alpha_i = Symbol("alpha_i")
        r_i = Symbol("r_i")
        d_i = Symbol("d_i")

        DH_Matric_Generic = Matrix([
                [cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), r_i*cos(theta_i)],
                [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), r_i*sin(theta_i)],
                [0, sin(alpha_i), cos(alpha_i), d_i],
                [0,0,0,1]
            ])

        # create A01, A12, A23
        self.theta_1 = Symbol("theta_1")
        self.theta_2 = Symbol("theta_2")
        self.theta_3 = Symbol("theta_3")

        alpha_1 = pi/2
        alpha_2 = 0.0
        alpha_3 = 0.0

        self.r_1 = 0.0
        self.r_2 = Symbol("r_2")
        self.r_3 = Symbol("r_3")

        d_1 = 0.0
        d_2 = 0.0
        d_3 = 0.0

        self.A01 = DH_Matric_Generic.subs(r_i,self.r_1).subs(alpha_i,alpha_1).subs(d_i,d_1).subs(theta_i, self.theta_1)
        self.A12 = DH_Matric_Generic.subs(r_i,self.r_2).subs(alpha_i,alpha_2).subs(d_i,d_2).subs(theta_i, self.theta_2)
        self.A23 = DH_Matric_Generic.subs(r_i,self.r_3).subs(alpha_i,alpha_3).subs(d_i,d_3).subs(theta_i, self.theta_3)

        self.A03 = self.A01 * self.A12 * self.A23

        self._A03_simplify = trigsimp(self.A03)

        self.A03_func = lambdify(
            (self.theta_1, self.theta_2, self.theta_3, self.r_2, self.r_3),
            self.A03_simplified,
            modules='numpy'
        )

    @property
    def A03_simplified(self):
        # Copy is done since SymPy matrices are mutable
        return self._A03_simplify.copy()
    
    def evaluate_A03(self, theta1_val, theta2_val, theta3_val, r2_val, r3_val):
        """
        Evaluate simplified A03 matrix
        """
        return np.array(
            self.A03_func(theta1_val, theta2_val, theta3_val, r2_val, r3_val),
            dtype=float
        )

def main(args=None):

    robot_arm_DH = RobotArmDH()
    
    preview(robot_arm_DH.A01, viewer='file', filename="A01.png", dvioptions=['-D','300'])
    preview(robot_arm_DH.A03, viewer='file', filename="A03.png", dvioptions=['-D','300'])

    preview(robot_arm_DH.A12, viewer='file', filename="A12.png", dvioptions=['-D','300'])
    preview(robot_arm_DH.A23, viewer='file', filename="A23.png", dvioptions=['-D','300'])
    preview(robot_arm_DH._A03_simplify, viewer='file', filename="A03_simplify.png", dvioptions=['-D','300'])

if __name__ == '__main__':
    main()