#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp, preview, pi
from sympy.interactive import printing


class RobotArmDH:
    def __init__(self):
        theta_i = Symbol("theta_i")
        alpha_i = Symbol("alpha_i")
        r_i = Symbol("r_i")
        d_i = Symbol("d_i")

        # # To make display prety
        # printing.init_printing(use_latex = True)

        DH_Matric_Generic = Matrix([
                [cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), r_i*cos(theta_i)],
                [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), r_i*sin(theta_i)],
                [0, sin(alpha_i), cos(alpha_i), d_i],
                [0,0,0,1]
            ])

        # result_simpl = simplify(DH_Matric_Generic)
        # # Save to local file
        # preview(result_simpl, viewer='file', filename="out.png", dvioptions=['-D','300'])

        # Now create A01, A12, A23
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
        self.A02 = self.A01 * self.A12

        self.A03_simplify = trigsimp(self.A03)
        self.A02_simplify = trigsimp(self.A02)

def main(args=None):

    robot_arm_DH = RobotArmDH()
    # We save

    preview(robot_arm_DH.A01, viewer='file', filename="A01.png", dvioptions=['-D','300'])
    # preview(robot_arm_DH.A02, viewer='file', filename="A02.png", dvioptions=['-D','300'])
    preview(robot_arm_DH.A03, viewer='file', filename="A03.png", dvioptions=['-D','300'])

    preview(robot_arm_DH.A12, viewer='file', filename="A12.png", dvioptions=['-D','300'])
    preview(robot_arm_DH.A23, viewer='file', filename="A23.png", dvioptions=['-D','300'])
    preview(robot_arm_DH.A03_simplify, viewer='file', filename="A03_simplify.png", dvioptions=['-D','300'])
    # preview(robot_arm_DH.A02_simplify, viewer='file', filename="A02_simplify.png", dvioptions=['-D','300'])

if __name__ == '__main__':
    main()