#!/usr/bin/env python3

from sympy import preview, Matrix
from generate_matrixes import RobotArmDH

def main():
    # Get user input for angles (radians)
    theta1_val = float(input("Enter the value for theta_1(rad): "))
    theta2_val = float(input("Enter the value for theta_2(rad): "))
    theta3_val = float(input("Enter the value for theta_3(rad): "))

    r2_val = 1.0
    r3_val = 1.0

    robot_arm = RobotArmDH()

    A03_eval = robot_arm.evaluate_A03(
        theta1_val,
        theta2_val,
        theta3_val,
        r2_val,
        r3_val
    )

    # Extract position
    position = A03_eval[:3, 3]

    # Extract orientation
    orientation = A03_eval[:3, :3]

    print("\nPosition Matrix:")
    print(position)

    print("\nOrientation matrix:")
    print(orientation)

    # Convert back to SymPy for preview
    A03_sym_eval = Matrix(A03_eval)

    preview(A03_sym_eval, viewer='file',
            filename="A03_simplify_evaluated.png",
            dvioptions=['-D','300'])

if __name__ == '__main__':
    main()