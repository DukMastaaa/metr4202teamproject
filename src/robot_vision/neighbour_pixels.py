import numpy as np
from math import sqrt, acos, cos, pow
from typing import List, Tuple

# tester
x0, y0, x1, y1, x2, y2, x3, y3 = 1.0,1.0,2.0,5.0,4.0,4.0,3.0,0.0
fid = [x0, y0, x1, y1, x2, y2, x3, y3]

# def fid_to_points(fid: List[float]) -> List[np.array]:
#     """Converts a list of 8 floats to a list of 4 np.array points."""
#     points = []
#     for i in range(len(fid)):
#         points.append(np.array([fid[i], fid[i+1]]))
#     return points

def in_tag(point, vertices):
    # point: A list with the input point's coordinates (length = 2)
    # fid: A list with the fiducial coordinates from x0 to y3
    # Return: bool of whether the point is in the quadrilateral
    
    epsilon = 0.01 # margin of error
    P_x = point[0]
    P_y = point[1]
    i = 0 # iteration variable
    total_area = 0 # area of four quadrants made by point to fid coordinates

    while i < len(fid):
        A_x = fid[i - 2]
        A_y = fid[i - 1]
        B_x = fid[i]
        B_y = fid[i + 1]

        if ((P_x == A_x) and (P_y == A_y)) or ((P_x == B_x) and (P_y == B_y)):
            return True

        a = sqrt(pow(P_x - A_x, 2) + pow(P_y - A_y, 2))
        b = sqrt(pow(P_x - B_x, 2) + pow(P_y - B_y, 2))
        c = sqrt(pow(A_x - B_x, 2) + pow(A_y - B_y, 2))
        C = acos((pow(a, 2.0) + pow(b, 2.0) - pow(c, 2.0)) / (2.0 * a * b))
        area = abs(0.5 * a * b * cos(C)) # area of single quadrant
        total_area += area
        print(area)
        i += 2
    
    print("Quadrant total: ", total_area)
    actual_area = 0.5 * abs((((abs(fid[0] - fid[4])) * (abs(fid[3] - fid[7]))) - (abs(fid[2] - fid[6])) * (abs(fid[1] - fid[5])))) # area of fid quadrilateral
    # actual_area = 1/2 * ((x_0 - x_2)(y_1 - y_3) - (x_1 - x_3)(y_0 - y_2))
    print("Actual total: ", actual_area)

    if abs(total_area - actual_area) < epsilon:
        return True
    else:
        return False

def pixel_picker():
    # Picks valid pixels to use for colour detection

    i = 0 # iteration variable
    pixel_choices = [] # a list of two-dimensional coordinates

    L = 5
    
    while i < len(fid):
        A_x = fid[i]
        A_y = fid[i + 1]
    
        # Takes 6 points around fid point (above / below omitted)
        for j in range(-L//2, L//2 + 1):
            for k in range(-L//2, L//2 + 1):
                if (in_tag([A_x + j, A_y + k]) == False):
                    pixel_choices += [A_x + j, A_y + k]
            
        i += 2
    print(len(pixel_choices))
    return pixel_choices
    