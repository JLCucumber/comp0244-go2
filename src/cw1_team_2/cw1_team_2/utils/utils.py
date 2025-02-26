from geometry_msgs.msg import  Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker# 点
import os
import csv

"""
MIT License

Copyright (c) 2025 Comp0244-Team2

Authors: 
- Hongbo Li (Code)
- Yiyang Jia (Code)
- Xinyun Mo (Testing)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

class Point2D(object):

    def __init__(self, x, y):
        self.x, self.y = x, y

# 向量
class Vector(object):

    def __init__(self, start_point, end_point):
        try:
            start_point.x
        except AttributeError:
            start_point = Point2D(start_point[0], start_point[1])
            end_point = Point2D(end_point[0], end_point[1])
        self.start_point, self.end_point = start_point, end_point
        self.x = end_point.x - start_point.x
        self.y = end_point.y - start_point.y
ZERO = 1e-9

def negative(vector):
    """return the negative vector of the input vector"""
    return Vector(vector.end_point, vector.start_point)

def vector_product(vectorA, vectorB):
    '''Calculate x_1 * y_2 - x_2 * y_1'''
    return vectorA.x * vectorB.y - vectorB.x * vectorA.y

def is_intersected(A, B, C, D):
    '''Check if line segment AB intersects with line segment CD'''
    AC = Vector(A, C)
    AD = Vector(A, D)
    BC = Vector(B, C)
    BD = Vector(B, D)
    CA = negative(AC)
    CB = negative(BC)
    DA = negative(AD)
    DB = negative(BD)

    return (vector_product(AC, AD) * vector_product(BC, BD) <= ZERO) \
        and (vector_product(CA, CB) * vector_product(DA, DB) <= ZERO)

def get_rect_marker(rect,stamp):
    """Get a Marker message for a rectangle in livox frame"""
    # Publish the rectangle
    rect_msg = Marker()
    rect_msg.header.frame_id = "livox"
    rect_msg.header.stamp = stamp
    rect_msg.ns = "rectangle"
    rect_msg.id = 3
    rect_msg.type = Marker.POINTS
    rect_msg.action = Marker.ADD
    rect_msg.pose.orientation.w = 1.0
    rect_msg.scale = Vector3(x=0.1, y=0.1, z=0.1)
    rect_msg.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

    for i in range(len(rect)):
        point = Point()
        point.x = rect[i][0]
        point.y = rect[i][1]
        point.z = 0.0
        rect_msg.points.append(point)
    return rect_msg

# for task 1
def save_to_csv(msg, path):
    """Save the marker information to a CSV file"""
    # Collect information
    marker_id = msg.id
    timestamp_sec = msg.header.stamp.sec
    timestamp_nanosec = msg.header.stamp.nanosec

    # Check if file exists
    file_exists = os.path.isfile(path)

    # Write to CSV file
    with open(path, mode='a', newline='') as file:
        writer = csv.writer(file)

        # Write header if file does not exist
        if not file_exists:
            writer.writerow(['Marker ID', 'Timestamp (sec)', 'Timestamp (nanosec)'])
        writer.writerow([marker_id, timestamp_sec, timestamp_nanosec])
