'''
 @Author: JoeyforJoy
 @Date: 2022-03-25 17:03:34
 @LastEditTime: 2022-03-25 17:48:29
 @LastEditors: JoeyforJoy
 @Description: 
'''

def array2D2str(array):
    assert len(array.shape) == 2
    rows, cols = array.shape
    res = ""
    for i in range(rows):
        for j in range(cols):
            if j != cols-1:
                res += str(array[i, j]) + " "
            else:
                res += str(array[i, j])
        res += "\n"
    return res

def points2pcdstr(points):
    """
        NOTE: ONLY surpport pcd 0.7 now.
    """

    assert len(points.shape) == 2
    assert points.shape[1] == 3 or points.shape[1] == 4 or points.shape[1] == 6

    FILEHEAD = "# .PCD v.7 - Point Cloud Data file format\n"
    VERSION = "VERSION .7\n"
    if points.shape[1] == 3:
        FIELDS = "FIELDS " + "x y z\n"
    elif points.shape[1] == 4:
        FIELDS = "FIELDS " + "x y z rgb\n"
    elif points.shape[1] == 6:
        FIELDS = "FIELDS " + "x y z normal_x normal_y normal_z\n"
    else:
        raise NotImplementedError("ONLY surpport three kinds of fields: (x y z), (x y z rgb), (x y z normal_x normal_y normal_z).")

    SIZE = "SIZE " + str("4 " * points.shape[1])[:-1] + "\n"
    TYPE = "TYPE " + str("F " * points.shape[1])[:-1] + "\n"
    COUNT = "COUNT " + str("1 " * points.shape[1])[:-1] + "\n"
    WIDTH = "WIDTH " + str(points.shape[0]) + "\n"
    HEIGHT = "HEIGHT 1\n"
    VIEWPOINT = "VIEWPOINT 0 0 0 1 0 0 0\n"
    POINTS = "POINTS " + str(points.shape[0]) + "\n"
    DATA = "DATA ascii\n"
    POINTS_STR = array2D2str(points)

    final_str = FILEHEAD + VERSION + FIELDS + SIZE + TYPE + COUNT \
                + WIDTH + HEIGHT + VIEWPOINT + POINTS + DATA + POINTS_STR
    
    return str(final_str)

def dumpAsPCD(filepath, points):
    with open(filepath, "w") as f:
        f.write(points2pcdstr(points))
