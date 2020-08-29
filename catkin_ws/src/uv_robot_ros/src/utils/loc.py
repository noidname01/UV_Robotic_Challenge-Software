import math

def get_location(pixel, distance):
    '''
    input: 
        pixel: (int, int), the location of this pixel in the picture
        distance: double, the distance from the camera to the thing (detected by the camera)
    output:
        loc: (double, double, double), the location of this pixel, given the origin = (0, 0)
    '''
    x, y = pixel[0], pixel[1]
    if x > 320 and y > 240:
        x = x - 320
        y = y - 240
        theta = math.atan(x/(320*math.sin(degree_to_arc(43)/math.cos(degree_to_arc(43)))))
        phi = math.atan(y/(240*math.sin(degree_to_arc(28.5)/math.cos(degree_to_arc(28.5)))))
        r = math.sqrt(pow(distance, 2)/(pow(1/math.cos(theta), 2)+pow(math.sin(phi)/math.cos(phi), 2)))
        a = r * math.sin(theta) / math.cos(theta)
        b = r
        c = r * math.sin(phi) / math.cos(phi)
        return (a, b, c)
    elif x < 320 and y > 240:
        x = 320 - x
        y = y - 240
        theta = math.atan(x/(320*math.sin(degree_to_arc(43)/math.cos(degree_to_arc(43)))))
        phi = math.atan(y/(240*math.sin(degree_to_arc(28.5)/math.cos(degree_to_arc(28.5)))))
        r = math.sqrt(pow(distance, 2)/(pow(1/math.cos(theta), 2)+pow(math.sin(phi)/math.cos(phi), 2)))
        a = r * math.sin(theta) / math.cos(theta)
        b = r
        c = r * math.sin(phi) / math.cos(phi)
        return (-a, b, c)
    elif x < 320 and y < 240:
        x = 320 - x
        y = 240 - y
        theta = math.atan(x/(320*math.sin(degree_to_arc(43)/math.cos(degree_to_arc(43)))))
        phi = math.atan(y/(240*math.sin(degree_to_arc(28.5)/math.cos(degree_to_arc(28.5)))))
        r = math.sqrt(pow(distance, 2)/(pow(1/math.cos(theta), 2)+pow(math.sin(phi)/math.cos(phi), 2)))
        a = r * math.sin(theta) / math.cos(theta)
        b = r
        c = r * math.sin(phi) / math.cos(phi)
        return (-a, b, -c)
    elif x > 320 and y > 240:
        x = x - 320
        y = 240 - y
        theta = math.atan(x/(320*math.sin(degree_to_arc(43)/math.cos(degree_to_arc(43)))))
        phi = math.atan(y/(240*math.sin(degree_to_arc(28.5)/math.cos(degree_to_arc(28.5)))))
        r = math.sqrt(pow(distance, 2)/(pow(1/math.cos(theta), 2)+pow(math.sin(phi)/math.cos(phi), 2)))
        a = r * math.sin(theta) / math.cos(theta)
        b = r
        c = r * math.sin(phi) / math.cos(phi)
        return (a, b, -c)


def get_absolute_location(pos, direc, loc):
    '''
    input:
        pos: (double, double, double), the location of the robot
        direc: double, the direction of the camera (or the direction of the robot) in degree
        loc: (double, double, double), the location of this pixel, given the origin = (0, 0)
    output:
        abs_loc: (double, double, double), the location of this pixel, given the origin = pos
    '''    
    arc = degree_to_arc(direc)
    x = loc[0]
    y = loc[1]
    z = loc[2]
    a = x * math.cos(arc) - y * math.sin(arc)
    b = x * math.sin(arc) + y * math.cos(arc)
    c = z
    return (a+pos[1], b+pos[2], c+pos[3])


def degree_to_arc(degree):
    return degree*2*math.pi/360

def polar_to_cartesian(dis, direc):
    arc = direc*2*math.pi/360
    x_comp = -dis*math.sin(arc)   #to be checked
    y_comp = dis*math.cos(arc)
    return x_comp, y_comp

if __name__=='__main__':
    assert degree_to_arc(360)==180*2*math.pi/360, 'ValueError from degree_to_arc'
