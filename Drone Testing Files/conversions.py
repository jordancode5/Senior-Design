from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, mavutil, Vehicle
import math


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat**2) + (dlong**2)) * 1.113195e5

def yards_to_meters(yards):
    return yards * 0.9144

def plot_to_global_coords(home, x, y, theta):
    """
    * rotates the coordinates (y,x) by theta degrees
    * returns the rotated coordinates (x',y') to be used by goto() function
    """
    x = x - .5
    y = -(y - .5)
    theta = (90-theta) * (3.14159265359/180) # convert to radians
    x_prime = 10 * ( x * math.cos(theta) - y * math.sin(theta) )
    y_prime = 8 * ( x * math.sin(theta) + y * math.cos(theta) )
    return get_location_metres(home, y_prime, x_prime)  # y' is north, x' is east

def field_to_global_vel(x, y, theta):
    """
    Rotates the coordinates (x,y) by theta degrees
    x goes to the right relative to the drone's heading
    y goes up relative to the drone's heading
    x' goes north
    y' goes east
    """
    theta = theta * (3.14159265359/180) # convert to radians
    x_prime = ( x * math.cos(theta) - y * math.sin(theta) )
    y_prime = ( x * math.sin(theta) + y * math.cos(theta) )
    return (x_prime, y_prime)  # x' is north, y' is east

def coords_to_vel(coords):
    """
    function to convert camera coordinates to velocity
    """
    center_width = 18
    # calculate distance of target from center of frame
    dist_x = abs(coords[0]) - center_width
    dist_y = abs(coords[1]) - center_width

    #  if target is in view, move drone to keep target in center of frame
    is_offcenter = False
    x_vel = 0
    y_vel = 0
    vel_factor_x = round(.012 * dist_x,3)  # affects the speed of horizontal corrections depending on distance from center
    vel_factor_y = round(.012 * dist_y,3)  # affects the speed of vertical corrections depending on distance from center
    # move drone to keep target within 25 pixels from the center of the frame
    # IMPORTANT: x and y flipped for drone reference frame
    if (coords[0] > center_width):  # x coord is positive, move right (East) in y direction
        y_vel = vel_factor_x
        is_offcenter = True
    elif (coords[0] < -center_width):  # x coord is negative, move left (West) in -y direction
        y_vel = -vel_factor_x
        is_offcenter = True
    if (coords[1] > center_width):  # y coord is positive, move up (North) in x direction
        x_vel = vel_factor_y
        is_offcenter = True
    elif (coords[1] < -center_width):
        x_vel = -vel_factor_y
        is_offcenter = True
    
    return (x_vel, y_vel, is_offcenter)
