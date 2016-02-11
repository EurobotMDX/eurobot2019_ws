import math
import numpy as np

def convert_360_to_180_degrees(degrees_mode360):
    degrees_mode360 = degrees_mode360 % 360.0

    if degrees_mode360 <= 180:
        return degrees_mode360
    else:
        return -(360.0 - degrees_mode360)

def convert_360_to_180_radians(radians_mode2PI):
    radians_mode2PI = radians_mode2PI % (math.pi * 2.0)

    if radians_mode2PI <= math.pi:
        return radians_mode2PI
    else:
        return -((math.pi * 2.0) - radians_mode2PI)

def convert_180_to_360_degrees(degrees_mode180):
    if degrees_mode180 >= 0:
        return degrees_mode180
    else:
        return 360 + degrees_mode180

def convert_180_to_360_radians(radians_modePI):
    if radians_modePI >= 0:
        return radians_modePI
    else:
        return (math.pi * 2.0) + radians_modePI

def add_angles_in_180_mode_degrees(angle_deg_1, angle_deg_2):
    return convert_360_to_180_degrees(convert_180_to_360_degrees(angle_deg_1) + convert_180_to_360_degrees(angle_deg_2))

def get_modulus_360_degrees(angle_degrees):
    return angle_degrees % 360.0

def get_modulus_360_radians(angle_radians):
    return angle_radians % (2*math.pi)

def add_angles_in_180_mode_radians(angle_rad_1, angle_rad_2):
    return convert_360_to_180_radians(convert_180_to_360_radians(angle_rad_1) + convert_180_to_360_radians(angle_rad_2))

def test_function(function, param, expected_answer, failure_msg="excepted: {expected_answer}, but received: {result} for param: {param}", success_msg="Test successful"):
    result = function(param)

    if (result == expected_answer):
        print success_msg
    else:
        print failure_msg.format(expected_answer=expected_answer, result=result, param=param)


def test_function_with_params(function, params_n_expected_answers):
    #for index in range( min( len(expected_answers), len(params) ) ):
    for param_n_expected_answer in params_n_expected_answers:
        test_function(function, param_n_expected_answer[0], param_n_expected_answer[1])

# test_function_with_params(convert_360_to_180_degrees, [(0,0), (90, 90), (180, 180), (-180, 180), (360, 0), (270, -90), (720, 0), (540, 180)])
# test_function_with_params(convert_360_to_180_radians, [(0,0), (math.pi/2.0, math.pi/2.0), (math.pi, math.pi), (-math.pi, math.pi), (2*math.pi, 0), ((3.0/2.0)*math.pi, -(math.pi/2.0)), (math.pi*4, 0), (math.pi*2.5, math.pi/2.0)])
# test_function_with_params(convert_180_to_360_degrees, [(0,0), (90, 90), (180, 180), (-180, 180), (-90, 270)])
# test_function_with_params(convert_180_to_360_radians, [(0,0), (math.pi/2.0, math.pi/2.0), (math.pi, math.pi), (-math.pi, math.pi), (-math.pi/2.0, (3.0/2.0)*math.pi)])

# test_function_with_params(get_modulus_360_degrees, [(0,0), (90, 90), (180, 180), (720, 0)])
# test_function_with_params(get_modulus_360_radians, [(0,0), (math.pi/2.0, math.pi/2.0), (math.pi, math.pi), (4*math.pi, 0)])