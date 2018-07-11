"""
 [summary]
 Write the joints information of your robot.
 Keep in mind that your the name of base link must be "base_link".
 Remaining code is just an example. Change the code as you like.
"""

joints_dict = {
    'leg_l_joint':{
        'parent':'base_link', 
        'child':'leg_l', 
        'xyz': [-68.00, 34.40, 33.80], 
        'axis': [1, 0, 0]
    },
    'leg_r_joint':{
        'parent':'base_link', 
        'child':'leg_r', 
        'xyz': [99.999, -127.661, -22.845], 
        'axis': [1, 0, 0]
    },
     'wheel_l':{
        'parent':'leg_l', 
        'child':'wheel_l', 
        'xyz': [99.999, -127.661, -22.845], 
        'axis': [1, 0, 0]
    },
    'wheel_r':{
        'parent':'leg_r', 
        'child':'wheel_r', 
        'xyz': [99.999, -127.661, -22.845], 
        'axis': [1, 0, 0]
    }
}