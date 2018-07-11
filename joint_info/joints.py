joints_dict = {
    # left
    'leg_b_l':{
        'parent':'base_link', 
        'child':'back_leg_left', 
        'xyz': [-68.00, 34.40, 33.80], 
        'axis': [1, 0, 0]
    },
    'leg_f_l':{
        'parent':'back_leg_left', 
        'child':'front_leg_left', 
        'xyz': [99.999, -127.661, -22.845], 
        'axis': [1, 0, 0]
    }, 
     'wheel_b_l':{
        'parent':'back_leg_left', 
        'child':'back_wheel_left', 
        'xyz': [126.994, 130.414, -85.529], 
        'axis': [1, 0, 0]
    },
     'wheel_m_l':{
        'parent':'front_leg_left', 
        'child':'middle_wheel_left', 
        'xyz': [135.999, -40.876, -93.783], 
        'axis': [1, 0, 0]
    },
     'wheel_f_l':{
        'parent':'front_leg_left', 
        'child':'front_wheel_left', 
        'xyz': [136.399, -212.358, -95.775], 
        'axis': [1, 0, 0]
    }, 
    # right
     'leg_b_r':{
        'parent':'base_link', 
        'child':'back_leg_right', 
        'xyz': [-68.00, 34.40, 33.80], 
        'axis': [1, 0, 0]
    },
     'leg_f_r':{
        'parent':'back_leg_right', 
        'child':'front_leg_right', 
        'xyz': [-99.999, -127.661, -22.845 ], 
        'axis': [1, 0, 0]
    }, 
     'wheel_b_r':{
        'parent':'back_leg_right', 
        'child':'back_wheel_right', 
        'xyz': [-131.994, 130.414, -85.529], 
        'axis': [1, 0, 0]
    },
     'wheel_m_r':{
        'parent':'front_leg_right', 
        'child':'middle_wheel_right', 
        'xyz': [-135.999, -40.876, -93.783], 
        'axis': [1, 0, 0]
    },
     'wheel_f_r':{
        'parent':'front_leg_right', 
        'child':'front_wheel_right', 
        'xyz': [-136.999, -212.358, -95.775], 
        'axis': [1, 0, 0]
    }
}