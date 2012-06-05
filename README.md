# preflight
##### A framework for creating automated preflight checklists for robots running ROS.

Example preflight checklist:

    from preflight import *
    check('Camera Publisher', lambda: getPublisher('/camera/image_raw'),       lambda x: x[0].find('myrobot') > 0)
    check('Sonar Publisher',  lambda: getPublisher('/sonar_range'),            lambda x: x[0].find('myrobot') > 0)
    check('IMU Publisher',    lambda: getPublisher('/asctec/IMU_CALCDATA'),    lambda x: x[0].find('myrobot') > 0)
    check('Quatrotor Status', lambda: getValue("/robotstatus/status"),         lambda x: x == '1')
    check('Battery Level',    lambda: float(getValue("/robotstatus/battery")), lambda x: x > 11.0) 
    check('Frame Rate Param', lambda: float(getParam('/camera/frame_rate')),   lambda x: x == frame_rate)

Example output:

    Checking Camera Publisher...  OK   ([http://myrobot:59209/])
    Checking Sonar Publisher...   OK   ([http://myrobot:59210/])
    Checking IMU Publisher...     OK   ([http://myrobot:59211/])
    Checking Robot Status...      OK   (1)
    Checking Battery Level...     FAIL (10.8)
    Checking Frame Rate Param...  OK   (60.0)