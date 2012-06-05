#!/usr/bin/env python
import preflight

######################################################################
# Parameters for the pre-flight checklist
min_battery_level = 11.1
max_battery_level = 13.0
frame_rate        = 60
pelican_cpu       = 'asctec-atom'

######################################################################
preflight.check('Camera Publisher', lambda: preflight.getPublisher('/camera/image_raw'),                             lambda x: len(x) == 1 and x[0].find(pelican_cpu) > 0)
preflight.check('Sonar Publisher',  lambda: preflight.getPublisher('/sonar_range'),                                  lambda x: len(x) == 1 and x[0].find(pelican_cpu) > 0)
preflight.check('IMU Publisher',    lambda: preflight.getPublisher('/asctec/IMU_CALCDATA'),                          lambda x: len(x) == 1 and x[0].find(pelican_cpu) > 0)
preflight.check('Quatrotor Status', lambda: preflight.getValue("/asctec/LL_STATUS/status"),                          lambda x: x == '0')
preflight.check('Battery Level',    lambda: float(preflight.getValue("/asctec/LL_STATUS/battery_voltage_1"))/1000.0, lambda x: x > min_battery_level and x < max_battery_level)
preflight.check('Frame Rate Param', lambda: float(preflight.getParam('/camera/frame_rate')),                         lambda x: x == frame_rate)
