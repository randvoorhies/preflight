#!/usr/bin/env python
"""A framework for creating simple 'preflight checklists' for robots running ROS.

Randolph Voorhies (voorhies@usc.edu), 2012
"""
import subprocess
import sys
import threading
import time

######################################################################
def __runcommand(cmd, timeout=2):
  p = subprocess.Popen(cmd, shell=True, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
  endtime = time.time() + timeout
  while(time.time() < endtime and p.poll() is None):
    time.sleep(0.1)

  if p.poll() is None:
    p.kill()
    return ""
  else:
    stdout, stderr = p.communicate()
    return stdout

######################################################################
def getValue(topic):
  """Returns the latest value of a given message.
  
  This is useful for checking things like battery levels, status messages, etc.

  Example:
    preflight.check('Battery Level', lambda: preflight.getValue("/robotstatus/battery_voltage"), lambda x: float(x) > 11.0)
  """
  stdout = __runcommand("rostopic  echo " + topic + " -n 1 -p")
  try:
    return stdout.split('\n')[1].split(',')[1]
  except:
    return ""

######################################################################
def getParam(param):
  """Returns the value of a given param on ROS's param server.

  Example:
    preflight.check('Frame Rate Param', lambda: preflight.getParam('/camera/frame_rate'), lambda x: float(x) == 60.0)
  """
  return __runcommand("rosparam get " + param)

######################################################################
def getPublisher(topic):
  """Returns a list of publishers of the given topic.

  This is useful to make sure that all of your sensors are actually running on your robot, rather
  than from a simulator.

  Example:
    preflight.check('Camera Publisher', lambda: preflight.getPublisher('/camera/image_raw'), lambda x: len(x) == 1 and x[0].find('robot-hostname') > 0)
  """
  stdout = __runcommand("rostopic info " + topic)
  try:
    sublines = stdout[stdout.find('Publishers'):stdout.find('\n\nSubscribers')].split('\n')[1:]
    subs = []
    for subline in sublines:
      subs.append(subline.split('(')[1].split(')')[0])
    return subs
  except:
    return ''

######################################################################
def check(name, value_lambda, test_lambda):
  """Run a single pre-flight check.

  Arguments:
  name -- A short description of the test being performed. This is only used for printing,
          so try to keep it under 30 characters.
  value_lambda -- A lambda function which takes no arguments, and returns the value 
                  being checked. See the documentation for getValue, getParam, and
                  getPublisher for more info.
  test_lambda -- A lambda function which takes a single argument (the result of value_lambda),
                 and returns either true (if the value is acceptable) or false (if the value
                 fails the pre-flight check).

  Examples:
    preflight.check('Battery Level',    lambda: preflight.getValue("/robotstatus/battery_voltage"), lambda x: float(x) > 11.0)
    preflight.check('Frame Rate Param', lambda: preflight.getParam('/camera/frame_rate'),           lambda x: float(x) == 60.0)
    preflight.check('Camera Publisher', lambda: preflight.getPublisher('/camera/image_raw'),        lambda x: len(x) == 1 and x[0].find('robot-hostname') > 0)
  """
  OK   =  '\033[92m' + 'OK  ' + '\033[0m'
  FAIL =  '\033[91m' + 'FAIL' + '\033[0m'

  checkstr = 'Checking ' + name + '...'
  for x in range(1, 30-len(checkstr)): checkstr += ' '
  print checkstr,
  sys.stdout.flush()
  success = False
  val = ""
  try:
    val     = value_lambda()
    success = test_lambda(val)
  except:
    pass

  if success:
    print OK,
  else:
    print FAIL,

  print ' (' + str(val) + ')'
