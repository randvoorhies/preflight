#!/usr/bin/env python
import subprocess
import sys
import threading
import time

######################################################################
# Parameters for the pre-flight checklist
min_battery_level = 11.1
max_battery_level = 13.0
frame_rate = 60
pelican_cpu = 'asctec-atom'


######################################################################
class Command(object):
  def __init__(self, cmd):
    self.cmd = cmd
    self.process = None
    self.stdout = ""

  def run(self, timeout):
    def target():
      #self.process = subprocess.Popen(self.cmd, shell=True, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
      self.process = subprocess.Popen(self.cmd, shell=True)
      self.stdout, stderr = self.process.communicate()

    thread = threading.Thread(target=target)
    thread.start()

    thread.join(timeout)
    if thread.is_alive():
      self.process.kill()
      thread.join()
    return self.stdout

def runcommand(cmd, timeout=2):
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
  stdout = runcommand("rostopic  echo " + topic + " -n 1 -p")
  try:
    return stdout.split('\n')[1].split(',')[1]
  except:
    return ""

######################################################################
def getParam(param):
  return runcommand("rosparam get " + param)

######################################################################
def getPublisher(topic):
  stdout = runcommand("rostopic info " + topic)
  try:
    sublines = stdout[stdout.find('Publishers'):stdout.find('\n\nSubscribers')].split('\n')[1:]
    subs = []
    for subline in sublines:
      subs.append(subline.split('(')[1].split(')')[0])
    return subs
  except:
    return ''

######################################################################
def check(name, value, test):
  OK   =  '\033[92m' + 'OK' + '\033[0m'
  FAIL =  '\033[91m' + 'FAIL' + '\033[0m'

  checkstr = 'Checking ' + name + '...'
  for x in range(1, 30-len(checkstr)): checkstr += ' '
  print checkstr,
  sys.stdout.flush()
  success = False
  val = ""
  try:
    val     = value()
    success = test(val)
  except:
    pass

  if success:
    print OK,
  else:
    print FAIL,

  print ' (' + str(val) + ')'

######################################################################
check('Camera Publisher', lambda: getPublisher('/camera/image_raw'),                             lambda x: len(x) == 1 and x[0].find(pelican_cpu) > 0)
check('Sonar Publisher',  lambda: getPublisher('/sonar_range'),                                  lambda x: len(x) == 1 and x[0].find(pelican_cpu) > 0)
check('IMU Publisher',    lambda: getPublisher('/asctec/IMU_CALCDATA'),                          lambda x: len(x) == 1 and x[0].find(pelican_cpu) > 0)
check('Quatrotor Status', lambda: getValue("/asctec/LL_STATUS/status"),                          lambda x: x == '0')
check('Battery Level',    lambda: float(getValue("/asctec/LL_STATUS/battery_voltage_1"))/1000.0, lambda x: x > min_battery_level and x < max_battery_level)
check('Frame Rate Param', lambda: float(getParam('/camera/frame_rate')),                         lambda x: x == frame_rate)

