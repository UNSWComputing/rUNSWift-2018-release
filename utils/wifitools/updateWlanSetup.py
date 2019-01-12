
import sys
ROBOTS_TO_IPS = {
   # Please specify a two-digit number, with a leading zero if necessary
   "robot1":   "00",
   "robot2":   "01",
   "robot3":   "02",
   "robot4":   "03",
   "robot5":   "04",
   "robot6":   "05",
}


# Accept a robot name.
if len(sys.argv) < 2:
   sys.exit("No robot given")

robotName = sys.argv[1]
if robotName not in ROBOTS_TO_IPS:
   sys.exit("Invalid robot name")

# Generate the filled in runswiftwireless file.
runswiftWirelessFinalise = "/tmp/runswiftwireless"
runswiftWirelessTemplate = "runswiftwireless_template"

template = open(runswiftWirelessTemplate, "r").read()
filledIn = template % {
   "playerIP": ROBOTS_TO_IPS[robotName]
}

output = open(runswiftWirelessFinalise, "w")
output.write(filledIn)
output.close()
# Sync it to the robot.

import subprocess
subprocess.Popen("scp %s root@%s.local:/etc/init.d/runswiftwireless" % (runswiftWirelessFinalise, robotName), shell=True)
print('Completed copy of updated runswiftwireless service to {}'
      .format(robotName))
print('You might wish to restart wifi (4 chest button presses) now')
