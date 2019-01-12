#!/usr/bin/python
"""
Need to run this with sudo or it can't write /etc/wpa_supplicant/wpa_supplicant.conf
"""

from ConfigParser import ConfigParser
import os
import subprocess
import sys

TEMPLATE = """
ctrl_interface=/var/run/wpa_supplicant
ctrl_interface_group=0
ap_scan=1

network={
  ssid="%(ssid)s"
  scan_ssid=1
  key_mgmt=WPA-PSK
  psk="%(password)s"
}
"""
if len(sys.argv) < 2:
    config_path = os.path.join('home', 'nao', 'data', 'runswift.cfg')
    abs_path = '/' + config_path
    config = ConfigParser()
    config.read(abs_path)
    field = config.get('network', 'ssid')
    print('changeField.py will use the field SSID in {}'.format(abs_path))
    print('i.e. {}'.format(field))
else:
    field = sys.argv[1]

if len(field) == 1:
    # Special case single letter fields like 'A' to 'SPL_A'
    # for backwards compatibility
    ssid = 'SPL_%s' % field
else:
    ssid = field

if len(sys.argv) > 2:
    # Allow user-specified passwords
    password = sys.argv[2]
elif field == 'runswift':
    # Special case for runswift field for backwards compatibility
    password = 'runswift'
else:
    # Default shared SPL password
    password = 'Nao?!Nao?!'


print("Connecting to field %s" % ssid)

filledIn = TEMPLATE % {
    "ssid": ssid,
    "password": password,
}

print(filledIn)

open("/etc/wpa_supplicant/wpa_supplicant.conf", "w").write(filledIn)
subprocess.call(['/etc/init.d/runswiftwireless', 'restart'])
