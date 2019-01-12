# This file sets the system path so that behaviours can be simulated.
#
# As it stands, rUNSWift behaviours use Boost Python. This uses the Python
# interpreter in the 'ctc' folder, and injects some Pyhton modules generated
# on the fly from C++ (the 'robot' module).
#
# The problem with running this on a PC is that we don't have the correct
# PYTHONPATH to import our behaviours / modules from the CTC folder. To solve
# this, we add the CTC Python directories to our system path below. We also
# add our own Python installation in case CTC is missing a module that we need.
#
# Finally, we add rUNSWift behaviour folders so we can import them too.
import os
import sys

# If 'RUNSWIFT_CHECKOUT_DIR' exists as an environment variable, we are most
# likely running a simulation build (variable doesn't exist on Naos).
if 'RUNSWIFT_CHECKOUT_DIR' in os.environ:
    # Reset path
    sys.path = []

    for p in ['',
              '/usr/lib/python2.7',
              '/usr/lib/python2.7/plat-x86_64-linux-gnu',
              '/usr/lib/python2.7/lib-tk',
              '/usr/lib/python2.7/lib-old',
              '/usr/lib/python2.7/lib-dynload',
              '/usr/local/lib/python2.7/dist-packages',
              '/usr/lib/python2.7/dist-packages',
              '/usr/lib/python2.7/dist-packages/PILcompat',
              '/usr/lib/python2.7/dist-packages/gtk-2.0',
              '/usr/lib/pymodules/python2.7',
              '/usr/lib/python2.7/dist-packages/ubuntu-sso-client',
              '/usr/lib/python2.7/site-packages/']:
        # Add our own Python directory
        sys.path.append(p)
        # Add CTC python
        # NOTE numpy must be copied from robot to CTC
        sys.path.append(os.environ['RUNSWIFT_CHECKOUT_DIR'] +
                        '/ctc/sysroot_legacy' + p)

    # Add our behaviour folders so we can import them
    for p in ['/image/home/nao/data/behaviours/',
              '/build-relwithdebinfo/robot']:
        sys.path.append(os.environ['RUNSWIFT_CHECKOUT_DIR'] + p)
