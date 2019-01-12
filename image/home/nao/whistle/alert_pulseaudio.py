"""
Python module to restart pulseaudio + dbus if the amixer program reports
the wrong number of capture channels online.
"""
import subprocess
import time

# To test this, change MAGIC_CAPTURE_COUNT to another number and "nao restart".
# Then change it back to 7 and again "nao restart" (nao_sync-ing if necessary).
#
# I don't like the magic number, but am unsure how else to detect that we have
# all 4 'Capture' channels online instead of just 2. See #740 for details.
MAGIC_CAPTURE_COUNT = 7


start = time.time()
cmd = subprocess.Popen(
    ['amixer'],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
)
cmd_out = cmd.communicate()[0]
if cmd_out.count('Capture') != MAGIC_CAPTURE_COUNT:
    print('WHISTLES WILL NOT BE HEARD!')
    # Turned off as this does not work when libagent/naoqi is starting...
    # and don't want to test independently, but it might work...
    #
    # subprocess.Popen(
    #     ['pulseaudio', '-k'],
    #     stdout=subprocess.PIPE,
    #     stderr=subprocess.PIPE,
    # )
    # print('Restarting pulseaudio as sound channels are probably messed up.')
    # print('See https://github.com/UNSWComputing/rUNSWift/issues/740')
    # subprocess.Popen(
    #     ['pulseaudio', '-k'],
    #     stdout=subprocess.PIPE,
    #     stderr=subprocess.PIPE,
    # )
    # print('Killed pulseaudio. Restarting DBus may take 15-30 seconds...')
    # subprocess.Popen(
    #     ['pulseaudio', '--start'],
    # # This also fixed another issue on 4 robots in the lab,
    # # but did not work within libagent... and may have caused other issues
    # #    ['sudo', '/etc/init.d/dbus', 'restart'],
    #     stdout=subprocess.PIPE,
    #     stderr=subprocess.PIPE,
    # )
    # print('Completed restarting pulseaudio and DBus')

    # Not particularly special, but too low and
    # 'nao stop; naoqi -d' complains a lot more at a terminal on the robot
    # SLEEP_FOR = 2
    # time.sleep(SLEEP_FOR)

    taken = time.time() - start

    # Construct a non-zero exit code
    exit(int(taken) % 254 + 1)

exit(0)
