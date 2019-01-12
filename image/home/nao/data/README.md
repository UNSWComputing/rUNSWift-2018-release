# Nao Data Directory

This data directory is copied to the Nao as part of [`nao_sync`][nao-sync].


## Configuration files

### The `configs` directory

The `configs` directory contains a configuration file per robot hostname, with
robot-specific calibration settings, such as the player number and team.


### All robots `runswift.cfg`

The top-level `runswift.cfg` file contains many configuration
settings which apply to all robots, such as network settings.


### Speech volume `volumeinfo.dat`

The Nao's speech (and whistle detection) volume can be modified in 
`volumeinfo.dat`.


[nao-sync]: /bin/nao_sync
