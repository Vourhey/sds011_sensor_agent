# SDS011 Sensor Agent

This package contains [Robonomics](https://robonomics.network/) agent for [SDS011](https://aqicn.org/sensor/sds011/) sensor

After launching it publishes measurements every `INTERVAL` seconds to the network as a [Result](https://wiki.robonomics.network/docs/market-messages/#result) message

## Get Package and Build

Assuming you are running the agent under [AIRA](https://wiki.robonomics.network/docs/aira-installation-on-vb/) do the following:

```
su liability && cd
git clone https://github.com/Vourhey/sds011_sensor_agent
cd sds011_sensor_agent
nix build -f release.nix
```

## Launch

In the package's directory:

```
source result/setup.bash
roslaunch sds011_sensor_agent agent.launch
```

## Options

* `interval` - specify interval in seconds, default is 300 seconds
* `port` - specify the port the sensor is connected to, default is `/dev/ttyUSB0`
* `work_time` - minutes of a working period for the sensor, default is 5
* `geo` - latitude and longitude in format `lat, lon`, default is empty
* `sentry` - [sentry](sentry.io) API string, default is empty

## As a Service

Put the following in `/etc/nixos/configuration.nix` and run `nixos-rebuld switch`

```
systemd.services.sds011 = {
   requires = [ "roscore.service" ];  
      after = ["roscore.service" ];
      wantedBy = [ "multi-user.target" ];
      environment.ROS_MASTER_URI =  "http://localhost:11311";
      script = ''
        source /var/lib/liability/sds011_sensor_agent/result/setup.bash \
        && roslaunch sds011_sensor_agent agent.launch 
      '';
      serviceConfig = {
        Restart = "on-failure";
        StartLimitInterval = 0;
        RestartSec = 60;
        User = "liability";
      };
    };
```


