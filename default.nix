{ stdenv
, mkRosPackage
, robonomics_comm-nightly
, python3Packages
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "sds011_sensor_agent";
  version = "0.2.0";

  src = ./.;

  propagatedBuildInputs = [
    robonomics_comm-nightly
    python3Packages.pyserial
    python3Packages.requests
    python3Packages.sentry-sdk
    python3Packages.netifaces
  ];

  meta = with stdenv.lib; {
    description = "Agent that offers data from sensors";
    homepage = http://github.com/vourhey/sen0233_sensor_agent;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
