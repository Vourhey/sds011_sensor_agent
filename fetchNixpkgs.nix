{ rev    ? "cb5daa39c2fbd855b2d1567074eab2fc4ff51fc8"             # The Git revision of nixpkgs to fetch
, sha256 ? "1ljbqd580xgv5zrdbdmdl0fwr3lmq5mhrh8pybh0s8gmxn9mnk4c" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
