#!/bin/bash

scens=$(ls gen | wc -l)
i=0
for scen in gen/*.scen; do
  scen_name=$(basename $scen .scen)

  pattern="([a-zA-Z0-9_\-]+)-([0-9]+)-([0-9]+)$"
  
  if [[ $scen_name =~ $pattern ]]; then
    mapname=${BASH_REMATCH[1]}
    scen_id=${BASH_REMATCH[2]}
    agents=${BASH_REMATCH[3]}

  else
    echo error: $scen_name does not match pattern $pattern
    exit 1
  fi

  map=$(ls maps | grep $mapname | head -n 1)
  echo $i / $scens : scenario = $scen_name, map = $map
  # ./run.sh bench $scen_name maps/$map
  (( i++ ))
done