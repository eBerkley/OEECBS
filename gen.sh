#!/bin/bash

rm -rf gen
mkdir -p gen

for scen in scens/*.scen; do
  scen_name=$(basename $scen .scen)
  
  for mapname in Berlin Boston brc202d den312d den520d empty-8 empty-16 empty-32 empty-48 ht_chantry ht_mansion lak303d lt_gallowstemplar maze-32 maze-128 orz900d ost003d Paris random-32 w_woundedcoast; do
    pattern="$mapname[0-9a-zA-Z_\-]+-([0-9]+)$"

    if [[ $scen_name =~ $pattern ]] && [[ ${BASH_REMATCH[1]} -le 10 ]]; then

      scen_id=${BASH_REMATCH[1]}
      map=$(ls maps | grep $mapname | head -n 1)
      echo $mapname-$scen_id = $map

      python3 online-scen-converter.py 5 50 1 5 $scen gen/$mapname-$scen_id-1.scen
      python3 online-scen-converter.py 5 50 10 15 $scen gen/$mapname-$scen_id-10.scen

    fi

  done
done

