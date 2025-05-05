#!/bin/bash
# 0 = REPLAN SINGLE
# 1 = REPLAN SINGLE GROUP
# 2 = REPLAN ALL




scen_name=berlin-2
map=maps/Berlin_1_256.map

replan_type=2
targetReasoning=1

if [[ $replan_type = 0 ]]; then
  r_str="single"
elif [[ $replan_type = 1 ]]; then
  r_str="group"
else
  r_str="all"
fi

scen=gen/$scen_name.scen
screen=0
agents=60
bypass=1

mkdir -p out/$scen_name

run="./eecbs -m $map -a $scen"
run+=" -k $agents -t 60 --suboptimality=1.2"
run+=" --online=true --screen=$screen"

if [[ $targetReasoning = 0 ]]; then
  run+=" --targetReasoning=false"
fi
if [[ $bypass = 0 ]]; then
  run+=" --bypass=false"
fi

if [[ $1 = "bench" ]]; then
  make
  for i in $(seq 0 2); do 
    replan_type=$i

    if [[ $replan_type = 0 ]]; then
      r_str="single"
    elif [[ $replan_type = 1 ]]; then
      r_str="group"
    else
      r_str="all"
    fi

    $run -r $replan_type \
      --outputPaths=out/$scen_name/$r_str-paths.txt \
      -o out/$scen_name/$r_str-stats.csv
  done
  exit
fi

run+=" -r $replan_type \
      --outputPaths=out/$scen/$r_str-paths.txt \
      -o out/$scen/$r_str-stats.csv"

if [[ -z $1 ]]; then
  make && $run

elif [[ $1 = "gdb" ]]; then

  make && gdb --args $run

elif [[ $1 = "valgrind" ]]; then

  make && valgrind -s --track-origins=yes --leak-check=full --show-leak-kinds=all $run

fi

