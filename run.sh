#!/bin/bash
# 0 = REPLAN SINGLE
# 1 = REPLAN SINGLE GROUP
# 2 = REPLAN ALL
screen=0
replan_type=2
del=1

all_suboptimality=1.0001
single_suboptimality=1.2
group_suboptimality=1.2

# scen_name=berlin-5-15-1-5
# map=maps/Berlin_1_256.map
# agents=200

# scen_name=berlin-3_5-50-5-10
# map=maps/Berlin_1_256.map
# agents=25

# scen_name=berlin-4_5-20-3-10
# map=maps/Berlin_1_256.map
# agents=151

# scen_name=brc202d-3_5-50-5-10
# map=maps/brc202d.map
# agents=200

# scen_name=brc202d-4_5-50-5-10
# map=maps/brc202d.map
# agents=200
# scen_name=brc202d-5_5-50-5-10
# map=maps/brc202d.map

scen_name=$2
map=$3

agents=400

if [[ $del = 1 ]]; then
  rm -rf out/$scen_name
fi

targetReasoning=0

if [[ $replan_type = 0 ]]; then
  r_str="single"
  suboptimality=$single_suboptimality
elif [[ $replan_type = 1 ]]; then
  r_str="group"
  suboptimality=$group_suboptimality
else
  r_str="all"
  suboptimality=$all_suboptimality
fi

scen=gen/$scen_name.scen


bypass=1

mkdir -p out/$scen_name

run="./eecbs -m $map -a $scen"
# run+=" -k $agents -t 50 --suboptimality=1.0001"
run+=" -k $agents -t 100"
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
      suboptimality=$single_suboptimality
    elif [[ $replan_type = 1 ]]; then
      r_str="group"
      suboptimality=$group_suboptimality  
    else
      r_str="all"
      suboptimality=$all_suboptimality  
    fi

    if [[ $screen -ge 1 ]]; then
      $run -r $replan_type --suboptimality=$suboptimality \
        --outputPaths=out/$scen_name/$r_str-paths.txt \
        -o out/$scen_name/$r_str-stats.csv > $r_str.txt
    else
      $run -r $replan_type --suboptimality=$suboptimality \
        --outputPaths=out/$scen_name/$r_str-paths.txt \
        -o out/$scen_name/$r_str-stats.csv
    fi

  done
  exit
fi

run+=" -r $replan_type --suboptimality=$suboptimality \
      --outputPaths=out/$scen_name/$r_str-paths.txt \
      -o out/$scen_name/$r_str-stats.csv"

if [[ -z $1 ]]; then
  make && $run

elif [[ $1 = "gdb" ]]; then

  make && gdb --args $run

elif [[ $1 = "valgrind" ]]; then

  make && valgrind -s --track-origins=yes --leak-check=full --show-leak-kinds=all $run

fi

