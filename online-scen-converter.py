import sys
import math
import random

if __name__ == "__main__":
    n = len(sys.argv)
    print(f"argc: {n}")
    if(n != 7):
        print("To call use python3 ***.py timeBetweenLow timeBetweenHigh agentsPerLow agentsPerHigh filenameIn filenameOut")
        print("For the spawn interval list the gaps in between agents, .25 = 4 agents per second; 4 = 1 agent per 4 time units")
    
    time_between_low = float(sys.argv[1])
    time_between_high = float(sys.argv[2])
    agents_per_low = float(sys.argv[3])
    agents_per_high = float(sys.argv[4])

    input_file = sys.argv[5]
    output_file = sys.argv[6]

    print(f"time between:[{time_between_low},{time_between_high}); agents per: [{agents_per_low},{agents_per_high}); i:{input_file}; o:{output_file};")

    file = open(input_file, "r")

    # Add every line in the input to an array minus the \n 
    base_scenario = []
    for line in file:
        base_scenario.append(line[:-1])
    file.close()

    # Iterate through each line adding a spawn time
    time = 0
    agents_per = math.floor(random.uniform(agents_per_low, agents_per_high))

    for line in range(1, len(base_scenario)):
        if agents_per == 0:
            agents_per = math.floor(random.uniform(agents_per_low, agents_per_high))
            time += math.floor(random.uniform(time_between_low, time_between_high))

        base_scenario[line] += f"\t {math.floor(time)}"
        agents_per -= 1
        #print(base_scenario[line])

    # Construct output 
    out = ""
    for line in base_scenario:
        out += line + "\n" 
    
    # Write to file
    file = open(output_file, "w")
    file.writelines(out)
    file.close()
