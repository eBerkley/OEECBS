import sys
import math

if __name__ == "__main__":
    n = len(sys.argv)
    print(f"argc: {n}")
    if(n != 4):
        print("To call use python3 ***.py spawnInterval filenameIn filenameOut")
        print("For the spawn interval list the gaps in between agents, .25 = 4 agents per second; 4 = 1 agent per 4 time units")
    
    amt_per_second = float(sys.argv[1])
    input_file = sys.argv[2]
    output_file = sys.argv[3]

    print(f"a:{amt_per_second}; b:{input_file}; c:{output_file};")

    file = open(input_file, "r")

    # Add every line in the input to an array minus the \n 
    base_scenario = []
    for line in file:
        base_scenario.append(line[:-1])
    file.close()

    # Iterate through each line adding a spawn time
    count = 0
    for line in range(1, len(base_scenario)):
        base_scenario[line] += f"\t {math.floor(count)}"
        count += amt_per_second
        #print(base_scenario[line])

    # Construct output 
    out = ""
    for line in base_scenario:
        out += line + "\n" 
    
    # Write to file
    file = open(output_file, "w")
    file.writelines(out)
    file.close()
