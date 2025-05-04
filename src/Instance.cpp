#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include"Instance.h"

int RANDOM_WALK_STEPS = 100000;

Instance::Instance(const string& map_fname, const string& agent_fname, 
	int num_of_agents, int num_of_rows, int num_of_cols, int num_of_obstacles, int warehouse_width):
	map_fname(map_fname), agent_fname(agent_fname), num_of_agents(num_of_agents)
	{
	simulator_time = 0;
	bool succ = loadMap();
	if (!succ)
	{
		if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 && 
			num_of_obstacles < num_of_rows * num_of_cols) // generate random grid
		{
			generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_obstacles);
			saveMap();
		}
		else
		{
			cerr << "Map file " << map_fname << " not found." << endl;
			exit(-1);
		}
	}

	succ = loadAgents();
	if (!succ)
	{
		if (num_of_agents > 0)
		{
			generateRandomAgents(warehouse_width);
			saveAgents();
		}
		else
		{
			cerr << "Agent file " << agent_fname << " not found." << endl;
			exit(-1);
		}
	}
}

struct agent {
    int start_row;
	int start_col;
    int goal_row;
    int goal_col;
	int spawn_time;
};

/* The idea is that this function will add n amounts of agents to a random position
 * On the current map. 
 * PARAMS: amt_agents is the number of agents to add
 */
void Instance::AddRandAgents(int amt_agents){
	cout << "There are currently: " << num_of_agents << " agents" << endl;
	vector<bool> starts(map_size, false);
	vector<bool> goals(map_size, false);

	// Mark existing start and goal locations
	for (int loc : agent_location){
		if (loc >= 0) starts[loc] = true;
	}
	for (int loc : goal_locations){
		if (loc >= 0) goals[loc] = true;
	}

	// Resize vectors to accommodate new agents
	agent_location.resize(num_of_agents + amt_agents, -1);
	goal_locations.resize(num_of_agents + amt_agents, -1);

	// Choose random start locations
	int k = num_of_agents;
	while ( k < num_of_agents + amt_agents){
		// Get a random position and convert it to the standard 1d format
		int x = rand() % num_of_rows, y = rand() % num_of_cols;
		int start = linearizeCoordinate(x, y);

		// if we fail to place an agent then just move on and try again
		if (my_map[start] || starts[start]){
			continue;
		}

		// update start
		agent_location[k] = start;
		starts[start] = true;

		// find goal
		bool flag = false;
		int goal = rand() % map_size; // randomWalk(start, RANDOM_WALK_STEPS);
		while (my_map[goal] || goals[goal]){
			goal = rand() % map_size; // randomWalk(goal, 1);
		}

		//update goal
		goal_locations[k] = goal;
		goals[goal] = true;

		k++;
	}
	num_of_agents += amt_agents;
}

void Instance::AddSingleAgent(const Agent& agent){
	int linear_start = agent.start_locaton; 
	int linear_goal = agent.goal_location; 

	cout << "There are currently " << num_of_agents << " agents" << endl;

	// Mark existing start and goal locations
	// for (int loc : agent_location){
	// 	// cout << "loc: " << loc << endl;
	// 	if (loc == linear_start){
	// 		cout << "Agent spawn location collided: " << loc << endl;
	// 		// return;
	// 	}
	// }
	// Check for conflictics 
	if(my_map[linear_start]){
		cout << "agent collided with map" << endl;
		//return;
	}

	num_of_agents += 1;
	// Resize vectors to accommodate new agents
	agent_location.resize(num_of_agents, -1);
	goal_locations.resize(num_of_agents, -1);

	agent_location[num_of_agents-1] = linear_start;
	goal_locations[num_of_agents-1] = linear_goal;
	cout << "added agent " << agent << endl;
}

/* The idea is that whatever function is running instance will generate a list of places for each agent to move before calling this function
* It is important that each position has also been linearized.
*/
void Instance::timeStep(const vector<int>& moves, int batch){
	
	// Ensure moves vector matches number of active agents
	if (moves.size() != static_cast<size_t>(num_of_agents + agent_sets[batch].second)) {
		cerr << "Error: moves vector size (" << moves.size() << ") does not match number of active agents (" << num_of_agents << ")" << endl;
		for (auto move : moves) 
			cerr << move << " "; 
		cerr  << endl;
		exit(-1);
	}	
	int delta = agent_sets[batch].first - simulator_time;
	simulator_time = agent_sets[batch].first;
	cout << "timestep: delta, simulator_time: " << delta << ", " << simulator_time << endl;	

	// iterate through every existing agent
	for (int i = 0; i < num_of_agents; i++) {
        // Once it works then we can update this to remove agents 
        if (agent_location[i] == goal_locations[i]){
            continue;
		}

		// bounds check and update the agent
		if ( validMove(agent_location[i], moves[i], delta) && !my_map[moves[i]]) {
			agent_location[i] = moves[i];
		} else {
			cout << endl << "TimeStep( {";
				for (auto m : moves) cout << m << " ";
				cout << "}, " << simulator_time <<
			"): Invalid move requested: " << moves[i] << endl;
		}
	}	
  
	
	int num_agents = agent_sets[batch].second;


	// See if we need to add any new agents
	for(auto& agent : agent_list){
		if(agent.spawn_time == simulator_time){
			AddSingleAgent(agent);
		}
	}

	
}
/* As the name implies this function is meant to remove one agent
 * PARAMS: index is the index of the agent you want to remove in start_location and goal_locations
 */
void Instance::removeAgent(int index){
	// Basic bounds checking
	if (index < 0 || index >= num_of_agents){
		cout << "Attempted to remove an invalid agent number" << endl;
        return;
	}

	agent_location.erase(agent_location.begin() + index);
	goal_locations.erase(goal_locations.begin() + index);
	num_of_agents -= 1;
}

int Instance::randomWalk(int curr, int steps) const
{
	for (int walk = 0; walk < steps; walk++)
	{
		list<int> l = getNeighbors(curr);
		vector<int> next_locations(l.cbegin(), l.cend());
		auto rng = std::default_random_engine{};
		std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
		for (int next : next_locations)
		{
			if (validMove(curr, next))
			{
				curr = next;
				break;
			}
		}
	}
	return curr;
}

void Instance::generateRandomAgents(int warehouse_width)
{
	cout << "Generate " << num_of_agents << " random start and goal locations " << endl;
	vector<bool> starts(map_size, false);
	vector<bool> goals(map_size, false);
	agent_location.resize(num_of_agents);
	goal_locations.resize(num_of_agents);

	if (warehouse_width == 0)//Generate agents randomly
	{
		// Choose random start locations
		int k = 0;
		while ( k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % num_of_cols;
			int start = linearizeCoordinate(x, y);
			if (my_map[start] || starts[start])
				continue;
				
			// update start
			agent_location[k] = start;
			starts[start] = true;

			// find goal
			bool flag = false;
			int goal = rand() % map_size; // randomWalk(start, RANDOM_WALK_STEPS);
			while (my_map[goal] || goals[goal])
				goal = rand() % map_size; // randomWalk(goal, 1);

			//update goal
			goal_locations[k] = goal;
			goals[goal] = true;

			k++;
		}
	}
	else //Generate agents for warehouse scenario
	{
		// Choose random start locations
		int k = 0;
		while (k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % warehouse_width;
			if (k % 2 == 0)
				y = num_of_cols - y - 1;
			int start = linearizeCoordinate(x, y);
			if (starts[start])
				continue;
			// update start
			agent_location[k] = start;
			starts[start] = true;

			k++;
		}
		// Choose random goal locations
		k = 0;
		while (k < num_of_agents)
		{
			int x = rand() % num_of_rows, y = rand() % warehouse_width;
			if (k % 2 == 1)
				y = num_of_cols - y - 1;
			int goal = linearizeCoordinate(x, y);
			if (goals[goal])
				continue;
			// update goal
			goal_locations[k] = goal;
			goals[goal] = true;
			k++;
		}
	}
}

bool Instance::validMove(int curr, int next) const
{
	if (next < 0 || next >= map_size)
		return false;
	if (my_map[next])
		return false;
	return getManhattanDistance(curr, next) < 2;
}

bool Instance::validMove(int curr, int next, int time) const
{
	if (next < 0 || next >= map_size)
		return false;
	if (my_map[next])
		return false;
	return getManhattanDistance(curr, next) < time + 1;
}

bool Instance::addObstacle(int obstacle)
{
	if (my_map[obstacle])
		return false;
	my_map[obstacle] = true;
	int obstacle_x = getRowCoordinate(obstacle);
	int obstacle_y = getColCoordinate(obstacle);
	int x[4] = { obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1 };
	int y[4] = { obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y };
	int start = 0;
	int goal = 1;
	while (start < 3 && goal < 4)
	{
		if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 || y[start] >= num_of_cols 
			|| my_map[linearizeCoordinate(x[start], y[start])])
			start++;
		else if (goal <= start)
			goal = start + 1;
		else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 || y[goal] >= num_of_cols 
			|| my_map[linearizeCoordinate(x[goal], y[goal])])
			goal++;
		else if (isConnected(linearizeCoordinate(x[start], y[start]), linearizeCoordinate(x[goal], y[goal]))) // cannot find a path from start to goal 
		{
			start = goal;
			goal++;
		}
		else
		{
			my_map[obstacle] = false;
			return false;
		}
	}
	return true;
}

bool Instance::isConnected(int start, int goal)
{
	std::queue<int> open;
	vector<bool> closed(map_size, false);
	open.push(start);
	closed[start] = true;
	while (!open.empty())
	{
		int curr = open.front(); open.pop();
		if (curr == goal)
			return true;
		for (int next : getNeighbors(curr))
		{
			if (closed[next])
				continue;
			open.push(next);
			closed[next] = true;
		}
	}
	return false;
}

void Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles)
{
	cout << "Generate a " << rows << " x " << cols << " grid with " << obstacles << " obstacles. " << endl;
	int i, j;
	num_of_rows = rows + 2;
	num_of_cols = cols + 2;
	map_size = num_of_rows * num_of_cols;
	my_map.resize(map_size, false);
	// Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/

	// add padding
	i = 0;
	for (j = 0; j<num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	i = num_of_rows - 1;
	for (j = 0; j<num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = 0;
	for (i = 0; i<num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = num_of_cols - 1;
	for (i = 0; i<num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;

	// add obstacles uniformly at random
	i = 0;
	while (i < obstacles)
	{
		int loc = rand() % map_size;
		if (addObstacle(loc))
		{
			printMap();
			i++;
		}
	}
}

bool Instance::loadMap()
{
	using namespace boost;
	using namespace std;
	ifstream myfile(map_fname.c_str());
	if (!myfile.is_open())
		return false;
	string line;
	tokenizer< char_separator<char> >::iterator beg;
	getline(myfile, line);
	if (line[0] == 't') // Nathan's benchmark
	{
		char_separator<char> sep(" ");
		getline(myfile, line);
		tokenizer< char_separator<char> > tok(line, sep);
		beg = tok.begin();
		beg++;
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		getline(myfile, line);
		tokenizer< char_separator<char> > tok2(line, sep);
		beg = tok2.begin();
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
		getline(myfile, line); // skip "map"
	}
	else // my benchmark
	{
		char_separator<char> sep(",");
		tokenizer< char_separator<char> > tok(line, sep);
		beg = tok.begin();
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
	}
	map_size = num_of_cols * num_of_rows;
	my_map.resize(map_size, false);
	// read map (and start/goal locations)
	for (int i = 0; i < num_of_rows; i++) {
		getline(myfile, line);
		for (int j = 0; j < num_of_cols; j++) {
			my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
		}
	}
	myfile.close();

	// initialize moves_offset array
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/
	return true;
}


void Instance::printMap() const
{
	vector<vector<string>> map(num_of_rows, vector<string>(num_of_cols));

	for (int i = 0; i< num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (this->my_map[linearizeCoordinate(i, j)])
				// cout << '@';
				map[i][j] = "@ ";
			else
				// cout << '.';
				map[i][j] = ". ";
		}
		// cout << endl;
	}

	for (int ag = 0; ag < num_of_agents; ag++)
	{
		int loc = agent_location[ag];
		int row = getRowCoordinate(loc);
		int col = getColCoordinate(loc);
		if (map[row][col] != ". ") {
			cout << "err: agent" << ag << " at " << "( " << row << ", " << col << ") collided with agent " << map[row][col] << endl;
			continue;
		}
		// assert(map[row][col] == ". ");
		map[row][col] = std::to_string(ag);
		if (map[row][col].length() == 1)
			map[row][col] = " " + map[row][col];
	}
	
	cout << "    ";
	for (int j = 0; j < num_of_cols; j++)
	{
		if (j < 10)
			cout << " " << j;
		else
			cout << j;
		cout << " ";
	}
	cout << endl;
	for (int i = 0; i < num_of_rows; i++)
	{
		if (i < 10)
			cout << " " << i << "  ";
		else
			cout << i << "  ";

		for (int j = 0; j < num_of_cols; j++)
		{
			cout << map[i][j] << " ";
		}
		cout << endl;
	}
}


void Instance::saveMap() const
{
	ofstream myfile;
	myfile.open(map_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the map to " << map_fname << endl;
		return;
	}
	myfile << num_of_rows << "," << num_of_cols << endl;
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (my_map[linearizeCoordinate(i, j)])
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}

std::ostream& operator<<(std::ostream& os, const Instance::Agent& agent) {
	os << "Agent " << agent.index << ": S=(" << agent.start_locaton << ") ; G=(" << agent.goal_location << "), t=" << agent.spawn_time;	
	return os;
}

bool Instance::loadAgents()
{
	using namespace std;
	using namespace boost;

	string line;
	ifstream myfile (agent_fname.c_str());
	if (!myfile.is_open()) 
	return false;

	getline(myfile, line);
	
	if (line[0] == 'v') // Nathan's benchmark
	{
		if (num_of_agents == 0)
		{
			cerr << "The number of agents should be larger than 0" << endl;
			exit(-1);
		}
		agent_location.resize(num_of_agents);
		goal_locations.resize(num_of_agents);
		char_separator<char> sep("\t");
		for (int i = 0; i < num_of_agents; i++)
		{
			getline(myfile, line);
			tokenizer< char_separator<char> > tok(line, sep);
			tokenizer< char_separator<char> >::iterator beg = tok.begin();
			beg++; // skip the first number
			beg++; // skip the map name
			beg++; // skip the columns
			beg++; // skip the rows
				   // read start [row,col] for agent i

			struct Agent new_agent = {.index = i};

			int col = atoi((*beg).c_str());
			beg++;
			int row = atoi((*beg).c_str());
			//agent_location[i] = linearizeCoordinate(row, col);
			//new_agent.start_locaton = agent_location[i];
			new_agent.start_locaton = linearizeCoordinate(row, col);
			// read goal [row,col] for agent i
			beg++;
			col = atoi((*beg).c_str());
			beg++;
			row = atoi((*beg).c_str());
			goal_locations[i] = linearizeCoordinate(row, col);
			new_agent.goal_location = goal_locations[i];
			beg++; // skip y coordinate
			beg++; // skip the optimal length
			// add the spawn in time
			int spawn_time = atoi((*beg).c_str());
			new_agent.spawn_time = spawn_time;
			if(new_agent.spawn_time == 0){
				agent_location[i] = new_agent.start_locaton;
			}
			agent_list.push_back(new_agent);
			cout << new_agent << endl;
		}
		// cout << agent_list[2].spawn_time << endl;
		// cout << agent_list[2].start_locaton << endl;
		// cout << agent_list[2].goal_location << endl;

		// cout << agent_list[1].spawn_time << endl;
		// cout << agent_list[1].start_locaton << endl;
		// cout << agent_list[1].goal_location << endl;


		this->initAgentSets();
	}
	else // My benchmark
	{
		cout << "in Instance.cpp using the other benchmark. unsupported" << endl;
		char_separator<char> sep(",");
		tokenizer< char_separator<char> > tok(line, sep);
		tokenizer< char_separator<char> >::iterator beg = tok.begin();
		num_of_agents = atoi((*beg).c_str());
		agent_location.resize(num_of_agents);
		goal_locations.resize(num_of_agents);
		for (int i = 0; i<num_of_agents; i++)
		{
			getline(myfile, line);
			tokenizer< char_separator<char> > col_tok(line, sep);
			tokenizer< char_separator<char> >::iterator c_beg = col_tok.begin();
			pair<int, int> curr_pair;
			// read start [row,col] for agent i
			int row = atoi((*c_beg).c_str());
			c_beg++;
			int col = atoi((*c_beg).c_str());
			agent_location[i] = linearizeCoordinate(row, col);
			// read goal [row,col] for agent i
			c_beg++;
			row = atoi((*c_beg).c_str());
			c_beg++;
			col = atoi((*c_beg).c_str());
			goal_locations[i] = linearizeCoordinate(row, col);
		}
	}
	myfile.close();
	return true;

}


void Instance::printAgents() const
{
  for (int i = 0; i < num_of_agents; i++) 
  {
    cout << "Agent" << i << " : S=(" << getRowCoordinate(agent_location[i]) << "," << getColCoordinate(agent_location[i]) 
				<< ") ; G=(" << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << ")" << endl;
  }
}


void Instance::saveAgents() const
{
  ofstream myfile;
  myfile.open(agent_fname);
  if (!myfile.is_open())
  {
	  cout << "Fail to save the agents to " << agent_fname << endl;
	  return;
  }
  myfile << num_of_agents << endl;
  for (int i = 0; i < num_of_agents; i++)
    myfile << getRowCoordinate(agent_location[i]) << "," << getColCoordinate(agent_location[i]) << ","
           << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << "," << endl;
  myfile.close();
}


list<int> Instance::getNeighbors(int curr) const
{
	list<int> neighbors;
	int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}
