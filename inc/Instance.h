#pragma once
#include"common.h"


// Currently only works for undirected unweighted 4-nighbor grids
class Instance 
{
public:
	int num_of_cols;
	int num_of_rows;
	int map_size;
	int simulator_time;

	// enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size

	Instance(){}
	Instance(const string& map_fname, const string& agent_fname, 
		int num_of_agents = 0, int num_of_rows = 0, int num_of_cols = 0, int num_of_obstacles = 0, int warehouse_width = 0);


	struct Agent {
		int index;
		int start_locaton;
		int goal_location;
		int spawn_time;
	};
	
	void printAgents() const;

		inline bool isObstacle(int loc) const { return my_map[loc]; }
		inline bool validMove(int curr, int next) const;
		inline bool validMove(int curr, int next, int time) const;
		list<int> getNeighbors(int curr) const;

		inline int linearizeCoordinate(int row, int col) const { return ( this->num_of_cols * row + col); }
		inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
		inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
		inline pair<int, int> getCoordinate(int id) const { return make_pair(id / this->num_of_cols, id % this->num_of_cols); }
		inline int getCols() const { return num_of_cols; }

		inline int getManhattanDistance(int loc1, int loc2) const
		{
			int loc1_x = getRowCoordinate(loc1);
			int loc1_y = getColCoordinate(loc1);
			int loc2_x = getRowCoordinate(loc2);
			int loc2_y = getColCoordinate(loc2);
			return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
		}

		inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2) const
		{
			return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
		}

	int getDegree(int loc) const
	{
		assert(loc >= 0 && loc < map_size && !my_map[loc]);
		int degree = 0;
		if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols])
			degree++;
		if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
			degree++;
		if (loc % num_of_cols > 0 && !my_map[loc - 1])
			degree++;
		if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
			degree++;
		return degree;
	}
	
	void initAgentSets() {
		map<int, int> times; 
		for (auto& ag : agent_list) {
			if (times.find(ag.spawn_time) == times.end()) {
				times[ag.spawn_time] = 1;
			} else {
				times[ag.spawn_time] += 1;
			}
		}
		agent_sets.clear();
		for (auto t : times ) {
			agent_sets.push_back(t);
		}
	}
	
	// [<timestep, num of agents>...]
	vector<pair<int, int>> getAgentSets() const {
		
		return agent_sets; 
	}
	int getDefaultNumberOfAgents() const { return num_of_agents; }
	void AddRandAgents(int amt_agents);
	void AddSingleAgent(const Agent& agent);
	void removeAgent(int index);
	void timeStep(const vector<int>& moves, int batch);


private:
	  // int moves_offset[MOVE_COUNT];
	  vector<bool> my_map;
	  string map_fname;
	  string agent_fname;
	  vector<Agent> agent_list; 

		vector<pair<int, int>> agent_sets;
	  int num_of_agents;
	  vector<int> agent_location;
	  vector<int> goal_locations;

	  bool loadMap();
	  void printMap() const;
	  void saveMap() const;

	  bool loadAgents();
	  void saveAgents() const;

	  void generateConnectedRandomGrid(int rows, int cols, int obstacles); // initialize new [rows x cols] map with random obstacles
	  void generateRandomAgents(int warehouse_width);
	  bool addObstacle(int obstacle); // add this obsatcle only if the map is still connected
	  bool isConnected(int start, int goal); // run BFS to find a path between start and goal, return true if a path exists.

	  int randomWalk(int loc, int steps) const;

	  // Class  SingleAgentSolver can access private members of Node 
	  friend class SingleAgentSolver;
		friend class SpaceTimeAStar;
		friend class CBS;
		friend class ECBS;
};

std::ostream& operator<<(std::ostream& os, const Instance::Agent& agent);