#pragma once
#include "CBS.h"
#include "ECBSNode.h"

enum replan_type { REPLAN_SINGLE, REPLAN_SINGLE_GROUP, REPLAN_ALL };


class ECBS : public CBS
{
public:
// TODO: delete this
ECBS(Instance& instance, bool sipp, int screen) : CBS(instance, sipp, screen), instance(instance){}


	ECBS(Instance& instance, bool sipp, int screen, replan_type replan) : CBS(instance, sipp, screen), batch(0), replanner(replan), agent_sets(instance.getAgentSets()), instance(instance){
		num_of_agents = agent_sets[0].second;
		instance.num_of_agents = num_of_agents;
	}

	// ECBSNode* dummy_start = nullptr;
	// ECBSNode* goal_node = nullptr;

	////////////////////////////////////////////////////////////////////////////////////////////
	// Runs the algorithm until the problem is solved or time is exhausted 
	bool solve(double time_limit, int cost_lowerbound = 0);
    void clear(); // used for rapid random  restart

	// ONLINE STUFF
	bool solveReplan(double time_limit, int _cost_lowerbound);

	bool nextBatch();

	void printPaths() const;

	void printBatchStats(const string &filename) const;

	std::vector<int> moves_out;
	vector< pair<Path, int> > paths_found_initially;  // contain initial paths found
private:
	// We are gonna be lazy here... lol
	Instance& instance;

	int batch;
	// [<timestamp, num of agents>...]
	vector<pair<int, int>> agent_sets;

	void updateStartNode();

	bool solveReplanAll(double time_limit, int _cost_lowerbound);
	bool generateRootAll();
	bool generateChildAll(ECBSNode* node, ECBSNode* parent);
	
	bool solveReplanSingle(double time_limit, int _cost_lowerbound);
	bool generateRootSingle();
	bool generateChildSingle(ECBSNode* node, ECBSNode* parent);
	// FOR REPLAN SINGLE
	int agent_index = 0; 
	
	bool solveReplanSingleGroup(double time_limit, int _cost_lowerbound);
	bool generateRootSingleGroup();
	bool generateChildSingleGroup(ECBSNode* node, ECBSNode* parent);

	replan_type replanner;

	vector<int> min_f_vals; // lower bounds of the cost of the shortest path

	pairing_heap< ECBSNode*, compare<ECBSNode::compare_node_by_f> > cleanup_list; // it is called open list in ECBS
	pairing_heap< ECBSNode*, compare<ECBSNode::compare_node_by_inadmissible_f> > open_list; // this is used for EES
	pairing_heap< ECBSNode*, compare<ECBSNode::compare_node_by_d> > focal_list; // this is ued for both ECBS and EES

	void adoptBypass(ECBSNode* curr, ECBSNode* child, const vector<int>& fmin_copy);

	// node operators
	void pushNode(ECBSNode* node);
	ECBSNode* selectNode();
	bool reinsertNode(ECBSNode* node);
	void clearNodes();

	 // high level search
	bool generateChild(ECBSNode* child, ECBSNode* curr);
	bool generateRoot();
	bool findPathForSingleAgent(ECBSNode*  node, int ag);
	void classifyConflicts(ECBSNode &node);
	void computeConflictPriority(shared_ptr<Conflict>& con, ECBSNode& node);

	//update information
	void updatePaths(ECBSNode* curr);

	void updatePathsFoundInitially(ECBSNode* curr);
	
};