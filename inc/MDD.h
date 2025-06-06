#pragma once
#include "SingleAgentSolver.h"
#include "ECBSNode.h"


class MDDNode
{
public:
	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		if(parent == nullptr)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
	}
	MDDNode(int location, int t): location(location), level(t) {}
	int location;
	int level;
  int cost; // minimum cost of path traversing this MDD node

	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level);
	}


	list<MDDNode*> children;
	list<MDDNode*> parents;
};

class MDD
{
private:
    const SingleAgentSolver* solver;

public:
	vector<list<MDDNode*>> levels;

	bool buildMDD(const ConstraintTable& ct,
		int num_of_levels, const SingleAgentSolver* solver); // build mdd of given levels
	bool buildMDD(ConstraintTable& ct, const SingleAgentSolver* solver); // build minimal MDD
	// bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels,
	// 	int start_location, const int* moves_offset, const std::vector<int>& my_heuristic, int map_size, int num_col);

	MDDNode* find(int location, int level) const;
	void deleteNode(MDDNode* node);
	void clear();
	// bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons) const;

    void increaseBy(const ConstraintTable&ct, int dLevel, SingleAgentSolver* solver);
    MDDNode* goalAt(int level);
    void printNodes() const;

	MDD()= default;;
	MDD(const MDD & cpy);
	~MDD();
};

std::ostream& operator<<(std::ostream& os, const MDD& mdd);

class SyncMDDNode
{
public:
	SyncMDDNode(int currloc, SyncMDDNode* parent)
	{
		location = currloc;
		if (parent != nullptr)
		{
			//level = parent->level + 1;
			parents.push_back(parent);
		}
		//parent = NULL;
	}
	int location;
	//int level;

	bool operator == (const SyncMDDNode & node) const
	{
		return (this->location == node.location);
	}


	list<SyncMDDNode*> children;
	list<SyncMDDNode*> parents;
	list<const MDDNode*> coexistingNodesFromOtherMdds;

};


class SyncMDD
{
public:
	vector<list<SyncMDDNode*>> levels;

	SyncMDDNode* find(int location, int level) const;
	void deleteNode(SyncMDDNode* node, int level);
	void clear();

	explicit SyncMDD(const MDD & cpy);
	~SyncMDD();
};

class MDDTable
{
public:
	double accumulated_runtime = 0;  // runtime of building MDDs
	uint64_t num_released_mdds = 0; // number of released MDDs ( to save memory)

	MDDTable(const vector<ConstraintTable>& initial_constraints,
						const vector<SingleAgentSolver*>& search_engines):
		initial_constraints(initial_constraints), search_engines(search_engines) {}
	
	void init(int number_of_agents)
	{
		lookupTable.resize(number_of_agents);
	}
	~MDDTable() { clear(); }

	MDD* findMDD(HLNode& node, int agent) const;
	MDD * getMDD(HLNode& node, int agent, int mdd_levels = -1);
	// void findSingletons(HLNode& node, int agent, Path& path);
	void clear();
private:
	int max_num_of_mdds = 10000; // per agent

	vector<unordered_map<ConstraintsHasher, MDD*, 
		ConstraintsHasher::Hasher, ConstraintsHasher::EqNode> >lookupTable;

	const vector<ConstraintTable>& initial_constraints;
	const vector<SingleAgentSolver*>& search_engines;
	void releaseMDDMemory(int id);

	friend class ECBS;
};

unordered_map<int, MDDNode*> collectMDDlevel(MDD* mdd, int i);
