#include "ECBS.h"
#include "SpaceTimeAStar.h"


void ECBS::clearNodes() {
	this->open_list.clear();
	this->focal_list.clear();
	this->cleanup_list.clear();
  
  // for (auto& node : allNodes_table) {
  //   if (node != this->goal_node) {
  //     delete node;
  //   }
  // }

  allNodes_table.clear();
}

bool ECBS::solveReplanSingle(double time_limit, int _cost_lowerbound) {
	this->cost_lowerbound = _cost_lowerbound;
	this->inadmissible_cost_lowerbound = 0;
	this->time_limit = time_limit;

	// set timer
	start = clock();
	auto curr = static_cast<ECBSNode*>(dummy_start);
	for (agent_index = num_of_agents - agent_sets[batch].second;
	     agent_index < num_of_agents; agent_index++){

				 generateRootSingle();

				 while (!cleanup_list.empty() && !solution_found)
				 {
					curr = selectNode();
					 if (terminate(curr)) {
							break;
					 }
			 
					 if ((curr == dummy_start || curr->chosen_from == "cleanup") &&
								!curr->h_computed) // heuristics has not been computed yet
					 {
									 runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
									 bool succ = heuristic_helper.computeInformedHeuristics(*curr, min_f_vals, time_limit - runtime);
									 runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
									 if (!succ) // no solution, so prune this node
									 {
											 if (screen > 1)
													 cout << "	Prune " << *curr << endl;
											 curr->clear();
											 continue;
									 }
									 if (reinsertNode(curr)) {
										 continue;
									 }
					 }
			 
						classifyConflicts(*curr);
			 
					 //Expand the node
					 num_HL_expanded++;
					 curr->time_expanded = num_HL_expanded;
					 if (bypass && curr->chosen_from != "cleanup")
					 {
						 bool foundBypass = true;
						 while (foundBypass)
						 {
							 	if (terminate(curr)) {
									// return solution_found;
									break;
							 	}
							 
								foundBypass = false;
							 	ECBSNode* child[2] = { new ECBSNode() , new ECBSNode() };
							 	curr->conflict = chooseConflict(*curr);
			 
								addConstraints(curr, child[0], child[1]);
								// Don't expand nodes where an agent can't be replanned.
								if (curr->conflict->a1 < agent_index) {
									assert(curr->conflict->a2 >= agent_index);
									delete child[0];
									child[0] = nullptr;
								}
								if  (curr->conflict->a2 < agent_index) {
									assert(child[0] != nullptr);
									delete child[1];
									child[1] = nullptr;
								}
			 
								if (screen > 1)
									cout << "	Expand " << *curr << endl << 	"	on " << *(curr->conflict) << endl;
			 
								bool solved[2] = { false, false };
								vector<vector<PathEntry>*> path_copy(paths);
								vector<int> fmin_copy(min_f_vals);
								for (int i = 0; i < 2; i++)
								{
									if (child[i] == nullptr) {
										continue;
									}
				
									if (i > 0)
									{
										paths = path_copy;
										min_f_vals = fmin_copy;
									}
									solved[i] = generateChildSingle(child[i], curr);
									if (!solved[i])
									{
										delete (child[i]);
										continue;
									}
									else if (i == 1 && !solved[0])
										continue;
									else if (bypass &&
										child[i]->sum_of_costs <= suboptimality * cost_lowerbound &&
										child[i]->distance_to_go < curr->distance_to_go) // Bypass1
									{
										foundBypass = true;
										for (const auto& path : child[i]->paths)
										{
											 /*if (path.second.first.size() != path_copy[path.first]->size()) // CBS bypassing
																	 {
																			 foundBypass = false;
																			 break;
																	 }*/
											if ((double)path.second.first.size() - 1 > suboptimality * fmin_copy[path.first]) // Our bypassing
											{
												foundBypass = false;
												break;
											}
										}
										if (foundBypass)
										{
											adoptBypass(curr, child[i], fmin_copy);
											if (screen > 1)
												cout << "	Update " << *curr << endl;
											break;
										}
									}
								}
								if (foundBypass)
								{
									for (auto & i : child)
									{
										if (i != nullptr)
											delete i;
									}
													 classifyConflicts(*curr); // classify the new-detected conflicts
								}
								else
								{
									for (int i = 0; i < 2; i++)
									{
										if (solved[i])
										{
											pushNode(child[i]);
											curr->children.push_back(child[i]);
											if (screen > 1)
											{
												cout << "		Generate " << *child[i] << endl;
											}
										}
									}
								}
							}
						}
						else // no bypass
						{
							ECBSNode* child[2] = { new ECBSNode() , new ECBSNode() };
							curr->conflict = chooseConflict(*curr);
							addConstraints(curr, child[0], child[1]);
							if (curr->conflict->a1 < agent_index) {
								assert(curr->conflict->a2 >= agent_index);
								delete child[0];
								child[0] = nullptr;
							}
							if  (curr->conflict->a2 < agent_index) {
								assert(child[0] != nullptr);
								delete child[1];
								child[1] = nullptr;
							}
				
							if (screen > 1)
								cout << "	Expand " << *curr << endl << "	on " << *(curr->conflict) << endl;
				
							bool solved[2] = { false, false };
							vector<vector<PathEntry>*> path_copy(paths);
							vector<int> fmin_copy(min_f_vals);
							for (int i = 0; i < 2; i++)
							{
								if (child[i] == nullptr) {
									continue;
								}
								if (i > 0)
								{
									paths = path_copy;
									min_f_vals = fmin_copy;
								}
								solved[i] = generateChildSingle(child[i], curr);
								if (!solved[i])
								{
									delete (child[i]);
									continue;
								}
								pushNode(child[i]);
								curr->children.push_back(child[i]);
								if (screen > 1)
									cout << "		Generate " << *child[i] << endl;
							}
						}
						if (curr->conflict == nullptr && terminate(curr)) {
							break;
						}
						switch (curr->conflict->type)
						{
						case conflict_type::RECTANGLE:
							num_rectangle_conflicts++;
							break;
						case conflict_type::CORRIDOR:
							num_corridor_conflicts++;
							break;
						case  conflict_type::TARGET:
							num_target_conflicts++;
							break;
						case conflict_type::STANDARD:
							num_standard_conflicts++;
							break;
						case conflict_type::MUTEX:
							num_mutex_conflicts++;
							break;
						default:
							break;
						}
						if (curr->chosen_from == "cleanup")
							num_cleanup++;
						else if (curr->chosen_from == "open")
							num_open++;
						else if (curr->chosen_from == "focal")
							num_focal++;
						if (curr->conflict->priority == conflict_priority::CARDINAL)
							num_cardinal_conflicts++;
								if (!curr->children.empty())
										heuristic_helper.updateOnlineHeuristicErrors(*curr); // update online heuristic errors
						curr->clear();
					}  // end of while loop

		if (!solution_found) {
			cerr << "No solution found for agent " << agent_index << endl;
			return false;
		}
		if (agent_index != num_of_agents - 1) {
			
			updateStartNode();
			solution_found = false;
		}
		
	}
	return solution_found;
}


// ran by solveReplanSingle for each agent_index
bool ECBS::generateRootSingle() {
  auto root = static_cast<ECBSNode *>(this->dummy_start);

	search_engines.resize(agent_index + 1);
	for (int i = 0; i < agent_index; i++) {
		search_engines[i]->plannable = false;
	}
	search_engines[agent_index] = new SpaceTimeAStar(this->instance, agent_index);
	search_engines[agent_index]->plannable = true;
	
	paths_found_initially[agent_index] = search_engines[agent_index]->
		findSuboptimalPath(*root, initial_constraints[agent_index], paths, agent_index, 0, suboptimality);
	if (paths_found_initially[agent_index].first.empty()) {
		cerr << "the start-goal locations of agent " << agent_index << "are not connected" << endl; 
		exit(-1);
	}
	
	paths[agent_index] = &paths_found_initially[agent_index].first;
	min_f_vals[agent_index] = paths_found_initially[agent_index].second;
	num_LL_expanded += search_engines[agent_index]->num_expanded;
	num_LL_generated += search_engines[agent_index]->num_generated;  

	root->makespan = 0;
	root->g_val = 0;
	root->sum_of_costs = 0;

	for (int ag = 0; ag < agent_index + 1; ag++) {
		root->makespan = max(root->makespan, paths[ag]->size() - 1);
		root->g_val += min_f_vals[ag];
		root->sum_of_costs += (int)paths[ag]->size() - 1;
	}

  root->h_val = 0;
	root->depth = 0;
	findConflicts(*root);
  heuristic_helper.computeQuickHeuristics(*root);

	pushNode(root);

	return true;
}

// ran by solveReplanSingle for each agent_index
bool ECBS::generateChildSingle(ECBSNode* node, ECBSNode* parent) {
  clock_t t1 = clock();
  node->parent = parent;
  node->HLNode::parent = parent;
  node->g_val = parent->g_val;
	node->sum_of_costs = parent->sum_of_costs;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;
	auto agents = getInvalidAgents(node->constraints);
	assert(!agents.empty());
	for (auto agent : agents)
	{
    // if (agent < num_of_agents - agent_sets[batch].second)  {
		if (agent != agent_index) {
      continue;
    }
		if (!findPathForSingleAgent(node, agent))
		{
            if (screen > 1)
                cout << "	No paths for agent " << agent << ". Node pruned." << endl;
			runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	findConflicts(*node);
	heuristic_helper.computeQuickHeuristics(*node);
	runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
	return true;
}


bool ECBS::solveReplanSingleGroup(double time_limit, int _cost_lowerbound) {
  this->cost_lowerbound = _cost_lowerbound;
	this->inadmissible_cost_lowerbound = 0;
	this->time_limit = time_limit;

	// set timer
	start = clock();

  generateRootSingleGroup();
	
	if (screen > 1)
		printPaths();

	while (!cleanup_list.empty() && !solution_found)
	{
		auto curr = selectNode();
		if (terminate(curr)) {
			return solution_found;
    }

		if ((curr == dummy_start || curr->chosen_from == "cleanup") &&
		     !curr->h_computed) // heuristics has not been computed yet
		{
            runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
            bool succ = heuristic_helper.computeInformedHeuristics(*curr, min_f_vals, time_limit - runtime);
            runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
            if (!succ) // no solution, so prune this node
            {
                if (screen > 1)
                    cout << "	Prune " << *curr << endl;
                curr->clear();
                continue;
            }

            if (reinsertNode(curr)) {
              continue;
            }
		}

        classifyConflicts(*curr);

		//Expand the node
		num_HL_expanded++;
		curr->time_expanded = num_HL_expanded;
		if (bypass && curr->chosen_from != "cleanup")
		{
			bool foundBypass = true;
			while (foundBypass)
			{
				if (terminate(curr)) {
					return solution_found;
        }
				
        foundBypass = false;
				ECBSNode* child[2] = { new ECBSNode() , new ECBSNode() };
				curr->conflict = chooseConflict(*curr);

				addConstraints(curr, child[0], child[1]);
        // Don't expand nodes where an agent can't be replanned.
        if (curr->conflict->a1 < num_of_agents - agent_sets[batch].second) {
          assert(curr->conflict->a2 >= num_of_agents - agent_sets[batch].second);
          delete child[0];
          child[0] = nullptr;
        }
        if  (curr->conflict->a2 < num_of_agents - agent_sets[batch].second) {
          assert(child[0] != nullptr);
          delete child[1];
          child[1] = nullptr;
        }

				if (screen > 1)
					cout << "	Expand " << *curr << endl << 	"	on " << *(curr->conflict) << endl;

				bool solved[2] = { false, false };
				vector<vector<PathEntry>*> path_copy(paths);
				vector<int> fmin_copy(min_f_vals);
				for (int i = 0; i < 2; i++)
				{
          if (child[i] == nullptr) {
            continue;
          }

					if (i > 0)
					{
						paths = path_copy;
						min_f_vals = fmin_copy;
					}
					solved[i] = generateChildSingleGroup(child[i], curr);
					if (!solved[i])
					{
						delete (child[i]);
						continue;
					}
					else if (i == 1 && !solved[0])
						continue;
					else if (bypass &&
						child[i]->sum_of_costs <= suboptimality * cost_lowerbound &&
						child[i]->distance_to_go < curr->distance_to_go) // Bypass1
					{
						foundBypass = true;
						for (const auto& path : child[i]->paths)
						{
						    /*if (path.second.first.size() != path_copy[path.first]->size()) // CBS bypassing
                            {
                                foundBypass = false;
                                break;
                            }*/
							if ((double)path.second.first.size() - 1 > suboptimality * fmin_copy[path.first]) // Our bypassing
							{
								foundBypass = false;
								break;
							}
						}
						if (foundBypass)
						{
							adoptBypass(curr, child[i], fmin_copy);
							if (screen > 1)
								cout << "	Update " << *curr << endl;
							break;
						}
					}
				}
				if (foundBypass)
				{
					for (auto & i : child)
					{
            if (i != nullptr)
						  delete i;
					}
                    classifyConflicts(*curr); // classify the new-detected conflicts
				}
				else
				{
					for (int i = 0; i < 2; i++)
					{
						if (solved[i])
						{
							pushNode(child[i]);
							curr->children.push_back(child[i]);
							if (screen > 1)
							{
								cout << "		Generate " << *child[i] << endl;
							}
						}
					}
				}
			}
		}
		else // no bypass
		{
			ECBSNode* child[2] = { new ECBSNode() , new ECBSNode() };
			curr->conflict = chooseConflict(*curr);
			addConstraints(curr, child[0], child[1]);
      if (curr->conflict->a1 < num_of_agents - agent_sets[batch].second) {
        assert(curr->conflict->a2 >= num_of_agents - agent_sets[batch].second);
        delete child[0];
        child[0] = nullptr;
      }
      if  (curr->conflict->a2 < num_of_agents - agent_sets[batch].second) {
        assert(child[0] != nullptr);
        delete child[1];
        child[1] = nullptr;
      }

			if (screen > 1)
				cout << "	Expand " << *curr << endl << "	on " << *(curr->conflict) << endl;

			bool solved[2] = { false, false };
			vector<vector<PathEntry>*> path_copy(paths);
			vector<int> fmin_copy(min_f_vals);
			for (int i = 0; i < 2; i++)
			{
        if (child[i] == nullptr) {
          continue;
        }
				if (i > 0)
				{
					paths = path_copy;
					min_f_vals = fmin_copy;
				}
				solved[i] = generateChildSingleGroup(child[i], curr);
				if (!solved[i])
				{
					delete (child[i]);
					continue;
				}
				pushNode(child[i]);
				curr->children.push_back(child[i]);
				if (screen > 1)
					cout << "		Generate " << *child[i] << endl;
			}
		}
		switch (curr->conflict->type)
		{
		case conflict_type::RECTANGLE:
			num_rectangle_conflicts++;
			break;
		case conflict_type::CORRIDOR:
			num_corridor_conflicts++;
			break;
		case  conflict_type::TARGET:
			num_target_conflicts++;
			break;
		case conflict_type::STANDARD:
			num_standard_conflicts++;
			break;
		case conflict_type::MUTEX:
			num_mutex_conflicts++;
			break;
		default:
			break;
		}
		if (curr->chosen_from == "cleanup")
			num_cleanup++;
		else if (curr->chosen_from == "open")
			num_open++;
		else if (curr->chosen_from == "focal")
			num_focal++;
		if (curr->conflict->priority == conflict_priority::CARDINAL)
			num_cardinal_conflicts++;
        if (!curr->children.empty())
            heuristic_helper.updateOnlineHeuristicErrors(*curr); // update online heuristic errors
		curr->clear();
	}  // end of while loop
	
	return solution_found;
}


bool ECBS::generateRootSingleGroup() {
  auto root = static_cast<ECBSNode *>(this->dummy_start);
	
  search_engines.resize(num_of_agents);
	for (int i = 0; i < num_of_agents - agent_sets[batch].second; i++) {
		search_engines[i]->plannable = false;
	}

  for (int i = num_of_agents - agent_sets[batch].second; i < num_of_agents; i++) {
    search_engines[i] = new SpaceTimeAStar(this->instance, i);
		search_engines[i]->plannable = true;
  }

  // TODO: This is replacing the functionality of CBS::shuffleAgents. Clearly it is not shuffling. Do we care?
  vector<int> agents(agent_sets[batch].second);
  for (int i = 0; i < agent_sets[batch].second; i++) {
    agents[i] = num_of_agents - i - 1;
  }
	
  for (auto ag : agents) {
		// cout << ag << ": " << endl;
    paths_found_initially[ag] = search_engines[ag]->findSuboptimalPath(*root, initial_constraints[ag], paths, ag, 0, suboptimality);
    if (paths_found_initially[ag].first.empty()) {
      cerr << "the start-goal locations of agent " << ag << "are not connected" << endl; 
      exit(-1);
    }
    paths[ag] = &paths_found_initially[ag].first;
    min_f_vals[ag] = paths_found_initially[ag].second;


		num_LL_expanded += search_engines[ag]->num_expanded;
		num_LL_generated += search_engines[ag]->num_generated;

  }

	root->makespan = 0;
	root->g_val = 0;
	root->sum_of_costs = 0;

	for (int ag = 0; ag < num_of_agents; ag++) {
		root->makespan = max(root->makespan, paths[ag]->size() - 1);
		root->g_val += min_f_vals[ag];
		root->sum_of_costs += (int)paths[ag]->size() - 1;
	}

  root->h_val = 0;
	root->depth = 0;
	findConflicts(*root);
    heuristic_helper.computeQuickHeuristics(*root);
	pushNode(static_cast<ECBSNode *>(root));
	dummy_start = root;

	// if (screen >= 2) // print start and goals
	// 	printPaths();

	return true;
}

bool ECBS::generateChildSingleGroup(ECBSNode* node, ECBSNode* parent) {
  clock_t t1 = clock();
  node->parent = parent;
  node->HLNode::parent = parent;
  node->g_val = parent->g_val;
	node->sum_of_costs = parent->sum_of_costs;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;
	auto agents = getInvalidAgents(node->constraints);
	assert(!agents.empty());
	for (auto agent : agents)
	{
    if (agent < num_of_agents - agent_sets[batch].second)  {
      continue;
    }
		if (!findPathForSingleAgent(node, agent))
		{
            if (screen > 1)
                cout << "	No paths for agent " << agent << ". Node pruned." << endl;
			runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	findConflicts(*node);
	heuristic_helper.computeQuickHeuristics(*node);
	runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
	return true;
}
