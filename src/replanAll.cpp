#include "ECBS.h"
#include "SpaceTimeAStar.h"

bool ECBS::solveReplanAll(double time_limit, int _cost_lowerbound) {
    this->cost_lowerbound = _cost_lowerbound;
    this->inadmissible_cost_lowerbound = 0;
    this->time_limit = time_limit;

    this->dummy_start = this->goal_node;
    this->goal_node = nullptr;
    this->clearNodes();

    start = clock();
    generateRoot(); // Plan for all agents from current positions

    while (!cleanup_list.empty() && !solution_found) {
        auto curr = selectNode();
        if (terminate(curr)) return solution_found;

        if ((curr == dummy_start || curr->chosen_from == "cleanup") &&
            !curr->h_computed) {
            runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
            bool succ = heuristic_helper.computeInformedHeuristics(*curr, min_f_vals, time_limit - runtime);
            runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
            if (!succ) {
                if (screen > 1) cout << "\tPrune " << *curr << endl;
                curr->clear();
                continue;
            }
            if (reinsertNode(curr)) continue;
        }

        classifyConflicts(*curr);
        num_HL_expanded++;
        curr->time_expanded = num_HL_expanded;

        ECBSNode* child[2] = { new ECBSNode(), new ECBSNode() };
        curr->conflict = chooseConflict(*curr);
        addConstraints(curr, child[0], child[1]);

        if (screen > 1)
            cout << "\tExpand " << *curr << endl << "\ton " << *(curr->conflict) << endl;

        bool solved[2] = { false, false };
        vector<vector<PathEntry>*> path_copy(paths);
        vector<int> fmin_copy(min_f_vals);

        for (int i = 0; i < 2; i++) {
            if (i > 0) {
                paths = path_copy;
                min_f_vals = fmin_copy;
            }
            solved[i] = generateChild(child[i], curr);
            if (!solved[i]) {
                delete child[i];
                continue;
            }
            pushNode(child[i]);
            curr->children.push_back(child[i]);
            if (screen > 1) cout << "\t\tGenerate " << *child[i] << endl;
        }

        switch (curr->conflict->type) {
            case conflict_type::RECTANGLE: num_rectangle_conflicts++; break;
            case conflict_type::CORRIDOR: num_corridor_conflicts++; break;
            case conflict_type::TARGET: num_target_conflicts++; break;
            case conflict_type::STANDARD: num_standard_conflicts++; break;
            case conflict_type::MUTEX: num_mutex_conflicts++; break;
            default: break;
        }

        if (curr->chosen_from == "cleanup") num_cleanup++;
        else if (curr->chosen_from == "open") num_open++;
        else if (curr->chosen_from == "focal") num_focal++;

        if (curr->conflict->priority == conflict_priority::CARDINAL)
            num_cardinal_conflicts++;

        if (!curr->children.empty())
            heuristic_helper.updateOnlineHeuristicErrors(*curr);

        curr->clear();
    }

    return solution_found;
}
