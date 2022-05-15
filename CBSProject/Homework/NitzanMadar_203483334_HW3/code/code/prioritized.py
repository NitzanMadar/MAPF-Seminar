import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from math import inf as INF

class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        # constraints = [{'agent': 0, 'loc': [(1, 5)], 'timestep': 4}]  #todo 1.2
        # constraints = [{'agent': 1, 'loc': [(1, 2),(1,3)], 'timestep': 1}]  #todo 1.3

        # this constrains are needed to success (A* Optimal, than we need all 3 to make agent 1 do down and the rest will work)
        # constraints = [{'agent' : 1, 'loc' : [(1,2)], 'timestep' : 2},
        #                {'agent' : 1, 'loc' : [(1,3)], 'timestep' : 2},
        #                {'agent' : 1, 'loc' : [(1,4)], 'timestep' : 2}]

        #constraints = [{'agent': 0, 'loc': [(1, 5)], 'timestep': 10}]  #todo 1.4

        for i in range(self.num_of_agents):  # Find path for each agent

            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # TODO 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            ##############################
            # max_path_lengths = max(len(path), max_path_lengths)

            t = 0 # time variable for current agent
            for future_agent in range(i+1, self.num_of_agents): # run on other agent
                for cell in path:
                    #vertex contrains
                    constraints.append({'agent': future_agent, 'loc': [cell], 'timestep': t})

                    #edge constrains
                    if t<len(path)-1:
                        constraints.append({'agent': future_agent, 'loc': [path[t+1],cell], 'timestep': t+1})
                    t = t + 1 #one step

                constraints.append({'agent': future_agent, 'loc':(len(path)-1,[path[-1]]),'timestep': INF})
                t = 0 # for the next agent



        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        print(constraints)
        # print(self.my_map)
        return result