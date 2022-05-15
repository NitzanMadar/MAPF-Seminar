import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # TODO Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    Tmin = min(len(path1),len(path2))
    Tmax = max(len(path1),len(path2))
    for t in range(Tmax):
        # when t is out of path range the get_location bring the final position
        L1 = get_location(path1, t)
        L2 = get_location(path2, t)
        # vertex:
        if L1 == L2:
            return {'loc': [L1], 'timestep':t}

        # edge:
        if t < Tmin - 1:
            next1 = get_location(path1, t + 1)
            next2 = get_location(path2, t + 1)
            if L1 == next2 and L2 == next1: #switch positions, edge collision
                # PAY ATTENTION! the output is according to path1!
                return {'loc': [L1, next1], 'timestep': t + 1}

    return None     # no collisions


def detect_collisions(paths):
    ##############################
    # TODO Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    # print("paths are:",paths)

    collisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)): #next agents
            first_coll = detect_collision(paths[i],paths[j])
            if first_coll is not None:
                collisions.append(
                    {'a1': i, 'a2': j, 'loc': first_coll['loc'], 'timestep': first_coll['timestep']})

    return collisions




def standard_splitting(collision):
    ##############################
    # todo Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    return [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False},
            {'agent': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep'], 'positive': False}]


def disjoint_splitting(collision):
    ##############################
    # todo Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    rand01 = random.randint(0, 1)

    if rand01 is 0:
        return [
            {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True},
            {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}]
    else:
        return [
            {'agent': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep'], 'positive': True},
            {'agent': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep'], 'positive': False}]


def paths_violate_constraint(paths, constraint):
    violated_agents = []
    timestep = constraint['timestep']
    main_path = paths[constraint['agent']]
    for agent_id in range(len(paths)): # we look for the other agents who violate
        if agent_id == constraint['agent']:
            continue

        path = paths[agent_id] # current path of agent_id
        collisions = detect_collision(main_path, path)
        if collisions is not None: # there is a collision
            violated_agents.append(agent_id)

    return violated_agents


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths

        root = {'cost': 0, # Line 4 initializte
                'constraints': [], # Line 1
                'paths': [], # Line 2 initializte
                'collisions': []} #Line 3 initializte

        # Line 2
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])#, goal_constrains, max_path_lengths)

            if path is None: # if any agent can go to his goal without constarints than there is no solution...
                raise BaseException('No solutions')

            root['paths'].append(path) # add paths of any agent

        root['cost'] = get_sum_of_cost(root['paths']) #Line 4
        root['collisions'] = detect_collisions(root['paths']) #Line 3
        self.push_node(root) # Line 5

        # # Task 3.1: Testing
        # print(root['collisions'])
        #
        # # Task 3.2: Testing
        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        ##############################
        # todo Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node())
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            curr_node = self.pop_node() # Line 7

            if (curr_node['collisions'] == []): # Line 8
                self.print_results(curr_node, disjoint)
                return curr_node['paths'] #Line 9

            collision = curr_node['collisions'][0] #Line 10

            # Line 11
            if disjoint is True:    #one of the features, use the command line
                constraints = disjoint_splitting(collision)
            else:                   # standard splitting
                constraints = standard_splitting(collision)

            for constraint in constraints:
                # Line 13
                new_node = {'cost': 0,
                     'constraints': curr_node['constraints'].copy() + [constraint], #line 14
                     'paths': curr_node['paths'].copy(), #line 15
                     'collisions': []}

                a_i = constraint["agent"] #line 16

                path = a_star(self.my_map, self.starts[a_i], self.goals[a_i], self.heuristics[a_i],
                              a_i, new_node['constraints'])  # line 17


                if path is not None and len(path) > 0:

                    #######
                    new_node["paths"][a_i] = path # Line 19

                    no_solution = False

                    # adding negative constraints that implements the possitive constraint
                    if disjoint is True and constraint['positive'] is True:
                        new_constraints = [{'agent': a, 'loc': constraint['loc'][::-1],
                                            'timestep': collision['timestep'], 'positive': False}
                                           for a in range(len(new_node['paths'])) if a is not a_i]

                        new_node['constraints'] = new_node['constraints'] + new_constraints

                        # find the agents who violate the new constraints list
                        violated_agents = paths_violate_constraint(new_node['paths'], constraint)

                        # for every agent who violate - find a new path using A* function
                        for vio in violated_agents:
                            new_path = a_star(self.my_map, self.starts[vio], self.goals[vio],
                                              self.heuristics[vio], vio, new_node['constraints'])

                            # maybe some agent have no solution... raise a flag and break the loop
                            if new_path is None or len(new_path) is 0:
                                no_solution = True
                                break

                            else: # new paths for all violated agent found
                                new_node['paths'][vio] = new_path.copy()

                    if no_solution:
                        # one of the violated agents doesn't have a solution with the new positive constraint
                        # do not add child
                        continue

                    new_node["collisions"] = detect_collisions(new_node["paths"]) # line 20
                    new_node["cost"] = get_sum_of_cost(new_node["paths"])#line 21
                    self.push_node(new_node) #line 22

        raise BaseException('No solutions') #return "No Solutions", line 23


    def print_results(self, node, disjoint):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        if disjoint==True:
            print("Splitting Method: Disjoint")
        else:
            print("Splitting Method: Standard")
