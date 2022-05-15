import heapq
from math import inf as INF

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]  # NITZAN: ADDED WAIT "DIRECTION"
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))

            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # TODO 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.
    #               constrain example: {'agent': 2, 'loc': [(3,4)], 'timestep': 5}

    filtered_agents_constraints = list(filter(lambda con: con['agent'] is agent, constraints))

    timestep_loc_list = list(map(lambda con: (con['timestep'], (con['loc'], con['positive'])), filtered_agents_constraints))
    const_table = {}
    for timestep, loc_pos in timestep_loc_list:
        if timestep in const_table:
            const_table[timestep].append(loc_pos)
        else:
            const_table[timestep] = [loc_pos]

    return const_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_negative_constraint(curr_loc, next_loc, next_time, constraint_table):
    return (([next_loc], False) in constraint_table[next_time]) or \
           (([curr_loc, next_loc], False) in constraint_table[next_time])


def is_positive_constraint(curr_loc, next_loc, next_time, constraint_table):
    positive_in_timestep = list(filter(lambda con: con[1] is True, constraint_table[next_time]))
    if len(positive_in_timestep) > 0:
        return ([next_loc], True) not in positive_in_timestep and ([curr_loc, next_loc], True) not in positive_in_timestep


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # todo Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time not in constraint_table: # no constraint at the timestep
        if INF in constraint_table: # someone on goal - inf constraint
            # print(constraint_table[INF])
            # print((((next_time, [next_loc]), False)))
            for con in constraint_table[INF]:
                if con[0][0] <= next_time and con[0][1] == [next_loc] and con [1]==False:
                    return True
            # return len(list(filter(lambda inf_con: # next time smaller than t in inf and is the same location
            #                        inf_con[0][0] <= next_time and [next_loc] in inf_con[0][1], constraint_table[INF]))) > 0
        else: # next time no in constraints table and there is no INF constraint
            return False
    # if INF in constraint_table: # if there is INF constraint...
    #     # print(constraint_table[INF][0])
    #     # print((((next_time, [next_loc]),False)))
    #     print(constraint_table[next_time])
    #     return (([next_loc], False) in constraint_table[next_time]) or \
    #            (([curr_loc, next_loc],False) in constraint_table[next_time]) or \
    #            (((next_time, [next_loc]),False) in constraint_table[INF])

    else:
        return is_negative_constraint(curr_loc, next_loc, next_time, constraint_table) or \
               is_positive_constraint(curr_loc, next_loc, next_time, constraint_table)


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def no_goal_constraints(loc, timestep, constraints_table, constraints, agent):
    #extract all positive constraints which violated by loc at t
    all_positive_constraints = list(filter(lambda con: con['positive'] is True and con['timestep'] > timestep and
                                                       con['loc'] is [loc] and con['agent'] is not agent, constraints))
    if len(all_positive_constraints) > 0:
        return True #positive constraint on that goal location
    for timestep_key, con_list in constraints_table.items():
        if timestep_key < timestep:
            continue
        if ([loc], False) in con_list:
            return False
    return True


def out_of_bounds(map, row, col): # tests are without wrap map
    return row >= len(map) or row < 0 or col >= len(map[0]) or col < 0


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
                      example: {'agent': 2, 'loc': [(3,4)], 'timestep': 5}
                      in disjoint splitting CBS, added 'positive' field
    """
    ##############################
    # TODO 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]

    constraints_table = build_constraint_table(constraints, agent)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)

        #############################
        if curr['timestep'] > 4 * len(my_map) * len(my_map[1]):  # simple running time limit ...
            continue

        # TODO 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and no_goal_constraints(goal_loc, curr['timestep'], constraints_table, constraints, agent):
            return get_path(curr)

        for dir in range(5):  #  changed to 5 because I added dir=5 stay, see "move" function
            child_loc = move(curr['loc'], dir)

            if out_of_bounds(my_map, child_loc[0], child_loc[1]) or my_map[child_loc[0]][child_loc[1]]:
                continue

            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}

            # print(constraints_table)

            if is_constrained(curr['loc'], child['loc'], child['timestep'], constraints_table): #negative constraints
                continue


            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
