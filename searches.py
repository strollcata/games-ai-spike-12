'''  PriorityQueue and Path classes for DFS, BSF, Dijkstra and A* searches

Created for HIT3046 AI for Games by Clinton Woodward cwoodward@swin.edu.au

See readme.txt for details.

'''
from heapq import heappush, heappop

class PriorityQueue(object):
    ''' Cost sorted (min-to-max) queue. Equal cost items revert to FIFO order.'''

    def __init__(self):
        self.q = []
        self.i = 0 # default order counter

    def push(self, item, cost):
        '''Add an item and its cost to the queue. '''
        heappush(self.q, (cost, self.i, item))
        self.i += 1

    def pop(self):
        '''Remove the item of lowest cost, or FIFO order if cost equal.
        Returns the item (whatever it is) and the cost as a tuple. '''
        cost, i, item = heappop(self.q)
        return item, cost

    def __len__(self):
        return len(self.q)

    def __str__(self):
        '''Print a sorted view of the queue contents. '''
        return 'pq: ' + str(sorted(self.q))

    def __contains__(self, item):
        return any(item == values[2] for values in self.q)

    def __iter__(self):
        '''Support iteration. This enables support of the "in" operator. '''
        return iter(values[2] for values in self.q)

    def peek(self, item):
        '''Return a tuple of (item, cost) if it exists, without removing. '''
        for values in self.q:
            if values[2] == item:
                return (item, values[0])

    def remove(self, item):
        '''Remove the first item that matches.'''
        for i, values in enumerate(self.q):
            if values[2] == item:
                del self.q[i]
                return

def add_to_path(graph, route, curr_target_idx, path):
    new_segment = []
    curr_idx = curr_target_idx
    while curr_idx not in path:
        new_segment.append(curr_idx)
        curr_idx = route[curr_idx]
    path_waypt = curr_idx
    new_segment.reverse()
    doubled_segment = []
    for waypt in new_segment:
        if waypt != curr_target_idx:
            doubled_segment.append(waypt)
    doubled_segment.reverse()
    new_segment += doubled_segment
    new_segment.append(path_waypt)
    new_path = path
    path_index = path.index(path_waypt)
    for waypt in new_segment:
        new_path.insert(path_index + 1, waypt)
        path_index += 1
    return new_path

class Path(object):
    ''' Convenient container and converter for route-path information'''
    def __init__(self, graph, route, target_idx, item_idx, point_idx, open_nodes, closed, steps):
        # keep any data if we are asked
        self.route = route
        self.open_nodes = open_nodes
        self.closed = closed
        self.target_idx = target_idx
        self.item_idx = item_idx
        self.point_idx = point_idx
        self.steps = steps
        # Convert dictionary back in to a list of nodes for a path
        if target_idx in route:
            path = []
            curr_idx = target_idx
            while curr_idx != route[curr_idx]:
                path.append(curr_idx)
                curr_idx = route[curr_idx]
            self.result = 'Success! '
            self.result += 'Still going...' if target_idx in open_nodes else 'Done!'
            path.append(curr_idx)
            path.reverse()
            if item_idx in closed:
                path = add_to_path(graph, route, item_idx, path)
            if point_idx in closed:
                path = add_to_path(graph, route, point_idx, path)
            remove_these = []
            for path_idx in range(len(path)):
                if path_idx == len(path) - 1:
                    break
                if path[path_idx + 1] == path[path_idx]:
                    remove_these.append(path_idx + 1)
            for to_remove in remove_these:
                path.remove(path[to_remove])
                for tr2 in remove_these:
                    tr2 -= 1
            print(path)
            self.path = path
            self.path_cost = str(graph.path_cost(path))
            self.source_idx = curr_idx
        else:
            self.result = 'Failed.'
            self.path = []
            self.path_cost = '---'

    def report(self, verbose=2):
        tmp = "%s Steps: %d Cost: %s\n" % (self.result, self.steps, self.path_cost)
        if verbose > 0:
            tmp += "Path (%d)=%s\n"  % (len(self.path), self.path)
        if verbose > 1:
            tmp += "Open (%d)=%s\n"   % (len(self.open_nodes), self.open_nodes)
            tmp += "Closed (%d)=%s\n"   % (len(self.closed), self.closed)
        if verbose > 2:
            tmp += "Route (%d)=%s\n"   % (len(self.route), self.route)
        return tmp

def SearchDijkstra(graph, source_idx, target_idx, item_idx, point_idx, limit=0):
    ''' Dijkstra Search. Expand the minimum path cost-so-far '''
    closed = set() # set - of visited nodes
    route = {} # dict of {to:from} items to find our way home
    open_nodes = PriorityQueue() # priority queue of the current leaf edges
    steps = 0 # if limit
    # add starting node, with cost-so-far (G)
    open_nodes.push( source_idx, 0.0 )
    route[source_idx] = source_idx # to:from
    # search loop
    while len(open_nodes):
        steps += 1
        leaf, cost = open_nodes.pop() # get the lowest cost-so-far node to investigate
        closed.add(leaf)
        if ((leaf == target_idx) and (item_idx in closed) and (point_idx in closed)):
            break
        else:
            idxs = graph.get_neighbours(leaf)
            for dest in idxs:
                if dest not in closed: # visited
                    cost_f = cost + graph.get_edge(leaf,dest).cost # cost_g
                    if dest in open_nodes: # old path to same node?
                        if open_nodes.peek(dest)[1] <= cost_f: # if better, keep it
                            continue
                        else: # remove the old, and the new one be added
                            open_nodes.remove(dest)
                    route[dest] = leaf # to:from
                    open_nodes.push(dest, cost_f )
        # stop early?
        if limit > 0 and steps >= limit:
            break
    # return the partial/complete path details
    return Path(graph, route, target_idx, item_idx, point_idx, open_nodes, closed, steps)

def SearchAStar(graph, source_idx, target_idx, item_idx, point_idx, limit=0):
    ''' A* Search. Expand the minimum path cost-so-far + lowest heuristic cost. '''
    closed = set() # set - of visited nodes
    route = {} # dict of {to:from} items to find our way home
    open_nodes = PriorityQueue() # priority queue of the current leaf edges
    steps = 0
    # add starting node, with F = cost-so-far (G) + heuristic (H)
    open_nodes.push(source_idx, graph.cost_h(source_idx, target_idx) )
    route[source_idx] = source_idx
    # search loop
    while len(open_nodes):
        steps += 1
        leaf, cost_f = open_nodes.pop() # get the lowest cost-so-far node to investigate
        closed.add(leaf) # set 'visited'
        if ((leaf == target_idx) and (item_idx in closed) and (point_idx in closed)):
            break
        else:
            # use the old cost_f to get the real base cost_g for the path so-far
            cost = cost_f - graph.cost_h(leaf, target_idx)
            # get new children
            idxs = graph.get_neighbours(leaf)
            for dest in idxs:
                if dest not in closed: # visited
                    cost_g = cost + graph.get_edge(leaf, dest).cost # G cost-so-far
                    cost_h = graph.cost_h(dest, target_idx) # H estimated-cost
                    cost_f = cost_g + cost_h
                    if dest in open_nodes:
                        if open_nodes.peek(dest)[1] <= cost_f:
                            continue
                        else:
                            open_nodes.remove(dest)
                    route[dest] = leaf
                    open_nodes.push(dest, cost_f)
        # stop early?
        if limit > 0 and steps >= limit:
            break
    # return the partial/complete path details
    return Path(graph, route, target_idx, item_idx, point_idx, open_nodes, closed, steps)



# A simple dictionary with string keys to each search class type.
SEARCHES = {
    'Dijkstra': SearchDijkstra,
    'AStar': SearchAStar,
}


#==============================================================================

if __name__ == '__main__':
    import graph
    # build a sample graph
    adj_list = ((0,),(1,2,3),(2,1,5),(3,1,4),(4,3,5,6),(5,2,4,6),(6,4,5))

    g = graph.SparseGraph.FromAdjacencyList(adj_list, False)
    print(g.summary())
    print(g.get_adj_list_str())

    # test the priority queue...
    pq = PriorityQueue()
    pq.push('A',2.0)
    pq.push('B',1.0)
    pq.push('C',3.0)
    print(pq.pop()) # should give (1.0, 1, 'B')

    # cost based searches
    g = graph.SparseGraph()
    g.add_node(graph.Node())
    g.add_node(graph.Node())
    g.add_node(graph.Node())
    g.add_node(graph.Node())
    g.add_node(graph.Node())
    g.add_node(graph.Node())
    g.add_node(graph.Node())
    g.add_edge(graph.Edge(1,5,2.9))
    g.add_edge(graph.Edge(1,6,1.0))
    g.add_edge(graph.Edge(2,3,3.1))
    g.add_edge(graph.Edge(3,5,0.8))
    g.add_edge(graph.Edge(4,3,3.7))
    g.add_edge(graph.Edge(5,2,1.9))
    g.add_edge(graph.Edge(5,6,3.0))
    g.add_edge(graph.Edge(6,4,1.1))
    print(g.summary())
    print(g.get_adj_list_str())
    # Dijkstra's cost based search
    print('from 5 to 3 Dijkstra:')
    print(SearchDijkstra(g,5,3))
    # A* Search
    g.cost_h = SimpleTestHeuristic
    print('from 1 to 3 A*:')
    print(SearchAStar(g,1,3))
