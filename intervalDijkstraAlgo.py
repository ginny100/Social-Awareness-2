# Python implementation of Dijkstra's algorithm for
# interval weighted graphs
# Cite NetworkX

import networkx as nx
from heapq import heappush, heappop
from itertools import count

# Going to need to add an attribute for fuzzy logic -
# comparing two adjacent edges based on what fuzzy membership
# it has compare the two edges - if the fuzzy member
# ship is any of the things then it is less than

''' 1 do fuzzy membership for all adj edges connected to a point
    2 once that is done if there are multiple of the same ones
      do drastic sum operation (to find the maximum membership)
      and add that to the cloud
'''

def fuzzy_check(l1, l2):
    # Given two lists checks the fuzzy membership and
    # returns a number
    l1_r = ((l1[1] - l1[0])/2)
    l2_r = ((l2[1] - l2[0])/2)
    
    if l1[1] < l2[0]:
        fuzzy = 1
    elif l1[0] <= l2[0] <= l1[1] < l2[1] and l1_r > 0: 
        fuzzy = -1
    elif l2[0] <= l1[0] < l1[1] <= l2[1] and l2_r > l1_r:
        fuzzy = (l2[1] - l1[1])/(2(l2_r - l1_r))
    elif l2_r == l1_r and l1[1] == l2[1]:
        fuzzy = 0.5
    else:
        fuzzy = 2

    return fuzzy

def dijkstra(G, sources, weight="weight"):
    
    G_succ = G._succ if G.is_directed() else G._adj
    
    push = heappush
    pop = heappop
    dist = {}  # dictionary of final distances
    seen = {}
    fuzzy = 0
    # fringe is heapq with 3-tuples (distance,c,node)
    # use the count c to avoid comparing nodes (may not be able to)
    c = count()
    fringe = []
    for source in sources:
        if source not in G:
            raise nx.NodeNotFound(f"Source {source} not in G")
        seen[source] = [0,0]
        push(fringe, ([0,0], next(c), source))
        
    while fringe:
        (d, _, v) = pop(fringe)
        if v in dist:
            continue  # already searched this node.
        dist[v] = d
        for u, e in G_succ[v].items():
            w = e.get('weight')
            prev_weight = dist[v]
            fuzzy = fuzzy_check(d, w)

            vu_dist = [prev_weight[i] + w[i] for i in range(len(prev_weight))]

            if u not in seen or fuzzy == 0:
                seen[u] = vu_dist
                push(fringe, (vu_dist, next(c), u))
    # The optional predecessor and path dictionaries can be accessed
    # by the caller via the pred and paths objects passed as arguments.
    return dist
                                       
'''def dijkstra(graph, initial):
    visited = {initial: [0,0]}
    path = {}
  
    nodes = set(graph.nodes)
  
    while nodes: 
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node

        if min_node is None:
            break

        nodes.remove(min_node)
        l1 = visited[min_node]
        cw_radius = ((l1[1] - l1[0])/2)

        for edge in graph.edges[min_node]:
            # Need to find Fuzzy Membership Here somehow (Use def 3?)
            # D[z] is visited[edge]
            # weight is D[u] + w(u,z)
            # Fuzzy Path Membership Test
            l2 = graph.distance[(min_node, edge)]
            l2_radius = ((l2[1] - l2[0])/2)
            if l1[1] < l2[0]:
                fuzzy = 1
            elif l1[0] <= l2[0] <= l1[1] < l2[1] and cw_radius > 0: 
                fuzzy = -1
            elif l2[0] <= l1[0] < l1[1] <= l2[1] and l2_radius > cw_radius:
                fuzzy = (l2[1] - l1[1])/(2(l2_radius - cw_radius))
            elif l2_radius == cw_radius and l1[1] == l2[1]:
                fuzzy = 0.5
            else:
                print("error")
                
            weight = sum_lists(l1, graph.distance[(min_node, edge)])
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path'''

def main():
    graph = nx.Graph()
    ''' Picture of Graph
               B
            / /\ \
          / /  \ \
        A--C---E--F
         \ \  / /
          \ \/ /
            D
    '''
    
    graph.add_edge('A', 'B', weight=[6,8])
    graph.add_edge('A', 'C', weight=[2,4])
    graph.add_edge('A', 'D', weight=[7,9])
    graph.add_edge('B', 'C', weight=[2,4])
    graph.add_edge('B', 'E', weight=[3,5])
    graph.add_edge('B', 'F', weight=[9,11])
    graph.add_edge('C', 'E', weight=[1,3])
    graph.add_edge('C', 'D', weight=[5,7])
    graph.add_edge('E', 'D', weight=[4,6])
    graph.add_edge('E', 'F', weight=[3,5])
    graph.add_edge('F', 'D', weight=[8,10])
    
    for node1, node2, data in graph.edges(data=True):
        print(node1, node2)
    
    dict1 = dijkstra(graph, 'A')

if __name__ == "__main__":
    main()
