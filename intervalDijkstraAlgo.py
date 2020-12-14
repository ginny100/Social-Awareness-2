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
        fuzzy = (l2[1] - l1[1])/(2*(l2_r - l1_r))
    elif l2_r == l1_r and l1[1] == l2[1]:
        fuzzy = 0.5
    else:
        fuzzy = 0

    return fuzzy

def drastic_sum(fu_1, fu_2):
    
    if fu_2 == 0:
        result = fu_1
    elif fu_1 == 0:
        result = fu_2
    else:
        result = 1
    
    return result

def dijkstra(graph, initial):
    visited = {initial: [0,0]}
    path = {}

    nodes = set(graph.nodes)

    while nodes: 
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                else:
                    fuzzy = fuzzy_check(visited[node], visited[min_node])
                    if fuzzy != 0: # Need to check member here and test them against the min node
                        min_node = node

        if min_node is None:
            break

        nodes.remove(min_node)
        current_list = visited[min_node]

        for u, v in graph.edges(min_node):
            fuzzy = fuzzy_check(current_list, graph.edges[u,v]['weight'])
            if v not in visited or fuzzy != 0:
                visited[v] = graph.edges[u,v]['weight']
                path[v] = min_node    

    return visited, path

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
    
    graph.add_edge('A', 'B', weight=[2,8])
    graph.add_edge('A', 'C', weight=[3,4])
    graph.add_edge('A', 'D', weight=[3,5])
    graph.add_edge('B', 'C', weight=[1,4])
    graph.add_edge('B', 'E', weight=[3,5])
    graph.add_edge('B', 'F', weight=[9,11])
    graph.add_edge('C', 'E', weight=[4,6])
    graph.add_edge('C', 'D', weight=[5,7])
    graph.add_edge('E', 'D', weight=[4,6])
    graph.add_edge('E', 'F', weight=[2,4])
    graph.add_edge('F', 'D', weight=[8,10])
    
    v, path = dijkstra(graph, 'A')
    print('Path :', path)

if __name__ == "__main__":
    main()
