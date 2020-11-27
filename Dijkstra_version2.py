'''
Dijkstra's algorithm for undirected weighted graph
Version 2
Source: protonx.ai
'''
# Create a graph
N = 7

edges = [ [0, 2, 1], 
         [0, 1, 2], 
         [0, 6, 3], 
         [1, 5, 10], 
         [1, 4, 15], 
         [2, 1, 4], 
         [2, 3, 2], 
         [3, 4, 3], 
         [4, 5, 5], 
         [6, 1, 3] ]

inf = float('inf')
graph = [[] for _ in range(N)]
dist = [inf for _ in range(N)]
path = [-1 for _ in range(N)]

for edge in edges:
    v, u, w = edge
    graph[v].append((u, w))
    graph[u].append((v, w))

# Code Dijkstra
import heapq
def dijkstra(s): # s is the starting vertex
    minHeap = []
    dist[s] = 0
    heapq.heappush(minHeap, (0, s))
    while minHeap:
        v = heapq.heappop(minHeap)
        vWeight, vID = v   # vID is the value of the vertex v, vWeight is the shortest distance from s to v
        for u in graph[vID]:
            uID, uWeight = u
            if vWeight + uWeight < dist[uID]:
                dist[uID] = vWeight + uWeight
                path[uID] = vID
                heapq.heappush(minHeap, (vWeight + uWeight, uID))

# Test
dijkstra(0)
print(path)
print(dist)
