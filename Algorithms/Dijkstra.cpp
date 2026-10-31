// Primary Author: Yoonji Byun, 14740664

#include <vector>
#include <iostream>
#include <queue>
#include <limits>
#include <algorithm>

class Dijkstra
{
private:
    struct QueueEntry
    {
        int distance;
        Position position;
    };

    struct CompareDistance
    {
        bool operator()(const QueueEntry& a, const QueueEntry& b)
        {
            return a.distance > b.distance;
        }
    };

    template<typename T, typename Container, typename Compare>
    class MeasurablePriorityQueue : public std::priority_queue<T, Container, Compare>
    {
    public:
        size_t AllocatedMemory() const
        {
            return this->c.capacity() * sizeof(T);
        }
    };

public:
    DijkstraResult FindShortestDistance(Grid& grid)
    {
        DijkstraResult result;

        // best known distance
        std::vector<std::vector<int>> distance(
            grid.height,
            std::vector<int>(grid.width, std::numeric_limits<int>::max())
        );
        distance[grid.start.y][grid.start.x] = 0;

        // binary-heap priority queue
        MeasurablePriorityQueue<
            QueueEntry,
            std::vector<QueueEntry>,
            CompareDistance
        > queue;
        queue.push({ 0, grid.start });
        ++result.heapPushes;

        result.peakMemoryBytes = queue.AllocatedMemory();

        // parent vector to trace path
        std::vector<std::vector<Position>> parent(
            grid.height,
            std::vector<Position>(grid.width)
        );

        // main Dijkstra loop
        while (!queue.empty())
        {
            QueueEntry current = queue.top();
            queue.pop();
            ++result.heapPops;

            // ignore older entries if better path is already found
            if (current.distance > distance[current.position.y][current.position.x])
                continue;

            ++result.processedCells; // not count old entries

            // if destination is poped, its shortest distance is final
            if (current.position == grid.destination)
            {
                result.shortestPathCost = current.distance;
                break;
            }

            for (Position neighbour : grid.GetAdjacentPositions(current.position))
            {
                // relaxation = checking dist[u] + w(u, v) < dist[v]
                ++result.edgeRelaxations;

                int newDistance = current.distance + grid.GetCell(neighbour).cost;

                if (newDistance < distance[neighbour.y][neighbour.x])
                {
                    distance[neighbour.y][neighbour.x] = newDistance;
                    parent[neighbour.y][neighbour.x] = current.position;
                    queue.push({ newDistance, neighbour });
                    ++result.heapPushes;

                    result.peakMemoryBytes = std::max(result.peakMemoryBytes, queue.AllocatedMemory());
                }
            }
        }

        if (result.shortestPathCost != -1)
        {
            Position current = grid.destination;

            while (current != grid.start)
            {
                result.path.push_back(current);
                current = parent[current.y][current.x];
            }

            result.path.push_back(grid.start);
            std::reverse(result.path.begin(), result.path.end());
        }

        return result;
    }
};
