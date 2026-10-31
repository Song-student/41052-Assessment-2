// Primary Author: Reagan Song, 14740676

#include <vector>
#include <algorithm>

class Dial {
public:
    DialResult FindShortestDistance(Grid& grid, int maxCost)
    {
        DialResult result;

        // Number of buckets = W + 1 (W = max edge cost)
        const int bucketCount = maxCost + 1;

        // Circular buckets
        std::vector<std::vector<Position>> buckets(bucketCount);

        size_t bucketMemory = buckets.capacity() * sizeof(std::vector<Position>);
        result.peakMemoryBytes = bucketMemory;

        // Best known distance
        std::vector<std::vector<int>> distance(grid.height, std::vector<int>(grid.width, -1));

        // Parent to trace path
        std::vector<std::vector<Position>> parent(grid.height, std::vector<Position>(grid.width, { -1, -1 }));

        Position start = grid.start;
        Position destination = grid.destination;

        distance[start.y][start.x] = 0;

        size_t oldCapacity = buckets[0].capacity();
        buckets[0].push_back(start);
        result.bucketInsertions++;

        bucketMemory += (buckets[0].capacity() - oldCapacity) * sizeof(Position);
        result.peakMemoryBytes = std::max(result.peakMemoryBytes, bucketMemory);

        int currentDistance = 0;
        int bucketItemCount = 1;

        // Main Dial's loop
        while (bucketItemCount > 0)
        {
            // Find the next non-empty distance. (to find the next bucket)
            // O(WV)
            while (buckets[currentDistance % bucketCount].empty())
            {
                ++currentDistance;
                ++result.emptyBucketsScanned;
            }

            // Pop the last position in the bucket. (last element is the fastest to remove, O(1))
            int currentBucketIndex = currentDistance % bucketCount;
            Position current = buckets[currentBucketIndex].back();
            buckets[currentBucketIndex].pop_back();
            ++result.bucketRemovals;
            bucketItemCount--;

            // Ignore older entries if a better path is already found.
            if (distance[current.y][current.x] != currentDistance)
                continue;

            ++result.processedCells;

            // If destination is processed, its shortest distance is final.
            if (current == destination)
            {
                result.shortestPathCost = currentDistance;
                break;
            }

            // Relaxation for adjacent edges
            // O(E)
            std::vector<Position> adjacentPositions = grid.GetAdjacentPositions(current);
            for (Position next : adjacentPositions)
            {
                ++result.edgeRelaxations;

                Cell nextCell = grid.GetCell(next);
                int newDistance = currentDistance + nextCell.cost;

                // Update if a shorter path to the adjacent is found.
                if (distance[next.y][next.x] == -1 || newDistance < distance[next.y][next.x])
                {
                    distance[next.y][next.x] = newDistance;
                    parent[next.y][next.x] = current;

                    int bucketIndex = newDistance % bucketCount;

                    size_t oldCapacity = buckets[bucketIndex].capacity();
                    buckets[bucketIndex].push_back(next);
                    ++result.bucketInsertions;
                    ++bucketItemCount;

                    bucketMemory += (buckets[bucketIndex].capacity() - oldCapacity) * sizeof(Position);
                    result.peakMemoryBytes = std::max(result.peakMemoryBytes, bucketMemory);
                }
            }
        }

        // Destination not found
        if (distance[destination.y][destination.x] == -1)
            return result;

        // Make path
        Position current = destination;
        while (current != start)
        {
            result.path.push_back(current);
            current = parent[current.y][current.x];
        }
        result.path.push_back(start);
        std::reverse(result.path.begin(), result.path.end());

        return result;
    }
};
