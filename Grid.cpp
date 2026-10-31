// Primary Author: Reagan Song, 14740676

#pragma once

#include <vector>
#include <chrono>
#include <cmath>
#include <random>
#include <string>
#include <algorithm>
#include <iostream>

class SearchConfig
{
public:
    int  gridWidth;
    int  gridHeight;
    bool weighted = true;

    int  minCost;
    int  maxCost;
    int  occupiedRatio;
};

struct Position
{
    int x = 0;
    int y = 0;

    bool operator==(const Position&) const = default;
};

class Cell
{
public:
    Position  position;
    int       cost = 1;
    bool      occupied = false;

    // Render
    static constexpr char obstalceSymbol = '@';
    static constexpr char startSymbol = 'S';
    static constexpr char destinationSymbol = 'D';
};

class Grid
{
public:
    int width;
    int height;

    std::vector<std::vector<Cell>> cells;

    Position start;
    Position destination;

    Grid(int width, int height) : width(width), height(height)
    {
        cells.resize(height, std::vector<Cell>(width));

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                cells[y][x].position = { x, y };
            }
        }
    }

    Cell& GetCell(int x, int y)
    {
        return cells[y][x];
    }

    Cell& GetCell(Position position)
    {
        return cells[position.y][position.x];
    }

    bool ContainsPosition(Position position)
    {
        return  position.x >= 0 && position.x < width &&
                position.y >= 0 && position.y < height;
    }

    Cell& GetStart()
    {
        return GetCell(start);
    }

    Cell& GetDestination()
    {
        return GetCell(destination);
    }

    bool IsStart(Cell cell)
    {
        return cell.position.x == start.x && cell.position.y == start.y;
    }

    bool IsDestination(Cell cell)
    {
        return cell.position.x == destination.x && cell.position.y == destination.y;
    }

    std::vector<Position> GetAdjacentPositions(Position position)
    {
        std::vector<Position> adjacentPositions;

        Position candidates[4] =
        {
            { position.x + 1,  position.y     },
            { position.x - 1,  position.y     },
            { position.x    ,  position.y + 1 },
            { position.x    ,  position.y - 1 }
        };

        for (Position candidate : candidates)
        {
            if (!ContainsPosition(candidate))
                continue;

            if (GetCell(candidate).occupied)
                continue;

            adjacentPositions.push_back(candidate);
        }

        return adjacentPositions;
    }
};

struct SearchResult
{
    long long   executionTime = 0;
    int         processedCells = 0;
    int         edgeRelaxations = 0;
    int         shortestPathCost = -1;

    std::vector<Position> path;

    size_t peakMemoryBytes = 0; // Ignore distance/parent arrays.
};

struct DijkstraResult : SearchResult
{
    // Used in deprecated experiments
    int heapPushes = 0;
    int heapPops = 0;
};

struct DialResult : SearchResult
{
    int emptyBucketsScanned = 0;

    // Used in deprecated experiments
    int bucketInsertions = 0;
    int bucketRemovals = 0;
};

// Not required for search algorithm implementation (Rendering Only)
class GridRenderer
{
public:
    void Render(Grid grid)
    {
        std::string separator;

        for (int x = 0; x < grid.width; ++x)
            separator += "+-----";

        separator += "+";

        for (int y = 0; y < grid.height; ++y)
        {
            std::cout << separator << "\n";

            for (int x = 0; x < grid.width; ++x)
            {
                Cell cell = grid.GetCell(x, y);

                std::cout << "|";

                if (grid.IsStart(cell))
                {
                    std::cout << "  " << Cell::startSymbol << "  ";
                }
                else if (grid.IsDestination(cell))
                {
                    std::cout << "  " << Cell::destinationSymbol << "  ";
                }
                else if (cell.occupied)
                {
                    std::cout << "  " << Cell::obstalceSymbol << "  ";
                }
                else if (cell.cost < 10)
                {
                    std::cout << "  " << cell.cost << "  ";
                }
                else
                {
                    std::cout << " " << cell.cost << "  ";
                }
            }

            std::cout << "|\n";
        }

        std::cout << separator << "\n";
    }
};

// Not required for search algorithm implementation (Grid generation Only)
class GridGenerator
{
private:
    std::mt19937 rng{ std::random_device{}() };

public:
    void SetSeed(unsigned int seed)
    {
        rng.seed(seed);
    }

    int GetRandomInt(int min, int max)
    {
        std::uniform_int_distribution<int> distribution(min, max);
        return distribution(rng);
    }

    Grid GenerateGrid(const SearchConfig& config)
    {
        Grid grid(config.gridWidth, config.gridHeight);

        for (int y = 0; y < config.gridHeight; ++y)
        {
            for (int x = 0; x < config.gridWidth; ++x)
            {
                Cell& cell = grid.cells[y][x];

                if (config.weighted)
                    cell.cost = GetRandomInt(config.minCost, config.maxCost);
                else
                    cell.cost = 1;

                cell.occupied = false;
            }
        }

        grid.start = { GetRandomInt(0, config.gridWidth - 1), GetRandomInt(0, config.gridHeight - 1) };

        int startIndex = grid.start.y * config.gridWidth + grid.start.x;
        int destinationIndex = GetRandomInt(0, config.gridWidth * config.gridHeight - 2);

        if (destinationIndex >= startIndex)
            ++destinationIndex;

        grid.destination = {destinationIndex % config.gridWidth, destinationIndex / config.gridWidth };

        std::vector<int> availableCells;

        for (int i = 0; i < config.gridWidth * config.gridHeight; ++i)
        {
            if (i != startIndex && i != destinationIndex)
                availableCells.push_back(i);
        }

        for (int i = static_cast<int>(availableCells.size()) - 1; i > 0; --i)
        {
            int randomIndex = GetRandomInt(0, i);
            std::swap(availableCells[i], availableCells[randomIndex]);
        }

        int occupiedCount = config.gridWidth * config.gridHeight * config.occupiedRatio / 100;

        for (int i = 0; i < occupiedCount; ++i)
        {
            int index = availableCells[i];

            int x = index % config.gridWidth;
            int y = index / config.gridWidth;

            grid.cells[y][x].occupied = true;
        }

        return grid;
    }

    void ApplyWeights(Grid& grid, const SearchConfig& config)
    {
        for (int y = 0; y < grid.height; ++y)
        {
            for (int x = 0; x < grid.width; ++x)
            {
                Cell& cell = grid.cells[y][x];

                if (config.weighted)
                    cell.cost = GetRandomInt(config.minCost, config.maxCost);
                else
                    cell.cost = 1;
            }
        }
    }
};
