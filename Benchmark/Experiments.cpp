// Primary Author: Reagan Song, 14740676

#include <stdexcept>
#include <iomanip>

// Shared Settings for All Experiments
struct ExperimentSettingsBase
{
    int repeatCount = 10;
    int gridCount = 10;
    unsigned int randomSeed = 12345;
};

struct AlgorithmExecutionResult
{
    DijkstraResult dijkstraResult;
    DialResult dialResult;

    long long dijkstraTotalTime = 0;
    long long dialTotalTime = 0;
};
class ExperimentBase
{
protected:
    GridGenerator gridGenerator;

    struct RejectionRecord
    {
        std::string condition;
        int rejected = 0;
    };
    std::vector<RejectionRecord> rejectionRecords;
    int totalRejected = 0;

    void RecordRejections(const std::string& condition, int rejected)
    {
        rejectionRecords.push_back({ condition, rejected });
        totalRejected += rejected;
    }

    void PrintRejectionSummary(const std::string& title)
    {
        std::cout << "\n========================================\n"
            << title << " - UNREACHABLE GRID SUMMARY\n"
            << "========================================\n";
        for (const auto& item : rejectionRecords)
            std::cout << item.condition << ": " << item.rejected << " rejected\n";
        std::cout << "Total Unreachable Grids Rejected: " << totalRejected << "\n";
        rejectionRecords.clear();
        totalRejected = 0;
    }

    // Reachability depends on obstacles, not positive edge-weight values.
    static bool IsReachable(const Grid& input)
    {
        Grid& grid = const_cast<Grid&>(input); // GetCell is non-const in the existing API.
        const int w = grid.width, h = grid.height;
        const Position start = grid.start, goal = grid.destination;
        if (w <= 0 || h <= 0 || start.x < 0 || start.y < 0 || start.x >= w || start.y >= h ||
            goal.x < 0 || goal.y < 0 || goal.x >= w || goal.y >= h ||
            grid.GetCell(start.x, start.y).occupied || grid.GetCell(goal.x, goal.y).occupied)
            return false;
        std::vector<unsigned char> visited(static_cast<size_t>(w) * h, 0);
        std::vector<Position> queue;
        queue.reserve(static_cast<size_t>(w) * h);
        auto index = [w](int x, int y) { return static_cast<size_t>(y) * w + x; };
        visited[index(start.x, start.y)] = 1;
        queue.push_back(start);
        for (size_t head = 0; head < queue.size(); ++head)
        {
            Position p = queue[head];
            if (p == goal) return true;
            const int dx[4] = { 1, -1, 0, 0 };
            const int dy[4] = { 0, 0, 1, -1 };
            for (int k = 0; k < 4; ++k)
            {
                int x = p.x + dx[k], y = p.y + dy[k];
                if (x < 0 || x >= w || y < 0 || y >= h || visited[index(x, y)] ||
                    grid.GetCell(x, y).occupied) continue;
                visited[index(x, y)] = 1;
                queue.push_back({ x, y });
            }
        }
        return false;
    }

    Grid GenerateReachableGrid(const SearchConfig& config, int& rejected)
    {
        for (int attempt = 0; attempt < 100000; ++attempt)
        {
            Grid grid = gridGenerator.GenerateGrid(config);
            if (IsReachable(grid)) return grid;
            ++rejected;
        }
        throw std::runtime_error("Unable to generate a reachable grid after 100000 attempts.");
    }


    struct GridStats
    {
        int vertices = 0;
        int edges = 0;

        int minWeight = 0;
        int maxWeight = 0;

        double mean = 0;
        double median = 0;
    };

    struct AverageTimes
    {
        long long dijkstraNs = 0, dialNs = 0;
        double dijkstraNsExact = 0;
        double dialNsExact = 0;
    };

    // Sample standard deviation across independent grids (Welford's method).
    // Each observation is the mean of the timed runs on one grid.
    struct RunningStats
    {
        int count = 0;
        double mean = 0.0;
        double m2 = 0.0;

        void Add(double value)
        {
            ++count;
            double delta = value - mean;
            mean += delta / count;
            m2 += delta * (value - mean);
        }

        double StdDev() const
        {
            return count > 1 ? std::sqrt(std::max(0.0, m2 / (count - 1))) : 0.0;
        }
    };

    // Print additional statistics without changing the surrounding output precision.
    static void PrintStdDev(const char* label, double value, const char* unit = "")
    {
        const auto oldPrecision = std::cout.precision();
        std::cout << label << ": " << std::setprecision(6) << value << unit << "\n";
        std::cout.precision(oldPrecision);
    }

    // Aggregates one result per independent grid (timings are already averaged over repeats).
    struct GridAverages
    {
        double vertices = 0, edges = 0, weightMean = 0, weightMedian = 0;
        double dTime = 0, dialTime = 0;
        double dMemory = 0, dialMemory = 0;
        double dProcessed = 0, dialProcessed = 0;
        double dRelaxations = 0, dialRelaxations = 0;
        double emptyScans = 0, costTotal = 0;
        RunningStats dTimeSamples, dialTimeSamples, ratioSamples;
        int count = 0, reachableCount = 0;

        void Add(const GridStats& stats, const AlgorithmExecutionResult& result, const AverageTimes& times)
        {
            ++count;
            vertices += stats.vertices;
            edges += stats.edges;
            weightMean += stats.mean;
            weightMedian += stats.median;
            dTime += times.dijkstraNsExact;
            dialTime += times.dialNsExact;
            dTimeSamples.Add(times.dijkstraNsExact / 1000000.0);
            dialTimeSamples.Add(times.dialNsExact / 1000000.0);
            if (times.dijkstraNsExact > 0.0)
                ratioSamples.Add(times.dialNsExact / times.dijkstraNsExact);
            dMemory += result.dijkstraResult.peakMemoryBytes;
            dialMemory += result.dialResult.peakMemoryBytes;
            dProcessed += result.dijkstraResult.processedCells;
            dialProcessed += result.dialResult.processedCells;
            dRelaxations += result.dijkstraResult.edgeRelaxations;
            dialRelaxations += result.dialResult.edgeRelaxations;
            emptyScans += result.dialResult.emptyBucketsScanned;
            if (result.dialResult.shortestPathCost >= 0)
            {
                costTotal += result.dialResult.shortestPathCost;
                ++reachableCount;
            }
        }

        double V() const { return vertices / count; }
        double E() const { return edges / count; }
        double Mean() const { return weightMean / count; }
        double Median() const { return weightMedian / count; }
        double DTime() const { return dTime / count / 1000000.0; }
        double DialTime() const { return dialTime / count / 1000000.0; }
        double DTimeStdDev() const { return dTimeSamples.StdDev(); } // ms
        double DialTimeStdDev() const { return dialTimeSamples.StdDev(); } // ms
        double RatioStdDev() const { return ratioSamples.StdDev(); } // dimensionless
        double DMemory() const { return dMemory / count / 1024.0; }
        double DialMemory() const { return dialMemory / count / 1024.0; }
        double DProcessed() const { return dProcessed / count; }
        double DialProcessed() const { return dialProcessed / count; }
        double DRelaxations() const { return dRelaxations / count; }
        double DialRelaxations() const { return dialRelaxations / count; }
        double EmptyScans() const { return emptyScans / count; }
        double Cost() const { return reachableCount ? costTotal / reachableCount : -1.0; }
        double Ratio() const { return dialTime / dTime; }
    };

    void Accumulate(GridAverages& totals, Grid& grid, int maxCost, int repeats, bool includeMedian = false)
    {
        GridStats stats = AnalyzeGrid(grid, includeMedian);
        AlgorithmExecutionResult result = RunAlgorithms(grid, maxCost, repeats);
        ValidateMatchingCost(result.dijkstraResult, result.dialResult);
        totals.Add(stats, result, GetAverageTimes(result, repeats));
    }

    static SearchConfig MakeConfig(int width, int height, int occupied, int minCost, int maxCost, bool weighted = true)
    {
        SearchConfig config;
        config.gridWidth = width;
        config.gridHeight = height;
        config.occupiedRatio = occupied;
        config.weighted = weighted;
        config.minCost = minCost;
        config.maxCost = maxCost;
        return config;
    }

    static void PrintHeader(const std::string& title, bool hash = false)
    {
        const char* border = hash ? "########################################" : "========================================";
        std::cout << "\n\n" << border << "\n" << title << "\n" << border << "\n";
    }

    static void PrintCaseHeader(const std::string& name)
    {
        std::cout << "\n========================================\n" << name
            << "\n========================================\n";
    }

    template<class Func>
    static void ForEachFreeCell(Grid& grid, Func action)
    {
        for (int y = 0; y < grid.height; ++y)
            for (int x = 0; x < grid.width; ++x)
                if (!grid.GetCell(x, y).occupied)
                    action(x, y, grid.GetCell(x, y));
    }

    static void SetAllFreeWeights(Grid& grid, int weight)
    {
        ForEachFreeCell(grid, [weight](int, int, Cell& cell) { cell.cost = weight; });
    }

    static std::vector<Position> GetFreePositions(Grid& grid, bool excludeStart = false)
    {
        std::vector<Position> positions;
        ForEachFreeCell(grid, [&](int x, int y, Cell&)
            {
                Position p = { x, y };
                if (!excludeStart || p != grid.start)
                    positions.push_back(p);
            });
        return positions;
    }

    void ShufflePositions(std::vector<Position>& positions)
    {
        for (int i = static_cast<int>(positions.size()) - 1; i > 0; --i)
        {
            int j = gridGenerator.GetRandomInt(0, i);
            std::swap(positions[i], positions[j]);
        }
    }

    static void AssignFirstWeights(Grid& grid, const std::vector<Position>& positions,
        int defaultWeight, int firstWeight, int count)
    {
        for (Position p : positions)
            grid.GetCell(p.x, p.y).cost = defaultWeight;
        for (int i = 0; i < count; ++i)
        {
            Position p = positions[i];
            grid.GetCell(p.x, p.y).cost = firstWeight;
        }
    }

    // computeMedian=true is used by General only; other experiments do not sort.
    static GridStats AnalyzeGrid(Grid& grid, bool computeMedian = false)
    {
        GridStats stats;
        std::vector<int> weights;
        // Preserve original General's double accumulation and Unified's integer accumulation.
        double doubleTotal = 0.0;
        long long integerTotal = 0;
        bool first = true;
        ForEachFreeCell(grid, [&](int x, int y, Cell& cell)
            {
                ++stats.vertices;
                doubleTotal += cell.cost;
                integerTotal += cell.cost;
                if (computeMedian) weights.push_back(cell.cost);
                if (first)
                {
                    stats.minWeight = stats.maxWeight = cell.cost;
                    first = false;
                }
                else
                {
                    stats.minWeight = std::min(stats.minWeight, cell.cost);
                    stats.maxWeight = std::max(stats.maxWeight, cell.cost);
                }
                if (x + 1 < grid.width && !grid.GetCell(x + 1, y).occupied) ++stats.edges;
                if (y + 1 < grid.height && !grid.GetCell(x, y + 1).occupied) ++stats.edges;
            });
        if (stats.vertices > 0)
            stats.mean = computeMedian ? doubleTotal / stats.vertices
            : static_cast<double>(integerTotal) / stats.vertices;
        if (computeMedian && !weights.empty())
        {
            std::sort(weights.begin(), weights.end());
            int middle = static_cast<int>(weights.size()) / 2;
            stats.median = weights.size() % 2 == 0
                ? (weights[middle - 1] + weights[middle]) / 2.0 : weights[middle];
        }
        return stats;
    }

    AlgorithmExecutionResult RunAlgorithms(Grid& grid, int maxCost, int repeatCount, bool runDijkstra = true)
    {
        AlgorithmExecutionResult result;
        Dijkstra dijkstra;
        Dial dial;

        // Verify correctness before collecting timing measurements.
        if (runDijkstra)
        {
            DijkstraResult d = dijkstra.FindShortestDistance(grid);
            DialResult a = dial.FindShortestDistance(grid, maxCost);
            ValidateMatchingCost(d, a);
        }

        // Two additional warm-up runs per algorithm (excluded from timing).
        for (int i = 0; i < 2; ++i)
        {
            if (runDijkstra)
                dijkstra.FindShortestDistance(grid);
            dial.FindShortestDistance(grid, maxCost);
        }

        for (int i = 0; i < repeatCount; ++i)
        {
            if (runDijkstra)
            {
                auto start = std::chrono::high_resolution_clock::now();
                result.dijkstraResult = dijkstra.FindShortestDistance(grid);
                auto end = std::chrono::high_resolution_clock::now();
                result.dijkstraTotalTime += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            }

            auto start = std::chrono::high_resolution_clock::now();
            result.dialResult = dial.FindShortestDistance(grid, maxCost);
            auto end = std::chrono::high_resolution_clock::now();
            result.dialTotalTime += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        }

        return result;
    }

    static AverageTimes GetAverageTimes(const AlgorithmExecutionResult& result, int repeatCount)
    {
        AverageTimes times;
        times.dijkstraNs = result.dijkstraTotalTime / repeatCount;
        times.dialNs = result.dialTotalTime / repeatCount;
        times.dijkstraNsExact = static_cast<double>(result.dijkstraTotalTime) / repeatCount;
        times.dialNsExact = static_cast<double>(result.dialTotalTime) / repeatCount;
        return times;
    }

    static void ValidateMatchingCost(const DijkstraResult& d, const DialResult& dial)
    {
        if (d.shortestPathCost != dial.shortestPathCost)
            throw std::runtime_error(
                "Shortest path costs do not match: Dijkstra=" +
                std::to_string(d.shortestPathCost) + ", Dial=" +
                std::to_string(dial.shortestPathCost));
    }
};


#pragma region Main Experiments
// Main Experiment 1: Grid Factors, Edge-Weight Configurations, and Performance Crossover
struct GeneralExperimentSettings : ExperimentSettingsBase
{
    // Common grid configuration
    int gridWidth = 100;
    int gridHeight = 100;
    int occupiedRatio = 20;

    // Set 1 - Grid Factors
    // Baseline A: Wmin = 1, Wmax = 10
    int set1MinCost = 1;
    int set1MaxCost = 10;

    // Baseline B: Wmin = 1, Wmax = 100
    int set2MinCost = 1;
    int set2MaxCost = 100;

    // Grid size and obstacle ratio variations under both baselines
    std::vector<int> gridSizes = { 50, 100, 200, 400 };
    std::vector<int> obstacleRatios = { 0, 10, 20, 30 };

    // Set 2 - Edge-Weight Configurations
    // First test group: maximum, minimum, and uniform edge weights
    std::vector<int> set1MaxCosts = { 10, 100, 1000, 10000 };
    std::vector<int> set1MinCosts = { 1, 250, 500, 1000 };
    int              set1MinCostTestMax = 1000;
    std::vector<int> set1FixedWeights = { 1, 10, 100, 1000, 10000 };

    // Second test group: larger maximum, minimum, and uniform edge weights
    std::vector<int> set2MaxCosts = { 100, 1000, 10000, 100000 };
    std::vector<int> set2MinCosts = { 1, 2500, 5000, 10000 };
    int              set2MinCostTestMax = 10000;
    std::vector<int> set2FixedWeights = { 100, 1000, 10000, 100000 };

    // Set 3 - Performance Crossover
    // Combined effects of grid size and maximum edge weight
    std::vector<int> crossoverGridSizes = { 10, 50, 100, 200, 300, 500, 700, 1000,1500 };
    std::vector<int> crossoverMaxCosts = { 10, 100, 1000, 10000 };
};

// Main Experiment 2: Weight Scale and Weight Distribution
struct WeightScaleAndDistributionExperimentSettings : ExperimentSettingsBase
{
    // Common grid configuration
    int gridWidth = 100;
    int gridHeight = 100;
    int occupiedRatio = 20;

    // Base weight configuration
    int minCost = 1;
    int maxCost = 1000;

    // Set 1 - Weight Scale
    // Increase minimum and maximum edge weights together
    int weightScaleMinCost = 1;
    int weightScaleMaxCost = 10;
    std::vector<std::pair<int, int>> sensitivityWeightPairs =
    {
        { 1, 10 },
        { 10, 100 },
        { 100, 1000 },
        { 1000, 10000 },
        { 10000, 100000 }
    };

    // Set 2 - Weight Distribution
    // Compare uniform weights of 500 with a non-uniform distribution
    // containing 95% low-weight cells and 5% high-weight cells
    int fixedWeight = 500;
    double highWeightRatio = 0.05;
    int lowWeight = 1;
    int highWeight = 1000;
};

// Main Experiment 2: Mean and Maximum Edge Weights
struct MeanAndMaximumEdgeWeightsExperimentSettings : ExperimentSettingsBase
{
    // Common grid configuration
    int gridWidth = 100;
    int gridHeight = 100;
    int occupiedRatio = 20;

    // Base weight configuration
    int minCost = 1;
    int maxCost = 1000;

    // Set 3 - Mean Edge Weight
    // Vary mean edge weight while keeping Wmax fixed at 1000
    int fixedMaxWeight = 1000;
    std::vector<int> normalWeights = { 1, 100, 300, 500, 800 };

    // Set 4 - Maximum Edge Weight
    // Vary Wmax while maintaining Wmean at approximately 500
    int targetMean = 500;
    std::vector<int> maxWeights = { 500, 1000, 2000, 5000, 10000 };
};

class GeneralExperiment : public ExperimentBase
{
    GeneralExperimentSettings settings;
public:
    explicit GeneralExperiment(const GeneralExperimentSettings& s = GeneralExperimentSettings()) : settings(s) {}
    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);
        std::cout << std::fixed << std::setprecision(3);
        RunExperimentSet1();
        PrintRejectionSummary("EXPERIMENT SET 1");
        RunExperimentSet2();
        PrintRejectionSummary("EXPERIMENT SET 2");
        RunCrossoverAnalysis();
        PrintRejectionSummary("CROSSOVER ANALYSIS");
    }
private:
    // Experiment Set 1: baseline weights [1, 10].
    void RunExperimentSet1()
    {
        RunSet(1, settings.set1MinCost, settings.set1MaxCost, settings.set1MaxCosts,
            settings.set1MinCosts, settings.set1MinCostTestMax, settings.set1FixedWeights);
    }

    // Experiment Set 2: baseline weights [1, 100].
    void RunExperimentSet2()
    {
        RunSet(2, settings.set2MinCost, settings.set2MaxCost, settings.set2MaxCosts,
            settings.set2MinCosts, settings.set2MinCostTestMax, settings.set2FixedWeights);
    }

    void PrintGeneralAverages(const std::string& name, const SearchConfig& config, const GridAverages& t)
    {
        PrintCaseHeader(name);
        std::cout << "Grid Size: " << config.gridWidth << " x " << config.gridHeight << "\n";
        std::cout << "Obstacle Ratio: " << config.occupiedRatio << "%\n";
        std::cout << "Wmin: " << config.minCost << "\n";
        std::cout << "Wmax: " << config.maxCost << "\n";
        std::cout << "V: " << t.V() << "\n";
        std::cout << "E: " << t.E() << "\n";
        std::cout << "Wmean: " << t.Mean() << "\n";
        std::cout << "Wmedian: " << t.Median() << "\n";
        std::cout << "\nDijkstra\n";
        std::cout << "Execution Time: " << t.DTime() << " ms\n";
        PrintStdDev("Execution Time Std Dev", t.DTimeStdDev(), " ms");
        std::cout << "Processed Cells: " << t.DProcessed() << "\n";
        std::cout << "Edge Relaxations: " << t.DRelaxations() << "\n";
        std::cout << "Cost: " << t.Cost() << "\n";
        std::cout << "\nDial's Algorithm\n";
        std::cout << "Execution Time: " << t.DialTime() << " ms\n";
        PrintStdDev("Execution Time Std Dev", t.DialTimeStdDev(), " ms");
        std::cout << "Processed Cells: " << t.DialProcessed() << "\n";
        std::cout << "Edge Relaxations: " << t.DialRelaxations() << "\n";
        std::cout << "Cost: " << t.Cost() << "\n";
        std::cout << "Dijkstra Peak Memory: " << t.DMemory() << " KB\n";
        std::cout << "Dial Peak Memory: " << t.DialMemory() << " KB\n";
    }

    void RunGeneralCase(const std::string& name, const SearchConfig& config, int repeats)
    {
        GridAverages totals;
        int rejected = 0;
        for (int i = 0; i < settings.gridCount; ++i)
        {
            Grid grid = GenerateReachableGrid(config, rejected);
            Accumulate(totals, grid, config.maxCost, repeats, true);
        }
        PrintGeneralAverages(name, config, totals);
        RecordRejections(name, rejected);
    }

    template<class Modify, class Label>
    void RunVariants(const SearchConfig& baseline, const std::vector<int>& values,
        Modify modify, Label label, bool reuseLayout)
    {
        std::vector<Grid> baseGrids;
        int sharedRejected = 0;
        if (reuseLayout)
        {
            baseGrids.reserve(settings.gridCount);
            for (int i = 0; i < settings.gridCount; ++i)
                baseGrids.push_back(GenerateReachableGrid(baseline, sharedRejected));
        }
        for (int value : values)
        {
            SearchConfig config = baseline;
            modify(config, value);
            GridAverages totals;
            int rejected = 0;
            for (int i = 0; i < settings.gridCount; ++i)
            {
                Grid grid = reuseLayout ? baseGrids[i] : GenerateReachableGrid(config, rejected);
                if (reuseLayout)
                    gridGenerator.ApplyWeights(grid, config);
                Accumulate(totals, grid, config.maxCost, settings.repeatCount, true);
            }
            PrintGeneralAverages(label(value), config, totals);
            if (!reuseLayout) RecordRejections(label(value), rejected);
        }
        if (reuseLayout)
            RecordRejections("Shared base grids for " + label(values.front()) + " ...", sharedRejected);
    }

    void RunSet(int index, int minCost, int maxCost,
        const std::vector<int>& maxCosts, const std::vector<int>& minCosts,
        int minTestMax, const std::vector<int>& fixedWeights)
    {
        PrintHeader("EXPERIMENT SET " + std::to_string(index) + "\nBaseline Wmin = " +
            std::to_string(minCost) + ", Wmax = " + std::to_string(maxCost), true);
        SearchConfig baseline = MakeConfig(settings.gridWidth, settings.gridHeight,
            settings.occupiedRatio, minCost, maxCost);
        RunGeneralCase("Baseline", baseline, settings.repeatCount);
        RunVariants(baseline, settings.gridSizes,
            [](SearchConfig& c, int v) { c.gridWidth = c.gridHeight = v; },
            [](int v) { return "Grid Size: " + std::to_string(v) + " x " + std::to_string(v); }, false);
        RunObstacleRatioVariants(baseline);
        RunVariants(baseline, maxCosts,
            [](SearchConfig& c, int v) { c.minCost = 1; c.maxCost = v; },
            [](int v) { return "Wmax: " + std::to_string(v); }, true);
        RunVariants(baseline, minCosts,
            [minTestMax](SearchConfig& c, int v) { c.minCost = v; c.maxCost = minTestMax; },
            [minTestMax](int v) { return "Wmin: " + std::to_string(v) + ", Wmax: " + std::to_string(minTestMax); }, true);
        RunVariants(baseline, fixedWeights,
            [](SearchConfig& c, int v) { c.weighted = true; c.minCost = c.maxCost = v; },
            [](int v) { return "Fixed Weight: " + std::to_string(v); }, true);
        RunVariants(baseline, settings.gridSizes,
            [](SearchConfig& c, int v) { c.gridWidth = c.gridHeight = v; c.weighted = false; c.minCost = c.maxCost = 1; },
            [](int v) { return "Unweighted: " + std::to_string(v) + " x " + std::to_string(v); }, false);
    }

    void RunObstacleRatioVariants(const SearchConfig& baseline)
    {
        SearchConfig baseConfig = baseline;
        baseConfig.occupiedRatio = 0;

        struct ObstacleLayout
        {
            Grid grid;
            std::vector<Position> positions;
        };
        std::vector<ObstacleLayout> layouts;
        layouts.reserve(settings.gridCount);

        int rejected = 0;
        const int strictestRatio = *std::max_element(settings.obstacleRatios.begin(), settings.obstacleRatios.end());
        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            // Reject entire paired layout if the highest obstacle ratio is unreachable.
            for (int attempt = 0; ; ++attempt)
            {
                if (attempt >= 100000)
                    throw std::runtime_error("Unable to generate a reachable paired obstacle layout.");
                Grid baseGrid = gridGenerator.GenerateGrid(baseConfig);
                std::vector<Position> positions;
                for (int y = 0; y < baseGrid.height; ++y)
                    for (int x = 0; x < baseGrid.width; ++x)
                    {
                        Position p = { x, y };
                        if (p != baseGrid.start && p != baseGrid.destination)
                            positions.push_back(p);
                    }
                ShufflePositions(positions);
                Grid strictestGrid = baseGrid;
                const int occupiedCount = strictestGrid.width * strictestGrid.height * strictestRatio / 100;
                for (int i = 0; i < occupiedCount; ++i)
                {
                    Position p = positions[i];
                    strictestGrid.GetCell(p.x, p.y).occupied = true;
                }
                if (IsReachable(strictestGrid))
                {
                    layouts.push_back({ std::move(baseGrid), std::move(positions) });
                    break;
                }
                ++rejected;
            }
        }
        RecordRejections("Obstacle Ratio (paired layouts, checked at " +
            std::to_string(strictestRatio) + "%)", rejected);

        for (int ratio : settings.obstacleRatios)
        {
            SearchConfig config = baseline;
            config.occupiedRatio = ratio;
            GridAverages totals;
            for (const auto& layout : layouts)
            {
                Grid grid = layout.grid;
                int occupiedCount = grid.width * grid.height * ratio / 100;
                for (int i = 0; i < occupiedCount; ++i)
                {
                    Position position = layout.positions[i];
                    grid.GetCell(position.x, position.y).occupied = true;
                }
                Accumulate(totals, grid, config.maxCost, settings.repeatCount, true);
            }
            PrintGeneralAverages("Obstacle Ratio: " + std::to_string(ratio) + "%", config, totals);
        }
    }
    void RunCrossoverAnalysis()
    {
        PrintHeader("CROSSOVER ANALYSIS - GRID SIZE x WMAX", true);

        std::cout << "Obstacle Ratio: " << settings.occupiedRatio << "%\n";
        std::cout << "Wmin: " << settings.set1MinCost << "\n";

        std::cout << "Random Grids: " << settings.gridCount << "\n";
        std::cout << "Runs Per Grid: " << settings.repeatCount << "\n";

        std::cout << "\nSpeed Ratio = Dial Time / Dijkstra Time\n";
        std::cout << "Ratio < 1: Dial faster\n";
        std::cout << "Ratio > 1: Dijkstra faster\n\n";

        // Table header
        std::cout << std::left << std::setw(16) << "Grid Size / W";

        for (int maxCost : settings.crossoverMaxCosts)
        {
            std::cout << std::right << std::setw(14) << maxCost;
        }

        std::cout << "\n";

        // Keep the original speed-ratio table, then print a matching spread table.
        std::vector<std::vector<double>> ratioStdDevRows;

        // Test each grid size
        for (int gridSize : settings.crossoverGridSizes)
        {
            SearchConfig baseline = MakeConfig(
                gridSize,
                gridSize,
                settings.occupiedRatio,
                settings.set1MinCost,
                settings.crossoverMaxCosts.front()
            );

            // Generate shared grid layouts
            std::vector<Grid> baseGrids;
            baseGrids.reserve(settings.gridCount);

            int rejected = 0;

            for (int i = 0; i < settings.gridCount; ++i)
            {
                baseGrids.push_back(
                    GenerateReachableGrid(baseline, rejected)
                );
            }

            RecordRejections(
                "Crossover Grid Size " + std::to_string(gridSize),
                rejected
            );

            std::cout << std::left << std::setw(16)
                << std::to_string(gridSize) + " x " + std::to_string(gridSize);

            std::vector<double> gridRatioStdDevs;

            // Test each Wmax
            for (int maxCost : settings.crossoverMaxCosts)
            {
                SearchConfig config = baseline;
                config.maxCost = maxCost;

                GridAverages totals;

                for (const Grid& baseGrid : baseGrids)
                {
                    Grid grid = baseGrid;

                    // Preserve obstacles and regenerate weights
                    gridGenerator.ApplyWeights(grid, config);

                    Accumulate(
                        totals,
                        grid,
                        maxCost,
                        settings.repeatCount
                    );
                }

                std::cout << std::right << std::setw(14)
                    << totals.Ratio();
                gridRatioStdDevs.push_back(totals.RatioStdDev());
            }

            ratioStdDevRows.push_back(std::move(gridRatioStdDevs));
            std::cout << "\n";
        }

        std::cout << "\nSPEED RATIO STD DEV (across independent grid instances)\n";
        std::cout << std::left << std::setw(16) << "Grid Size / W";
        for (int maxCost : settings.crossoverMaxCosts)
            std::cout << std::right << std::setw(14) << maxCost;
        std::cout << "\n";
        const auto oldPrecision = std::cout.precision();
        std::cout << std::setprecision(6);
        for (size_t row = 0; row < ratioStdDevRows.size(); ++row)
        {
            int gridSize = settings.crossoverGridSizes[row];
            std::cout << std::left << std::setw(16)
                << std::to_string(gridSize) + " x " + std::to_string(gridSize);
            for (double stdDev : ratioStdDevRows[row])
                std::cout << std::right << std::setw(14) << stdDev;
            std::cout << "\n";
        }
        std::cout.precision(oldPrecision);
    }


};
class WeightScaleAndDistributionExperiment : public ExperimentBase
{
    WeightScaleAndDistributionExperimentSettings settings;
public:
    explicit WeightScaleAndDistributionExperiment(const WeightScaleAndDistributionExperimentSettings& s = WeightScaleAndDistributionExperimentSettings()) : settings(s) {}
    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);
        std::cout << std::fixed << std::setprecision(3);
        RunExperimentSet3WeightScale();
        RunExperimentSet3WeightDistribution();
        PrintRejectionSummary("EXPERIMENT SET 3");
    }
private:
    struct SensitivityTotals
    {
        // Dijkstra vs. Dial
        double dTime = 0, dialTime = 0;
        RunningStats dTimeSamples, dialTimeSamples, ratioSamples;
        double dProcessed = 0, dialProcessed = 0;
        double dRelaxations = 0, dialRelaxations = 0;
        double dTimePerRelaxation = 0, dialTimePerRelaxation = 0;
    };

    void RunExperimentSet3WeightScale()
    {
        PrintHeader("H3 - WEIGHT SCALE SENSITIVITY", true);
        SearchConfig baseline = MakeConfig(settings.gridWidth, settings.gridHeight,
            settings.occupiedRatio, settings.weightScaleMinCost, settings.weightScaleMaxCost);
        std::vector<SensitivityTotals> totals(settings.sensitivityWeightPairs.size());
        int rejected = 0;
        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            Grid baseGrid = GenerateReachableGrid(baseline, rejected);
            for (size_t i = 0; i < totals.size(); ++i)
            {
                SearchConfig config = baseline;
                config.minCost = settings.sensitivityWeightPairs[i].first;
                config.maxCost = settings.sensitivityWeightPairs[i].second;
                Grid grid = baseGrid;
                gridGenerator.ApplyWeights(grid, config);
                auto result = RunAlgorithms(grid, config.maxCost, settings.repeatCount);
                auto times = GetAverageTimes(result, settings.repeatCount);
                auto& t = totals[i];
                t.dTime += times.dijkstraNsExact;
                t.dialTime += times.dialNsExact;
                t.dTimeSamples.Add(times.dijkstraNsExact / 1000000.0);
                t.dialTimeSamples.Add(times.dialNsExact / 1000000.0);
                if (times.dijkstraNsExact > 0.0)
                    t.ratioSamples.Add(times.dialNsExact / times.dijkstraNsExact);
                t.dProcessed += result.dijkstraResult.processedCells;
                t.dialProcessed += result.dialResult.processedCells;
                t.dRelaxations += result.dijkstraResult.edgeRelaxations;
                t.dialRelaxations += result.dialResult.edgeRelaxations;
                if (result.dijkstraResult.edgeRelaxations > 0)
                    t.dTimePerRelaxation += times.dijkstraNsExact / result.dijkstraResult.edgeRelaxations;
                if (result.dialResult.edgeRelaxations > 0)
                    t.dialTimePerRelaxation += times.dialNsExact / result.dialResult.edgeRelaxations;
                ValidateMatchingCost(result.dijkstraResult, result.dialResult);
            }
        }
        RecordRejections("Weight Scale (shared base grids)", rejected);
        for (size_t i = 0; i < totals.size(); ++i)
        {
            const auto& t = totals[i];
            double n = settings.gridCount;
            double dMean = t.dTime / n;
            double dialMean = t.dialTime / n;
            PrintCaseHeader("Weight Scale: " + std::to_string(settings.sensitivityWeightPairs[i].first) +
                " - " + std::to_string(settings.sensitivityWeightPairs[i].second));
            std::cout << "Random Grids: " << settings.gridCount << "\n";
            std::cout << "Runs Per Grid: " << settings.repeatCount << "\n";
            std::cout << "\nDijkstra\n";
            std::cout << "Mean Execution Time: " << dMean / 1000000.0 << " ms\n";
            PrintStdDev("Execution Time Std Dev", t.dTimeSamples.StdDev(), " ms");
            std::cout << "Mean Processed Cells: " << t.dProcessed / n << "\n";
            std::cout << "Mean Edge Relaxations: " << t.dRelaxations / n << "\n";
            std::cout << "Mean Time / Relaxation: " << t.dTimePerRelaxation / n << " ns\n";
            std::cout << "\nDial's Algorithm\n";
            std::cout << "Mean Execution Time: " << dialMean / 1000000.0 << " ms\n";
            PrintStdDev("Execution Time Std Dev", t.dialTimeSamples.StdDev(), " ms");
            std::cout << "Mean Processed Cells: " << t.dialProcessed / n << "\n";
            std::cout << "Mean Edge Relaxations: " << t.dialRelaxations / n << "\n";
            std::cout << "Mean Time / Relaxation: " << t.dialTimePerRelaxation / n << " ns\n";
            std::cout << "\nSpeed Ratio: " << dialMean / dMean << "\n";
            PrintStdDev("Speed Ratio Std Dev", t.ratioSamples.StdDev());
        }
    }
    // Experiment Set 3: fixed weights vs mixed weight distribution.
    void RunExperimentSet3WeightDistribution()
    {
        SearchConfig config = MakeConfig(settings.gridWidth, settings.gridHeight,
            settings.occupiedRatio, settings.minCost, settings.maxCost);
        GridAverages a, b;
        int rejected = 0;
        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            Grid baseGrid = GenerateReachableGrid(config, rejected);
            Grid gridA = baseGrid;
            SetAllFreeWeights(gridA, settings.fixedWeight);
            Grid gridB = baseGrid;
            auto positions = GetFreePositions(gridB, true);
            SetAllFreeWeights(gridB, settings.lowWeight);
            ShufflePositions(positions);
            int highCount = static_cast<int>(positions.size() * settings.highWeightRatio);
            for (int i = 0; i < highCount; ++i)
            {
                Position p = positions[i];
                gridB.GetCell(p.x, p.y).cost = settings.highWeight;
            }
            Accumulate(a, gridA, settings.fixedWeight, settings.repeatCount);
            Accumulate(b, gridB, settings.highWeight, settings.repeatCount);
        }
        RecordRejections("Weight Distribution (shared base grids)", rejected);
        PrintDistributionCase("All Cells = " + std::to_string(settings.fixedWeight), a);
        PrintDistributionCase(std::to_string(static_cast<int>(settings.highWeightRatio * 100.0)) +
            "% Cells = " + std::to_string(settings.highWeight) + ", Rest = " +
            std::to_string(settings.lowWeight), b);
    }

    void PrintDistributionCase(const std::string& name, const GridAverages& t)
    {
        PrintCaseHeader(name);
        std::cout << "Dijkstra Time: " << t.DTime() << " ms\n";
        PrintStdDev("Dijkstra Time Std Dev", t.DTimeStdDev(), " ms");
        std::cout << "Dial Time: " << t.DialTime() << " ms\n";
        PrintStdDev("Dial Time Std Dev", t.DialTimeStdDev(), " ms");
        std::cout << "Dijkstra Processed Cells: " << t.DProcessed() << "\n";
        std::cout << "Dial Processed Cells: " << t.DialProcessed() << "\n";
        std::cout << "Dijkstra Edge Relaxations: " << t.DRelaxations() << "\n";
        std::cout << "Dial Edge Relaxations: " << t.DialRelaxations() << "\n";
        std::cout << "Dial Empty Buckets Scanned: " << t.EmptyScans() << "\n";
        std::cout << "Shortest Path Cost: " << t.Cost() << "\n";
        std::cout << "Speed Ratio: " << t.Ratio() << "\n";
        PrintStdDev("Speed Ratio Std Dev", t.RatioStdDev());
    }
};
class MeanAndMaximumEdgeWeightsExperiment : public ExperimentBase
{
    MeanAndMaximumEdgeWeightsExperimentSettings settings;
public:
    explicit MeanAndMaximumEdgeWeightsExperiment(const MeanAndMaximumEdgeWeightsExperimentSettings& s = MeanAndMaximumEdgeWeightsExperimentSettings()) : settings(s) {}
    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);
        std::cout << std::fixed << std::setprecision(3);
        RunExperimentSet4();
        PrintRejectionSummary("EXPERIMENT SET 4");
    }
private:
    void RunExperimentSet4()
    {
        SearchConfig config = MakeConfig(settings.gridWidth, settings.gridHeight,
            settings.occupiedRatio, settings.minCost, settings.maxCost);
        std::vector<GridAverages> normalTotals(settings.normalWeights.size());
        std::vector<GridAverages> maxTotals(settings.maxWeights.size());
        int rejected = 0;

        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            Grid baseGrid = GenerateReachableGrid(config, rejected);
            auto positions = GetFreePositions(baseGrid);
            ShufflePositions(positions);
            AccumulateSet4FixedMax(baseGrid, positions, normalTotals);
            AccumulateSet4FixedMean(baseGrid, positions, maxTotals);
        }
        RecordRejections("Set 4A/4B (shared base grids)", rejected);
        PrintExperimentSet4FixedMax(normalTotals);
        PrintExperimentSet4FixedMean(maxTotals);
    }

    // Set 4A: vary Wmean while Wmax is fixed.
    void AccumulateSet4FixedMax(const Grid& baseGrid, const std::vector<Position>& positions,
        std::vector<GridAverages>& totals)
    {
        for (size_t i = 0; i < settings.normalWeights.size(); ++i)
        {
            Grid grid = baseGrid;
            AssignFirstWeights(grid, positions, settings.normalWeights[i], settings.fixedMaxWeight,
                positions.empty() ? 0 : 1);
            Accumulate(totals[i], grid, settings.fixedMaxWeight, settings.repeatCount);
        }
    }

    // Set 4B: vary Wmax while Wmean is approximately fixed.
    void AccumulateSet4FixedMean(const Grid& baseGrid, const std::vector<Position>& positions,
        std::vector<GridAverages>& totals)
    {
        for (size_t i = 0; i < settings.maxWeights.size(); ++i)
        {
            int maxWeight = settings.maxWeights[i];
            Grid grid = baseGrid;
            int highCount = maxWeight == settings.targetMean
                ? static_cast<int>(positions.size())
                : static_cast<int>(positions.size() *
                    (static_cast<double>(settings.targetMean - 1) / (maxWeight - 1)));
            AssignFirstWeights(grid, positions, 1, maxWeight, highCount);
            Accumulate(totals[i], grid, maxWeight, settings.repeatCount);
        }
    }

    void PrintExperimentSet4FixedMax(const std::vector<GridAverages>& totals)
    {
        PrintHeader("FIXED WMAX = " + std::to_string(settings.fixedMaxWeight) + ", VARY WMEAN");
        for (size_t i = 0; i < totals.size(); ++i)
            PrintMeanMaxCase(totals[i], settings.fixedMaxWeight, settings.fixedMaxWeight);
    }

    void PrintExperimentSet4FixedMean(const std::vector<GridAverages>& totals)
    {
        PrintHeader("FIXED WMEAN ~= " + std::to_string(settings.targetMean) + ", VARY WMAX");
        for (size_t i = 0; i < totals.size(); ++i)
            PrintMeanMaxCase(totals[i], settings.maxWeights[i], settings.maxWeights[i]);
    }

    void PrintMeanMaxCase(const GridAverages& t, int maxWeight, int actualMaxWeight)
    {
        std::cout << "\n";
        std::cout << "W Mean: " << t.Mean() << "\n";
        std::cout << "W Max: " << actualMaxWeight << "\n";
        std::cout << "Bucket Count: " << (maxWeight + 1) << "\n";
        std::cout << "Dijkstra Time: " << t.dTime / t.count / 1000000.0 << " ms\n";
        PrintStdDev("Dijkstra Time Std Dev", t.DTimeStdDev(), " ms");
        std::cout << "Dijkstra Peak Memory: " << t.DMemory() << " KB\n";
        std::cout << "Dijkstra Processed / Relaxations: " << t.DProcessed() << " / " << t.DRelaxations() << "\n";
        std::cout << "Dial Time: " << t.dialTime / t.count / 1000000.0 << " ms\n";
        PrintStdDev("Dial Time Std Dev", t.DialTimeStdDev(), " ms");
        std::cout << "Dial Peak Memory: " << t.DialMemory() << " KB\n";
        std::cout << "Dial Processed / Relaxations: " << t.DialProcessed() << " / " << t.DialRelaxations() << "\n";
        std::cout << "Cost: " << t.Cost() << "\n";
        std::cout << "Empty Buckets Scanned: " << t.EmptyScans() << "\n";
    }
};
#pragma endregion

#pragma region Additional Expriments
// Additional Experiment 1 - Impact of Sample Size (Section 6.1)
// Examine whether larger sample sizes reduce unexpected performance variations
struct RandomWeightGridSizeSettings : ExperimentSettingsBase
{
    // Manually change the grid count.
    int gridCount = 100;

    std::vector<int> gridSizes = { 50, 100, 200, 400 };
    std::vector<int> maxWeights = { 1, 100, 1000 };

    int occupiedRatio = 20;
};

// Additional Experiment 2 - Effect of Weight Distribution (Section 6.2)
// Examine how high-weight proportions affect speed ratios across grid sizes
// while maintaining Wmean at approximately 500
struct FixedMeanDistributionSettings : ExperimentSettingsBase
{
    std::vector<int> gridSizes = { 5, 25, 100, 250, 500, 750, 1000 };

    std::vector<double> highWeightRatios =
    {
        0.05,
        0.10,
        0.25,
        0.50,
        0.75,
        1.00
    };

    int targetMean = 500;
    int lowWeight = 1;
    int occupiedRatio = 20;
};

// Additional Experiment 3 - Bucket Scanning (Section 6.2)
// Examine Dial's execution time, processed vertices, and empty bucket scans
// under different weight distributions with fixed grid size and Wmean
struct BucketDistributionSettings : ExperimentSettingsBase
{
    int gridWidth = 500;
    int gridHeight = 500;
    int occupiedRatio = 20;

    int targetMean = 500;
    int lowWeight = 1;

    std::vector<double> highWeightRatios = { 0.10, 0.50, 1.00 };
};

class RandomWeightGridSizeExperiment : public ExperimentBase
{
private:
    RandomWeightGridSizeSettings settings;

public:
    explicit RandomWeightGridSizeExperiment(const RandomWeightGridSizeSettings& s = RandomWeightGridSizeSettings()) : settings(s) {}
    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);

        std::cout << std::fixed << std::setprecision(3);

        PrintHeader("RANDOM WEIGHT - GRID SIZE EXPERIMENT", true);

        for (int size : settings.gridSizes)
        {
            RunGridSize(size);
        }

        PrintRejectionSummary("RANDOM WEIGHT GRID SIZE EXPERIMENT");
    }

private:
    void RunGridSize(int size)
    {
        SearchConfig baseConfig = MakeConfig(
            size,
            size,
            settings.occupiedRatio,
            1,
            1
        );

        std::vector<GridAverages> totals(settings.maxWeights.size());

        int rejected = 0;

        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            // Generate one grid layout shared by all weight configurations.
            Grid baseGrid = GenerateReachableGrid(baseConfig, rejected);

            for (size_t i = 0; i < settings.maxWeights.size(); ++i)
            {
                int maxWeight = settings.maxWeights[i];

                SearchConfig config = baseConfig;

                config.weighted = true;
                config.minCost = 1;
                config.maxCost = maxWeight;

                Grid grid = baseGrid;

                // Preserve obstacles and regenerate weights.
                gridGenerator.ApplyWeights(grid, config);

                Accumulate(
                    totals[i],
                    grid,
                    maxWeight,
                    settings.repeatCount
                );
            }
        }

        RecordRejections(
            "Grid Size " + std::to_string(size),
            rejected
        );

        PrintCaseHeader(
            "Grid Size: " + std::to_string(size) +
            " x " + std::to_string(size)
        );

        std::cout << "Obstacle Ratio: " << settings.occupiedRatio << "%\n";
        std::cout << "Random Grids: " << settings.gridCount << "\n";
        std::cout << "Runs Per Grid: " << settings.repeatCount << "\n";

        for (size_t i = 0; i < settings.maxWeights.size(); ++i)
        {
            const GridAverages& t = totals[i];

            std::cout << "\n----------------------------------------\n";
            std::cout << "Wmin: 1, Wmax: " << settings.maxWeights[i] << "\n";
            std::cout << "----------------------------------------\n";

            std::cout << "V: " << t.V() << "\n";
            std::cout << "E: " << t.E() << "\n";
            std::cout << "W Mean: " << t.Mean() << "\n";

            std::cout << "\nDijkstra\n";
            std::cout << "Execution Time: " << t.DTime() << " ms\n";
            PrintStdDev("Execution Time Std Dev", t.DTimeStdDev(), " ms");
            std::cout << "Processed Cells: " << t.DProcessed() << "\n";
            std::cout << "Edge Relaxations: " << t.DRelaxations() << "\n";

            std::cout << "\nDial's Algorithm\n";
            std::cout << "Execution Time: " << t.DialTime() << " ms\n";
            PrintStdDev("Execution Time Std Dev", t.DialTimeStdDev(), " ms");
            std::cout << "Processed Cells: " << t.DialProcessed() << "\n";
            std::cout << "Edge Relaxations: " << t.DialRelaxations() << "\n";
            std::cout << "Empty Buckets Scanned: " << t.EmptyScans() << "\n";

            std::cout << "\nShortest Path Cost: " << t.Cost() << "\n";
            std::cout << "Speed Ratio: " << t.Ratio() << "x\n";
            PrintStdDev("Speed Ratio Std Dev", t.RatioStdDev());
        }
    }
};
class FixedMeanDistributionExperiment : public ExperimentBase
{
private:
    FixedMeanDistributionSettings settings;

public:
    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);

        std::cout << std::fixed << std::setprecision(3);

        PrintHeader(
            "FIXED MEAN - WEIGHT DISTRIBUTION BY GRID SIZE",
            true
        );

        for (int size : settings.gridSizes)
            RunGridSize(size);

        PrintRejectionSummary("FIXED MEAN DISTRIBUTION EXPERIMENT");
    }

private:
    void ApplyDistribution(
        Grid& grid,
        const std::vector<Position>& positions,
        double highRatio)
    {
        const int count = static_cast<int>(positions.size());

        if (count == 0)
            return;

        // First assign low weights to every free cell.
        SetAllFreeWeights(grid, settings.lowWeight);

        // Special case: all weights are fixed at the target mean.
        if (highRatio >= 1.0)
        {
            SetAllFreeWeights(grid, settings.targetMean);
            return;
        }

        // Include the start cell in the mean calculation.
        const int totalFreeCells = count;

        int highCount = static_cast<int>(
            std::round(totalFreeCells * highRatio)
            );

        highCount = std::clamp(highCount, 1, totalFreeCells);

        // Total additional weight needed to reach the target mean.
        long long additionalWeight =
            static_cast<long long>(totalFreeCells) *
            (settings.targetMean - settings.lowWeight);

        long long baseIncrease = additionalWeight / highCount;
        long long remainder = additionalWeight % highCount;

        // Distribute the required weight among selected high-weight cells.
        for (int i = 0; i < highCount; ++i)
        {
            long long weight =
                settings.lowWeight +
                baseIncrease +
                (i < remainder ? 1 : 0);

            if (weight > std::numeric_limits<int>::max())
                throw std::overflow_error("Weight exceeds integer range.");

            Position p = positions[i];

            grid.GetCell(p.x, p.y).cost = static_cast<int>(weight);
        }
    }

    void RunGridSize(int size)
    {
        SearchConfig config = MakeConfig(
            size,
            size,
            settings.occupiedRatio,
            1,
            1
        );

        std::vector<GridAverages> distributionTotals(
            settings.highWeightRatios.size()
        );

        int rejected = 0;

        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            Grid baseGrid = GenerateReachableGrid(config, rejected);

            // Shared shuffled positions across every distribution.
            auto positions = GetFreePositions(baseGrid);

            ShufflePositions(positions);

            for (size_t i = 0; i < settings.highWeightRatios.size(); ++i)
            {
                Grid grid = baseGrid;

                ApplyDistribution(
                    grid,
                    positions,
                    settings.highWeightRatios[i]
                );

                // Use the actual maximum weight for Dial.
                GridStats stats = AnalyzeGrid(grid);

                Accumulate(
                    distributionTotals[i],
                    grid,
                    stats.maxWeight,
                    settings.repeatCount
                );
            }
        }

        RecordRejections(
            "Grid Size " + std::to_string(size),
            rejected
        );

        PrintCaseHeader(
            "Grid Size: " + std::to_string(size) +
            " x " + std::to_string(size)
        );

        std::cout << "Obstacle Ratio: " << settings.occupiedRatio << "%\n";
        std::cout << "Random Grids: " << settings.gridCount << "\n";
        std::cout << "Runs Per Grid: " << settings.repeatCount << "\n";
        std::cout << "Target Mean Weight: " << settings.targetMean << "\n";

        for (size_t i = 0; i < settings.highWeightRatios.size(); ++i)
        {
            const GridAverages& t = distributionTotals[i];

            std::cout << "\nHigh Weight Ratio: "
                << settings.highWeightRatios[i] * 100.0
                << "%\n";

            std::cout << "W Mean: " << t.Mean() << "\n";
            std::cout << "Dijkstra Time: " << t.DTime() << " ms\n";
            PrintStdDev("Dijkstra Time Std Dev", t.DTimeStdDev(), " ms");
            std::cout << "Dial Time: " << t.DialTime() << " ms\n";
            PrintStdDev("Dial Time Std Dev", t.DialTimeStdDev(), " ms");
            std::cout << "Empty Buckets Scanned: " << t.EmptyScans() << "\n";
            std::cout << "Speed Ratio: " << t.Ratio() << "\n";
            PrintStdDev("Speed Ratio Std Dev", t.RatioStdDev());
        }

        // One-line comparison for Report Table 23.
        std::cout << "\nSPEED RATIO COMPARISON\n";

        std::cout << "Grid Size";

        for (double ratio : settings.highWeightRatios)
            std::cout << "\t" << ratio * 100.0 << "%";

        std::cout << "\n" << size << " x " << size;

        for (const auto& t : distributionTotals)
            std::cout << "\t" << t.Ratio();

        std::cout << "\nSPEED RATIO STD DEV COMPARISON\n";
        std::cout << "Grid Size";
        for (double ratio : settings.highWeightRatios)
            std::cout << "\t" << ratio * 100.0 << "%";
        std::cout << "\n" << size << " x " << size;
        const auto oldPrecision = std::cout.precision();
        std::cout << std::setprecision(6);
        for (const auto& t : distributionTotals)
            std::cout << "\t" << t.RatioStdDev();
        std::cout.precision(oldPrecision);
        std::cout << "\n";
    }
};
class BucketDistributionExperiment : public ExperimentBase
{
private:
    BucketDistributionSettings settings;

public:
    explicit BucketDistributionExperiment(
        const BucketDistributionSettings& s = BucketDistributionSettings())
        : settings(s) {
    }

    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);
        std::cout << std::fixed << std::setprecision(3);

        PrintHeader("WEIGHT DISTRIBUTION - BUCKET SCANNING EXPERIMENT", true);

        std::cout << "Grid Size: "
            << settings.gridWidth << " x " << settings.gridHeight << "\n";
        std::cout << "Obstacle Ratio: " << settings.occupiedRatio << "%\n";
        std::cout << "Target Wmean: " << settings.targetMean << "\n";
        std::cout << "Random Grids: " << settings.gridCount << "\n";
        std::cout << "Runs Per Grid: " << settings.repeatCount << "\n";

        RunComparison();

        PrintRejectionSummary("BUCKET DISTRIBUTION EXPERIMENT");
    }

private:
    void ApplyDistribution(
        Grid& grid,
        const std::vector<Position>& positions,
        double highRatio)
    {
        const int count = static_cast<int>(positions.size());

        if (count == 0)
            return;

        SetAllFreeWeights(grid, settings.lowWeight);

        if (highRatio >= 1.0)
        {
            SetAllFreeWeights(grid, settings.targetMean);
            return;
        }

        int highCount = static_cast<int>(
            std::round(count * highRatio)
            );

        highCount = std::clamp(highCount, 1, count);

        long long additionalWeight =
            static_cast<long long>(count) *
            (settings.targetMean - settings.lowWeight);

        long long baseIncrease = additionalWeight / highCount;
        long long remainder = additionalWeight % highCount;

        for (int i = 0; i < highCount; ++i)
        {
            long long weight =
                settings.lowWeight +
                baseIncrease +
                (i < remainder ? 1 : 0);

            if (weight > std::numeric_limits<int>::max())
                throw std::overflow_error("Weight exceeds integer range.");

            Position p = positions[i];

            grid.GetCell(p.x, p.y).cost = static_cast<int>(weight);
        }
    }

    void RunComparison()
    {
        SearchConfig config = MakeConfig(
            settings.gridWidth,
            settings.gridHeight,
            settings.occupiedRatio,
            1,
            1
        );

        std::vector<GridAverages> totals(
            settings.highWeightRatios.size()
        );

        std::vector<double> maxWeightTotals(
            settings.highWeightRatios.size(), 0.0
        );

        int rejected = 0;

        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            // Generate one shared grid layout.
            Grid baseGrid = GenerateReachableGrid(config, rejected);

            // Use the same shuffled positions for every distribution.
            std::vector<Position> positions = GetFreePositions(baseGrid);

            ShufflePositions(positions);

            for (size_t i = 0; i < settings.highWeightRatios.size(); ++i)
            {
                Grid grid = baseGrid;

                ApplyDistribution(
                    grid,
                    positions,
                    settings.highWeightRatios[i]
                );

                GridStats stats = AnalyzeGrid(grid);

                maxWeightTotals[i] += stats.maxWeight;

                Accumulate(
                    totals[i],
                    grid,
                    stats.maxWeight,
                    settings.repeatCount
                );
            }
        }

        RecordRejections("Shared Base Grids", rejected);

        PrintResults(totals, maxWeightTotals);
    }

    void PrintResults(
        const std::vector<GridAverages>& totals,
        const std::vector<double>& maxWeightTotals)
    {
        PrintCaseHeader("WEIGHT DISTRIBUTION COMPARISON");

        std::cout << std::left << std::setw(28) << "Metric";

        for (double ratio : settings.highWeightRatios)
        {
            std::cout << std::setw(18)
                << std::to_string(static_cast<int>(std::round(ratio * 100))) + "%";
        }

        std::cout << "\n";

        PrintRow(totals, "Wmean",
            [](const GridAverages& t) { return t.Mean(); });

        std::cout << std::left << std::setw(28) << "Wmax";

        for (size_t i = 0; i < maxWeightTotals.size(); ++i)
        {
            std::cout << std::setw(18)
                << maxWeightTotals[i] / settings.gridCount;
        }

        std::cout << "\n\n";

        PrintRow(totals, "Dijkstra Time (ms)",
            [](const GridAverages& t) { return t.DTime(); });
        PrintStdDevRow(totals, "Dijkstra Std Dev (ms)",
            [](const GridAverages& t) { return t.DTimeStdDev(); });

        PrintRow(totals, "Dial Time (ms)",
            [](const GridAverages& t) { return t.DialTime(); });
        PrintStdDevRow(totals, "Dial Std Dev (ms)",
            [](const GridAverages& t) { return t.DialTimeStdDev(); });

        std::cout << "\n";

        PrintRow(totals, "Dijkstra Processed",
            [](const GridAverages& t) { return t.DProcessed(); });

        PrintRow(totals, "Dial Processed",
            [](const GridAverages& t) { return t.DialProcessed(); });

        PrintRow(totals, "Dijkstra Relaxations",
            [](const GridAverages& t) { return t.DRelaxations(); });

        PrintRow(totals, "Dial Relaxations",
            [](const GridAverages& t) { return t.DialRelaxations(); });

        std::cout << "\n";

        PrintRow(totals, "Empty Buckets Scanned",
            [](const GridAverages& t) { return t.EmptyScans(); });

        PrintRow(totals, "Shortest Path Cost",
            [](const GridAverages& t) { return t.Cost(); });

        PrintRow(totals, "Speed Ratio",
            [](const GridAverages& t) { return t.Ratio(); });
        PrintStdDevRow(totals, "Speed Ratio Std Dev",
            [](const GridAverages& t) { return t.RatioStdDev(); });
    }

    template<typename Func>
    void PrintStdDevRow(
        const std::vector<GridAverages>& totals,
        const std::string& name,
        Func getter)
    {
        const auto oldPrecision = std::cout.precision();
        std::cout << std::setprecision(6);
        PrintRow(totals, name, getter);
        std::cout.precision(oldPrecision);
    }

    template<typename Func>
    void PrintRow(
        const std::vector<GridAverages>& totals,
        const std::string& name,
        Func getter)
    {
        std::cout << std::left << std::setw(28) << name;

        for (const GridAverages& t : totals)
        {
            std::cout << std::setw(18) << getter(t);
        }

        std::cout << "\n";
    }
};
#pragma endregion


#pragma region Lagacy Experiments
struct Set5ExperimentSettings : ExperimentSettingsBase
{
    int gridWidth = 100;
    int gridHeight = 100;
    int occupiedRatio = 20;

    int minCost = 1;
    int maxCost = 100;

    std::vector<int> gridSizes = { 50, 100, 200, 400 };
    std::vector<int> maxWeights = { 1, 2, 5, 10, 100, 1000, 10000 };
};
struct UnifiedCorrelationSettings : ExperimentSettingsBase
{
    int caseCount = 60;

    std::vector<int> gridSizes = { 50, 75, 100, 150, 200 };
    std::vector<int> obstacleRatios = { 0, 10, 20, 30 };
    std::vector<int> maxWeights = { 10, 50, 100, 500, 1000, 5000, 10000 };
};
struct FixedWeightGridSizeSettings : ExperimentSettingsBase
{
    int gridCount = 10;

    std::vector<int> gridSizes = { 5, 25, 100, 250, 500, 750, 1000 };

    std::vector<int> fixedWeights = { 1, 10, 100, 1000 };

    std::vector<int> randomMeanWeights = { 10, 100, 1000 };

    int occupiedRatio = 20;
};

class Set5Experiment : public ExperimentBase
{
    Set5ExperimentSettings settings;

public:
    explicit Set5Experiment(const Set5ExperimentSettings& s = Set5ExperimentSettings()) : settings(s) {}

    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);
        std::cout << std::fixed << std::setprecision(3);

        RunGridSizeMemoryExperiment();
        RunMaxWeightMemoryExperiment();

        PrintRejectionSummary("EXPERIMENT SET 5");
    }

private:
    struct MemoryTotals
    {
        double dijkstraMemory = 0;
        double dialMemory = 0;
        int count = 0;

        void Add(const AlgorithmExecutionResult& result)
        {
            dijkstraMemory += result.dijkstraResult.peakMemoryBytes;
            dialMemory += result.dialResult.peakMemoryBytes;
            ++count;
        }

        double DijkstraMemory() const { return dijkstraMemory / count / 1024.0; }
        double DialMemory() const { return dialMemory / count / 1024.0; }
    };

    void RunGridSizeMemoryExperiment()
    {
        PrintHeader("SET 5A - FIXED WMAX, VARY GRID SIZE", true);

        std::cout << "Wmin: " << settings.minCost << "\n";
        std::cout << "Wmax: " << settings.maxCost << "\n";
        std::cout << "Obstacle Ratio: " << settings.occupiedRatio << "%\n";
        std::cout << "Random Grids: " << settings.gridCount << "\n";
        std::cout << "Runs Per Grid: " << settings.repeatCount << "\n";

        std::cout << "\nGrid Size\tDijkstra Memory (KB)\tDial Memory (KB)\n";

        for (int size : settings.gridSizes)
        {
            SearchConfig config = MakeConfig(
                size, size, settings.occupiedRatio,
                settings.minCost, settings.maxCost
            );

            MemoryTotals totals;
            int rejected = 0;

            for (int i = 0; i < settings.gridCount; ++i)
            {
                Grid grid = GenerateReachableGrid(config, rejected);
                AlgorithmExecutionResult result = RunAlgorithms(
                    grid, config.maxCost, settings.repeatCount
                );

                ValidateMatchingCost(result.dijkstraResult, result.dialResult);
                totals.Add(result);
            }

            RecordRejections("Set 5A - Grid Size " + std::to_string(size), rejected);

            std::cout << size << " x " << size << "\t"
                << totals.DijkstraMemory() << "\t"
                << totals.DialMemory() << "\n";
        }
    }

    void RunMaxWeightMemoryExperiment()
    {
        PrintHeader("SET 5B - FIXED GRID SIZE, VARY WMAX", true);

        std::cout << "Grid Size: " << settings.gridWidth << " x " << settings.gridHeight << "\n";
        std::cout << "Obstacle Ratio: " << settings.occupiedRatio << "%\n";
        std::cout << "Wmin: " << settings.minCost << "\n";
        std::cout << "Random Grids: " << settings.gridCount << "\n";
        std::cout << "Runs Per Grid: " << settings.repeatCount << "\n";

        std::cout << "\nWmax\tDijkstra Memory (KB)\tDial Memory (KB)\n";

        SearchConfig baseConfig = MakeConfig(
            settings.gridWidth, settings.gridHeight,
            settings.occupiedRatio, settings.minCost, settings.maxCost
        );

        std::vector<MemoryTotals> totals(settings.maxWeights.size());
        int rejected = 0;

        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            Grid baseGrid = GenerateReachableGrid(baseConfig, rejected);

            for (size_t i = 0; i < settings.maxWeights.size(); ++i)
            {
                SearchConfig config = baseConfig;
                config.maxCost = settings.maxWeights[i];

                Grid grid = baseGrid;
                gridGenerator.ApplyWeights(grid, config);

                AlgorithmExecutionResult result = RunAlgorithms(
                    grid, config.maxCost, settings.repeatCount
                );

                ValidateMatchingCost(result.dijkstraResult, result.dialResult);
                totals[i].Add(result);
            }
        }

        RecordRejections("Set 5B - Shared Base Grids", rejected);

        for (size_t i = 0; i < settings.maxWeights.size(); ++i)
        {
            std::cout << settings.maxWeights[i] << "\t"
                << totals[i].DijkstraMemory() << "\t"
                << totals[i].DialMemory() << "\n";
        }
    }
};
class UnifiedCorrelationExperiment : public ExperimentBase
{
    UnifiedCorrelationSettings settings;

public:
    explicit UnifiedCorrelationExperiment(
        const UnifiedCorrelationSettings& s = UnifiedCorrelationSettings())
        : settings(s) {
    }

    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);
        std::cout << std::fixed << std::setprecision(3);
        RunUnifiedCorrelation();
        PrintRejectionSummary("UNIFIED CORRELATION");
    }
private:
    void RunUnifiedCorrelation()
    {
        PrintHeader("UNIFIED CORRELATION EXPERIMENT", true);

        std::cout << "\nCase\tGridSize\tObstacle\tV\tE\tWMin\tWMax\tWMean\tCost\tProcessed\tRelaxations\tEmptyScans\tDialTimeMs\n";

        int rejected = 0;
        for (int caseIndex = 0; caseIndex < settings.caseCount; ++caseIndex)
        {
            int gridSize = settings.gridSizes[
                gridGenerator.GetRandomInt(0, static_cast<int>(settings.gridSizes.size()) - 1)
            ];

            int obstacleRatio = settings.obstacleRatios[
                gridGenerator.GetRandomInt(0, static_cast<int>(settings.obstacleRatios.size()) - 1)
            ];

            int maxCost = settings.maxWeights[
                gridGenerator.GetRandomInt(0, static_cast<int>(settings.maxWeights.size()) - 1)
            ];

            int minMode = gridGenerator.GetRandomInt(0, 3);
            int minCost;

            switch (minMode)
            {
            case 0: minCost = 1; break;
            case 1: minCost = std::max(1, maxCost / 10); break;
            case 2: minCost = std::max(1, maxCost / 2); break;
            default: minCost = maxCost; break;
            }

            SearchConfig config = MakeConfig(
                gridSize, gridSize, obstacleRatio, minCost, maxCost
            );

            Grid grid = GenerateReachableGrid(config, rejected);

            GridStats stats = AnalyzeGrid(grid);

            auto result = RunAlgorithms(
                grid, stats.maxWeight, settings.repeatCount, false
            );

            auto times = GetAverageTimes(result, settings.repeatCount);

            std::cout << (caseIndex + 1) << "\t" << gridSize << "\t" << obstacleRatio
                << "\t" << stats.vertices << "\t" << stats.edges
                << "\t" << stats.minWeight << "\t" << stats.maxWeight
                << "\t" << stats.mean << "\t" << result.dialResult.shortestPathCost
                << "\t" << result.dialResult.processedCells
                << "\t" << result.dialResult.edgeRelaxations
                << "\t" << result.dialResult.emptyBucketsScanned
                << "\t" << times.dialNsExact / 1000000.0 << "\n";
        }
        RecordRejections("Correlation cases", rejected);
    }
};
class FixedWeightGridSizeExperiment : public ExperimentBase
{
private:
    FixedWeightGridSizeSettings settings;

public:
    void Run()
    {
        gridGenerator.SetSeed(settings.randomSeed);

        std::cout << std::fixed << std::setprecision(3);

        PrintHeader("FIXED VS RANDOM WEIGHT - GRID SIZE EXPERIMENT", true);

        for (int size : settings.gridSizes)
        {
            RunGridSize(size);
        }

        PrintRejectionSummary("FIXED VS RANDOM WEIGHT GRID SIZE EXPERIMENT");
    }

private:
    void RunGridSize(int size)
    {
        SearchConfig baseConfig = MakeConfig(
            size,
            size,
            settings.occupiedRatio,
            1,
            1
        );

        std::vector<GridAverages> fixedTotals(settings.fixedWeights.size());
        std::vector<GridAverages> randomTotals(settings.randomMeanWeights.size());

        int rejected = 0;

        for (int gridIndex = 0; gridIndex < settings.gridCount; ++gridIndex)
        {
            // Generate one grid layout shared by all weight configurations.
            Grid baseGrid = GenerateReachableGrid(baseConfig, rejected);

            // Fixed weight experiments.
            for (size_t i = 0; i < settings.fixedWeights.size(); ++i)
            {
                int weight = settings.fixedWeights[i];

                Grid grid = baseGrid;

                SetAllFreeWeights(grid, weight);

                Accumulate(
                    fixedTotals[i],
                    grid,
                    weight,
                    settings.repeatCount
                );
            }

            // Random weight experiments.
            for (size_t i = 0; i < settings.randomMeanWeights.size(); ++i)
            {
                int targetMean = settings.randomMeanWeights[i];

                int minWeight = 1;
                int maxWeight = targetMean * 2 - 1;

                SearchConfig randomConfig = baseConfig;

                randomConfig.weighted = true;
                randomConfig.minCost = minWeight;
                randomConfig.maxCost = maxWeight;

                Grid grid = baseGrid;

                // Preserve grid layout and regenerate only cell weights.
                gridGenerator.ApplyWeights(grid, randomConfig);

                Accumulate(
                    randomTotals[i],
                    grid,
                    maxWeight,
                    settings.repeatCount
                );
            }
        }

        RecordRejections(
            "Grid Size " + std::to_string(size),
            rejected
        );

        PrintCaseHeader(
            "Grid Size: " + std::to_string(size) +
            " x " + std::to_string(size)
        );

        std::cout << "Obstacle Ratio: " << settings.occupiedRatio << "%\n";
        std::cout << "Random Grids: " << settings.gridCount << "\n";
        std::cout << "Runs Per Grid: " << settings.repeatCount << "\n";

        // Fixed Weight Results
        PrintCaseHeader("FIXED WEIGHT RESULTS");

        for (size_t i = 0; i < settings.fixedWeights.size(); ++i)
        {
            PrintResult(
                "Fixed Weight: " + std::to_string(settings.fixedWeights[i]),
                fixedTotals[i]
            );
        }

        // Random Weight Results
        PrintCaseHeader("RANDOM WEIGHT RESULTS");

        for (size_t i = 0; i < settings.randomMeanWeights.size(); ++i)
        {
            int targetMean = settings.randomMeanWeights[i];

            PrintResult(
                "Random Weight: [1, " +
                std::to_string(targetMean * 2 - 1) +
                "], Target Mean: " +
                std::to_string(targetMean),
                randomTotals[i]
            );
        }

        // Direct Comparison
        PrintComparison(fixedTotals, randomTotals);
    }

    void PrintResult(const std::string& name, const GridAverages& t)
    {
        std::cout << "\n----------------------------------------\n";
        std::cout << name << "\n";
        std::cout << "----------------------------------------\n";

        std::cout << "V: " << t.V() << "\n";
        std::cout << "E: " << t.E() << "\n";
        std::cout << "W Mean: " << t.Mean() << "\n";

        std::cout << "\nDijkstra\n";
        std::cout << "Execution Time: " << t.DTime() << " ms\n";
        std::cout << "Processed Cells: " << t.DProcessed() << "\n";
        std::cout << "Edge Relaxations: " << t.DRelaxations() << "\n";

        std::cout << "\nDial's Algorithm\n";
        std::cout << "Execution Time: " << t.DialTime() << " ms\n";
        std::cout << "Processed Cells: " << t.DialProcessed() << "\n";
        std::cout << "Edge Relaxations: " << t.DialRelaxations() << "\n";
        std::cout << "Empty Buckets Scanned: " << t.EmptyScans() << "\n";

        std::cout << "\nShortest Path Cost: " << t.Cost() << "\n";
        std::cout << "Speed Ratio: " << t.Ratio() << "x\n";
    }

    void PrintComparison(
        const std::vector<GridAverages>& fixedTotals,
        const std::vector<GridAverages>& randomTotals)
    {
        PrintCaseHeader("FIXED VS RANDOM - SPEED RATIO COMPARISON");

        std::cout << std::left
            << std::setw(16) << "Target Mean"
            << std::setw(20) << "Fixed Ratio"
            << std::setw(20) << "Random Ratio"
            << "\n";

        for (size_t i = 0; i < settings.randomMeanWeights.size(); ++i)
        {
            int targetMean = settings.randomMeanWeights[i];

            auto it = std::find(
                settings.fixedWeights.begin(),
                settings.fixedWeights.end(),
                targetMean
            );

            if (it == settings.fixedWeights.end())
                continue;

            size_t fixedIndex = static_cast<size_t>(
                std::distance(settings.fixedWeights.begin(), it)
                );

            std::cout << std::left
                << std::setw(16) << targetMean
                << std::setw(20) << fixedTotals[fixedIndex].Ratio()
                << std::setw(20) << randomTotals[i].Ratio()
                << "\n";
        }
    }
};
#pragma endregion