# Shortest Path Algorithms Comparison

**41052 Programming Assignment 2**  
*Theoretical Optimal vs. Practical in Shortest-Path Finding Problem (Binary-heap Dijkstra vs. Dial)* 

**Authors**
- Reagan Song (14740676)
- Yoonji Byun (14740664)


## Original Experiment Results

Contains the experimental results used in the report.

* **Complete Raw Experimental Tables:** All raw results organized into tables (with ChatGPT assistance).
* **Standard Deviation Tables:** All standard deviations organized into tables (with ChatGPT assistance).

## Algorithms & Benchmark

* `Algorithms/Dijkstra.cpp`: Binary-heap Dijkstra's algorithm. (Yoonji Byun)
* `Algorithms/Dial.cpp`: Dial's algorithm. (Reagan Song)
* `Benchmark/Experiments.cpp`: Benchmarking framework and experiments. (Reagan Song)
* `Grid.cpp`: Shared grid structures, generation, and rendering. (Reagan Song)
* `A2SearchAlgorithmsComparison.cpp`: Main entry point.

## How to Run

Run `run_experiments.bat` and select an experiment.

1. General: Main Experiments 1
2. Weight Scale and Distribution: Main Experiments 2 
3. Mean and Maximum Edge Weights: Main Experiments 3
4. Sample Size (50 grids): Additional Experiment 1
5. Sample Size (100 grids): Additional Experiment 1
6. Fixed Mean Distribution: Additional Experiment 2
7. Bucket Scanning: Additional Experiment 3

Results are saved in the `results/` folder.

All experiments use the same fixed random seed as the original experiments to reproduce the same input configurations.
