#include "Grid.cpp"
#include "Algorithms/Dijkstra.cpp"
#include "Algorithms/Dial.cpp"
#include "Benchmark/Experiments.cpp"

int main(int argc, char* argv[])
{
    // Main Experiments
    GeneralExperiment generalExperiment;
    WeightScaleAndDistributionExperiment weightScaleAndDistributionExperiment;
    MeanAndMaximumEdgeWeightsExperiment meanAndMaximumEdgeWeightsExperiment;

    // Additional Experiments
    FixedMeanDistributionExperiment weightDistributionGridSizeExperiment;
    BucketDistributionExperiment weightDistributionBucketScanningExperiment;
    RandomWeightGridSizeExperiment sampleSizeImpactExperiment;  // If you run the code through Visual Studio Solution,
                                                                // you need to manually change the grid count
                                                                // in the RandomWeightGridSizeSettings.

    //generalExperiment.Run();
    //weightScaleAndDistributionExperiment.Run();
    //meanAndMaximumEdgeWeightsExperiment.Run();
    //sampleSizeImpactExperiment.Run();
    //weightDistributionGridSizeExperiment.Run();
    //weightDistributionBucketScanningExperiment.Run();

    if (argc > 1)
    {
        std::string experiment = argv[1];

        if (experiment == "general")
            generalExperiment.Run();
        else if (experiment == "weight-scale")
            weightScaleAndDistributionExperiment.Run();
        else if (experiment == "mean-max")
            meanAndMaximumEdgeWeightsExperiment.Run();
        else if (experiment == "sample-size")
        {
            if (argc > 2)
            {
                RandomWeightGridSizeSettings settings;
                settings.gridCount = std::stoi(argv[2]);
                if (settings.gridCount <= 0)
                {
                    std::cerr << "Grid count must be positive.\n";
                    return 1;
                }
                RandomWeightGridSizeExperiment(settings).Run();
            }
            else
                sampleSizeImpactExperiment.Run();
        }
        else if (experiment == "fixed-mean")
            weightDistributionGridSizeExperiment.Run();
        else if (experiment == "bucket")
            weightDistributionBucketScanningExperiment.Run();
        else
        {
            std::cerr << "Unknown experiment: " << experiment << "\n";
            return 1;
        }

        return 0;
    }

    return 0;
}
