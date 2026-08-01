// =============================================================================
// Corium SimLab — Real-Time Simulation Engine & WebGPU Visualizer
//
// Standalone Application Entry Point
// =============================================================================

#include "corium_sim/App.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================\n";
    std::cout << " Corium SimLab Engine v1.0.0 (WebGPU + Zero-Heap MPSC)\n";
    std::cout << "=========================================================\n\n";

    corium_sim::SimRuntime runtime;
    corium_sim::SimLabApp app;

    runtime.initialize(app);
    app.run(runtime);
    runtime.shutdown();

    std::cout << "\nCorium SimLab exited successfully.\n";
    return 0;
}
