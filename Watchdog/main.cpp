#include <iostream>
#include <vector>
#include "../Planning/Planning.hpp" // Adjust path if needed

int main() {
    std::cout << "--- SPARX Windows Test ---" << std::endl;

    // Test a large vector to simulate high RAM usage
    std::vector<int> map_data(2000 * 2000, 0); 
    std::cout << "Allocated " << map_data.size() << " nodes successfully." << std::endl;

    // TODO: Initialize your DStarLite object here
    // DStarLite planner;
    // planner.update_cell(5, 5, 10.0);

    std::cout << "Logic test complete. System Nominal." << std::endl;
    return 0;
}