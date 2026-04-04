#include <iostream>
#include <vector>
#include "../Planning/Planning.hpp" // Adjust path if needed

int main() {
    std::cout << "--- SPARX Windows Test ---" << std::endl;
    std::vector<int> map_data(2000 * 2000, 0); 
    std::cout << "Allocated " << map_data.size() << " nodes successfully." << std::endl;
    return 0;
}