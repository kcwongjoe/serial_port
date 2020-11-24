#include "eg1_print_ports.h"
#include "eg2_read.h"
#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "Example 1: Print serial port information." << std::endl;
    std::cout << "Example 2: Read data stream." << std::endl;
    std::cout << "Enter the example number: ";

    int example_index;
    std::cin >> example_index;

    if (example_index == 1)
    {
        eg1_print_ports();
    }
    else if (example_index == 2)
    {
        eg2_read();
    }    
}