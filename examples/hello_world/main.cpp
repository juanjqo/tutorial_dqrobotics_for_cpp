#include <iostream>
#include <dqrobotics/DQ.h>

using namespace Eigen;
using namespace DQ_robotics;

int main()
{
    DQ x = DQ(1);
    std::cout << "Hello World! " <<x <<std::endl;
    return 0;
}
