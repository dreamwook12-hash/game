#include "test_support.hpp"
void runFoundationTests(TestSuite&);
void runFlightTests(TestSuite&);
void runSessionTests(TestSuite&);
int main() {
    TestSuite suite;
    runFoundationTests(suite); runFlightTests(suite); runSessionTests(suite);
    std::cout<<"RESULT passed="<<suite.passed<<" failed="<<suite.failed<<'\n';
    return suite.failed==0?0:1;
}
