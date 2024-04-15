#include "Common.hpp"
#include "param/param.hpp"
#include <iostream>
#include <iomanip>

using namespace std;

void setup()
{
    unsigned offset = 0;
    for (unsigned i = 0; i < param::getParamCount(); i++)
    {
        param::paramVarId_t param;
        param::getParamByIndex(i, param);

        cout << setw(3) << left << param.index << " | "
             << setw(3) << left << offset << " | "
             << setw(30) << left << param.ptr->name << " | " << endl;
        //  << setw(8) << param.ptr-> << " | " << endl;
    }
}