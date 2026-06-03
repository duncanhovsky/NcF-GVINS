#include "../ic_gvins/ic_gvins/fileio/filesaver.h"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

int main() {
    const std::string filename = "test_tum_filesaver_output.csv";
    {
        FileSaver saver(filename, 8, FileSaver::TUM_TEXT);
        assert(saver.isOpen());
        saver.dump(std::vector<double>{184126.25, 1.0, -2.5, 3.125, 0.1, 0.2, 0.3, 0.9});
        saver.flush();
    }

    std::string line;
    {
        std::ifstream input(filename);
        std::stringstream buffer;
        buffer << input.rdbuf();
        line = buffer.str();
        assert(line ==
               "184126.250000000 1.000000000 -2.500000000 3.125000000 0.100000000 "
               "0.200000000 0.300000000 0.900000000\n");
    }

    std::remove(filename.c_str());
    return 0;
}
