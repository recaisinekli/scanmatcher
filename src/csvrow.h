#ifndef CSVROW_H
#define CSVROW_H
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>

class CSVRow
{
public:
    CSVRow();
    std::string const& operator[](std::size_t index) const;
    std::size_t size() const;
    std::istream& readNextRow(std::istream& str);
private:
    std::vector<std::string>    m_data;
};

#endif // CSVROW_H
