#include "csvrow.h"

CSVRow::CSVRow()
{
}

std::string const& CSVRow::operator[](std::size_t index) const
{
    return m_data[index];
}
std::size_t CSVRow::size() const
{
    return m_data.size();
}
std::istream& CSVRow::readNextRow(std::istream& str)
{
    std::string         line;
    std::getline(str,line);

    std::stringstream   lineStream(line);
    std::string         cell;

    m_data.clear();
    while(std::getline(lineStream,cell,','))
    {
        m_data.push_back(cell);
    }
    return str;
}
