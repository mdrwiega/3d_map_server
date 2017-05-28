#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <stdexcept>

#include <boost/spirit/include/karma.hpp>
namespace karma = boost::spirit::karma;

namespace table_printer {
class endl{};

/** \class TablePrinter

  Print a table into specified output stream.

  Usage:
    TablePrinter tp(&std::cout);
    tp.AddColumn("Name", 20);
    tp.AddColumn("Age", 3);

    tp.PrintHeader();
    tp << "Dat Chu" << 25;
    tp << "John Doe" << 26;
    tp.PrintFooter();

 */
class TablePrinter
{
public:
    TablePrinter(std::ostream * output, const std::string & separator = "|")
        : outStream_(output), separator_(separator) { }

    ~TablePrinter() = default;

    size_t getColumns() const { return columnHeaders_.size(); }
    size_t getTableWidth() const { return tableWidth_; }
    void setSeparator(const std::string & separator) { separator_ = separator; }
    void setFlushLeft() { flushLeft_ = true; }
    void setFlushRight() { flushLeft_ = false; }

    void AddColumn(const std::string & headerName, int columnWidth)
    {
        if (columnWidth < 4)
            throw std::invalid_argument("Column size has to be >= 4");

        columnHeaders_.push_back(headerName);
        columnWidths_.push_back(columnWidth);
        tableWidth_ += columnWidth + separator_.size(); // for the separator
    }

    void PrintHeader()
    {
        PrintHorizontalLine();
        *outStream_ << "|";

        for (size_t i = 0; i < getColumns(); ++i)
        {
            if(flushLeft_)
                *outStream_ << std::left;
            else
                *outStream_ << std::right;

            *outStream_ << std::setw(columnWidths_.at(i))
                        << columnHeaders_.at(i).substr(0, columnWidths_.at(i));

            if (i != getColumns()-1)
                *outStream_ << separator_;
        }

        *outStream_ << "|\n";
        PrintHorizontalLine();
    }

    void PrintTitle(std::string title)
    {
        PrintHorizontalLine();
        *outStream_ << "| " << title;
        for (size_t i = 0; i < (tableWidth_ - 2 - title.size()); ++i)
            *outStream_ << " ";
        *outStream_ << "|\n";
    }

    void PrintFooter()
    {
        PrintHorizontalLine();
    }

    TablePrinter& operator<<(endl input)
    {
        while (col_ != 0)
            *this << "";
        return *this;
    }

    template<typename T>
    TablePrinter& operator<<(T input)
    {
        if (col_ == 0)
            *outStream_ << "|";

        if(flushLeft_)
            *outStream_ << std::left;
        else
            *outStream_ << std::right;

        *outStream_ << std::setw(columnWidths_.at(col_))
                    << std::setprecision(3) << input;

        if (col_ == getColumns()-1)
        {
            *outStream_ << "|\n";
            row_++;
            col_ = 0;
        }
        else
        {
            *outStream_ << separator_;
            col_++;
        }

        return *this;
    }

private:
    void PrintHorizontalLine()
    {
        *outStream_ << "+"; // the left bar

        for (size_t i=0; i < tableWidth_-1; ++i)
            *outStream_ << "-";

        *outStream_ << "+"; // the right bar
        *outStream_ << "\n";
    }

    template<typename T>
    void OutputDecimalNumber(T input)
    {
        auto colWidth = columnWidths_.at(col_);
        *outStream_ << karma::format(karma::maxwidth(colWidth)
            [karma::right_align(colWidth)[karma::double_]], input);

        if (col_ == getColumns()-1)
        {
            *outStream_ << "|\n";
            row_++;
            col_ = 0;
        }
        else
        {
            *outStream_ << separator_;
            col_++;
        }
    }

    std::ostream * outStream_;
    std::vector<std::string> columnHeaders_;
    std::vector<int> columnWidths_;
    std::string separator_;

    size_t row_{0};
    size_t col_{0};

    size_t tableWidth_{0};
    bool flushLeft_{false};
};

}
