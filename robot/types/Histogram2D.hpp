//
// Created by gary on 3/8/17.
//
#ifndef RUNSWIFT_HISTOGRAM2D_HPP
#define RUNSWIFT_HISTOGRAM2D_HPP

#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <utility>
#include <stdexcept>

typedef std::pair<int, int> binPos;

template <typename T>
class Histogram2D {
private:
    std::vector<double> X1_bins_;
    std::vector<double> X2_bins_;
    std::vector<std::vector<double> > density_;
    std::vector<std::vector<int> > counts_;

    bool filtered_;

    // Variable X1 is the the "outer" variable in the above vectors, X2 the "inner"
    int X1_bin_num_; // Number of bins the first Variable has
    int X2_bin_num_; // Number of bins the second Variable has

    int num_data_; // Number of Data points

    void filterVerticalPeaks(int max_bins);

    std::vector<std::pair<int, int> > getBinNeighbours(std::pair<int, int> bin);

public:
    std::vector<std::vector<bool> > filtered_bins_;

    // constructors
    Histogram2D();

    Histogram2D(std::vector<T> &X1_values, std::vector<T> &X2_values);

    Histogram2D(std::vector<T> &X1_values, std::vector<T> &X2_values,
                std::vector<double> X1_bins, std::vector<double> X2_bins);

    Histogram2D(const Histogram2D &obj);

    Histogram2D& operator=(const Histogram2D& obj);

    // destructor
    ~Histogram2D();

    // Add more data to the Histogram
    void addData(std::vector<T> &X1_values, std::vector<T> &X2_values);

    void showHistogram();

    double getDensity(T X1_val, T X2_val);

    // Filter bins with a method. Initializes private member filtered_bins_;
    void filterBins(int max_bins, std::string method = "vertical_peak");

    // Check if a value is filtered.
    bool isFiltered(T X1_val, T X2_val);
    bool areNeighboursFiltered(T X1_val, T X2_val);
    std::vector<std::vector<bool> >& getFilteredBins();
    void showFilteredBins();


    // Get the bin pos
    binPos getBinPos(T X1_val, T X2_val);
};

#include "Histogram2D.tcc"

#endif //RUNSWIFT_HISTOGRAM2D_HPP
