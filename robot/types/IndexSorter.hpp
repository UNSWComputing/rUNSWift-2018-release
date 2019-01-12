#pragma once

/*
Struct to allow creation of a vector of sorted indexes.
Use as std::sort(indexVec.begin(), indexVec.end(), IndexCompare(dataVec)).
indexVec should be sorted vector of the indexes from 0 to dataVec.size()-1.
dataVec is the vector for which you want the sorted indexes.
Credit:
https://stackoverflow.com/questions/25921706/creating-a-vector-of-indices-of-a-
sorted-vector
*/
struct IndexCompareLess
{
    const std::vector<int>& target;

    IndexCompareLess(const std::vector<int>& target): target(target) {};

    bool operator()(int a, int b) const { return target[a] < target[b]; }
};
struct IndexCompareGreater
{
    const std::vector<int>& target;

    IndexCompareGreater(const std::vector<int>& target): target(target) {};

    bool operator()(int a, int b) const { return target[a] > target[b]; }
};
