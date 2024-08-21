// sort_utils.h
#ifndef SORT_UTILS_H
#define SORT_UTILS_H

#include <utility> // For std::pair

inline bool sortByVal(const std::pair<int, int> &a, const std::pair<int, int> &b) {
    return a.second < b.second; // sort by increasing order of value
}

inline bool sortByValDesc(const std::pair<int, int> &a, const std::pair<int, int> &b) {
    return a.second > b.second; // sort by decreasing order of value
}

inline bool sortByValVal(const std::pair<int, std::pair<int, int>>& a,
                   const std::pair<int, std::pair<int, int>>& b) {
    if (a.second.first == b.second.first) {
        return a.second.second < b.second.second;
    }
    return a.second.first < b.second.first;
}

#endif // SORT_UTILS_H