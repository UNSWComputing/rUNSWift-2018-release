#ifndef PERCEPTION_VISION_OTHER_OTSU_HPP_
#define PERCEPTION_VISION_OTHER_OTSU_HPP_

#include <vector>

/* Given a histogram, calculate the threshold value using Otsu's method.
 * The histogram should be vector of integers.
 * Loosely based off the wikipedia page but with better variable names and comments.
 */
inline int getThresholdValueOtsu(std::vector<int> &values, double &intra_class_var) {
    long long weight_total = 0;
    long long weight_left = 0;
    long long weight_right;
    double max = 0.0;
    long long mean_numerator_total = 0;
    long long mean_numerator_left = 0;
    long long mean_numerator_right;
    double mean_left;
    double mean_right;
    double between;
    long long threshold = 0;
    long long value = 0;

    /* Go through and calculate otsu's */
    int t;
    std::vector<int>::iterator it;

    for (t = 0, it = values.begin(); it != values.end(); ++t, ++it) {
        weight_total += *it;
        mean_numerator_total   += *it * t;
    }

    weight_right = weight_total;
    mean_numerator_right = mean_numerator_total;

    for (t = 0, it = values.begin(); it != values.end(); ++t, ++it) {
        value = *it;

        if (value != 0) {
        /* calculate weight for left and right of the pivot */
            weight_left  += value;
            weight_right -= value;

            if (weight_right == 0){
                break;
            }

            mean_numerator_left  += t * value;
            mean_numerator_right -= t * value;

            mean_left  = (1.0 * mean_numerator_left) / weight_left;
            mean_right = (1.0 * mean_numerator_right) / weight_right;
            between = weight_left * weight_right * (mean_left - mean_right) * (mean_left - mean_right);
            if (between >= max){
                max = between;
                threshold = t + 1;
            }
        } 
    }

    // Multiply by the range of intensities that each bucket covers
    intra_class_var = max;
    return threshold * 256 / values.size();
}

#endif