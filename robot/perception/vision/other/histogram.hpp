#ifndef PERCEPTION_VISION_OTHER_HISTOGRAM_HPP_
#define PERCEPTION_VISION_OTHER_HISTOGRAM_HPP_

#include <vector>
#include <numeric>

#include "perception/vision/Region.hpp"
#include "perception/vision/other/Ransac.hpp"
#include "perception/vision/detector/BallDetector.hpp"

// The number of buckets in the histogram.
#define HISTOGRAM_BUCKETS 16

// The maximum number of pixels considered when building a histogram. Density
// will be reduced by factors of 2 until the number of pixels is below this.
#define MAX_HISTOGRAM_PIXELS 32*32

// Creates a normalised histogram from a region
void makeNormalisedHistogram(
        const RegionI& region,
        float buckets[],
        int num_buckets
) {
    // The total number of pixels.
    int total = region.getCols()*region.getRows();

    // Build the histogram.
    RegionI::iterator_raw cur_point = region.begin_raw();
    for(int pixel=0; pixel<total; ++pixel)
    {
        buckets[(*(cur_point.raw()))/(256/num_buckets)] += 1;
        ++cur_point;
    }

    // Normalise the histogram.
    for(int bucket=0; bucket<num_buckets; ++bucket)
        buckets[bucket] /= (float)total;
}

std::vector<int> makeHistogram(
        const RegionI& region,
        int num_histogram_buckets
) {
    // The total number of pixels.
    int total = region.getCols()*region.getRows();

    // The histogram buckets.
    std::vector<int> histogram(num_histogram_buckets, 0);
    int bucket_size = 256 / num_histogram_buckets;

    // Build the histogram.
    RegionI::iterator_raw cur_point = region.begin_raw();
    for(int pixel=0; pixel<total; ++pixel)
    {
        histogram[(*(cur_point.raw()))/bucket_size] += 1;
        ++cur_point;
    }

    return histogram;
}

std::vector<int> getMaxYPerRowInCircle(
        const RegionI& region,
        const RANSACCircle c)
{
    int x=0, y=0;
    int max_y = 0; 
    float radius_sq = c.radius * c.radius; 
    int total = region.getCols() * region.getRows();
    std::vector<int> arr;

    //std::cout << "Y values"  << std::endl;
    RegionI::iterator_raw cur_point = region.begin_raw();
    for (int pixel=0; pixel<total; ++pixel, ++x, ++cur_point){
        // Reset x, increment y.
        if (x == region.getCols()){
            arr.push_back(max_y);

            //std::cout << "y: " << max_y << std::endl;
            ++y;
            x=0;
            max_y=0;  
        }

        /// TODO(Addo): Fix this to skip the top regions or bottom regions if necessary. 
        // Or locate the circle better. 
        if(DISTANCE_SQR((float)x, (float)y, c.centre.x(), c.centre.y()) > radius_sq)
        {
            // Outside the circle.
            continue; 
        } else {
            max_y = std::max(max_y,(int) (*(cur_point.raw())));
        }
    }

    return arr;
}

std::vector<int> makeNormalisedHistogramInCircle(
        const RegionI& region,
        const RANSACCircle c, 
        int num_histogram_buckets, 
        const std::vector<int> brightness,
        BallDetectorVisionBundle& bdvb)
{
    int x=0, y=0;
    float radius_sq = c.radius * c.radius; 
    int total = region.getCols() * region.getRows();
    std::vector<int> histogram(num_histogram_buckets, 0);
    //int average = std::accumulate(brightness.begin(), brightness.end(), 0) / brightness.size();

    //std::cout << "Avg: " << average << std::endl;
    
    // Save the contrast adjustment values for later. 
    bdvb.contrast_row_multiplier.clear();
    double contrast = 255.0/(std::max(1, brightness[y]));

    bdvb.contrast_row_multiplier.push_back(contrast);

    RegionI::iterator_raw cur_point = region.begin_raw();
    for (int pixel=0; pixel<total; ++pixel, ++x, ++cur_point){
        // Reset x, increment y. 
        if (x == region.getCols()){
            contrast = 255.0/(std::max(1, brightness[y]));
            bdvb.contrast_row_multiplier.push_back(contrast);
            ++y;
            x=0;
        }

        /// TODO(Addo): Fix this to skip the top regions or bottom regions if necessary. 
        // Or locate the circle better. 
        if (DISTANCE_SQR((float)x, (float)y, c.centre.x(), c.centre.y()) > radius_sq)
        {
            // Outside the circle.
            continue; 
        } else {
            //int diff = average - brightness[y];
            //int diff = 40 - brightness[y];
            //int pix_val = std::min(std::max(*(cur_point.raw()) * 1.1 + diff , 0.0
            //            ), 255.0);
            //std::cout << "Max brightness " << brightness[y] << ", Contrast " << contrast;
            
            int pix_val = std::min(std::max(((*cur_point.raw())* contrast) , 0.0
                        ), 255.0);
            //std::cout << " Adding " << pix_val << ", Diff " << diff <<", original " <<(int)*cur_point.raw() << std::endl;
            histogram[(pix_val)/(256/num_histogram_buckets)] += 1;
        }
    }

    return histogram;
}


std::vector<int> makeHistogramInCircle(
        const RegionI& region,
        const RANSACCircle c, 
        const int middle, 
        int num_histogram_buckets, 
        bool topSection=true)
{
    int x=0, y=0;
    float radius_sq = c.radius * c.radius; 
    int total = region.getCols() * region.getRows();
/*
    if (topSection){
        total = region.getCols() * middle;
    } else {
        total = region.getCols() * (middle - region.getRows());
    }
*/
    std::vector<int> histogram(num_histogram_buckets, 0);

    RegionI::iterator_raw cur_point = region.begin_raw();
    for (int pixel=0; pixel<total; ++pixel, ++x, ++cur_point){
        // Reset x, increment y. 
        if (x == region.getCols()){
            ++y;
            x=0;
        }

        /// TODO(Addo): Fix this to skip the top regions or bottom regions if necessary. 
        // Or locate the circle better. 
        if ((y > middle && topSection) || (y < middle && !topSection) ||
                DISTANCE_SQR((float)x, (float)y, c.centre.x(), c.centre.y()) > radius_sq)
        {
            // Outside the circle.
            continue; 
        } else {
            histogram[(*(cur_point.raw()))/(256/num_histogram_buckets)] += 1;
        }
    }

    return histogram;
}
#endif
