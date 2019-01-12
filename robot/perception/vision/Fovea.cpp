#include <iostream>
#include <algorithm>
#include <vector>
#include <utility>

#include "perception/vision/Fovea.hpp"


//#define FOVEA_TIMINGS
#ifdef FOVEA_TIMINGS
#include "utils/Timer.hpp"
#endif // FOVEA_TIMINGS

/**
 * Free the _colour arrays.
 */
Fovea::~Fovea()
{//*
    // Clear existing child fovea.
    for(unsigned int fovea=0; fovea<child_fovea_.size(); ++fovea)
        delete child_fovea_[fovea];
    child_fovea_.clear();

    // Clear data array.
    if(hasColour)
        delete[] _colour;
    //*/
}


/**
 * Gets a pointer to the first item in the colour array.
 */
const Colour* Fovea::getColourArray()
{
    return _colour;
}

/**
 * Gets a pointer to the first item in the raw image.
 */
const uint8_t* Fovea::getRawPixelArray()
{
    return _rawImage;
}

/**
 * Updates the fovea to use the data in combined_frame and
 * adaptive thresholding window size and percentage
 */
void Fovea::generate(const CombinedFrame& combined_frame,
                     const int win_size, const int perc, bool do_body_part)
{
#ifdef FOVEA_TIMINGS
    static int startStopTime = 0;
    static int colourTime = 0;
    static int blurTime = 0;
    static int edgeTime = 0;
    static int frameCount = 0;

    if(bb.width() == TOP_SALIENCY_COLS)
        ++frameCount;

    Timer timer;
    timer.restart();
#endif // FOVEA_TIMINGS

    // Clear existing child fovea.
    for(unsigned int fovea=0; fovea<child_fovea_.size(); ++fovea)
        delete child_fovea_[fovea];
    child_fovea_.clear();

    // Record combined_frame and colour_classifier in case child fovea need
    // them.
    combined_frame_ = &combined_frame;
    //colour_classifier_ = &colour_classifier;

    // Set Adaptive thresholding values
    setWindowSize(win_size);
    setPercentage(perc);

    // Record the frame.
    if(top)
        _rawImage = combined_frame.top_frame_;
    else
        _rawImage = combined_frame.bot_frame_;

    // Translate the y axis stop coordinates into linear stop start coordinates.
    std::pair<std::vector<int>*, std::vector<const uint8_t*>* > startStop =
                                                                getStartStop_();

#ifdef FOVEA_TIMINGS
    if(bb.width() == TOP_SALIENCY_COLS)
        startStopTime += timer.elapsed_us();
    timer.restart();
#endif // FOVEA_TIMINGS

    // Generate colour image if needed.
    if(hasColour)
    {
        makeBinary_(*startStop.first, do_body_part);
        // if(top)
        // {
        //     makeColourTop_(combined_frame, colour_classifier, *startStop.first,
        //                                                      *startStop.second);
            
        // }
        // else
        // {
        //     makeColourBot_(combined_frame, colour_classifier, *startStop.first,
        //                                                      *startStop.second);
        // }
    }

#ifdef FOVEA_TIMINGS
    if(bb.width() == TOP_SALIENCY_COLS)
        colourTime += timer.elapsed_us();
    timer.restart();
#endif // FOVEA_TIMINGS

#ifdef FOVEA_TIMINGS
    if(bb.width() == TOP_SALIENCY_COLS)
        blurTime += timer.elapsed_us();
    timer.restart();
#endif // FOVEA_TIMINGS

#ifdef FOVEA_TIMINGS
    if(bb.width() == TOP_SALIENCY_COLS)
        edgeTime += timer.elapsed_us();
#endif // FOVEA_TIMINGS

    // Clean up x start stop.
    delete startStop.first;
    delete startStop.second;

#ifdef FOVEA_TIMINGS
    if(frameCount == 1000)
    {
        std::cout << "1000 frame average" << std::endl << "Width: " << bb.width() << " Height: " << bb.height() << std::endl;
        std::cout << "Start stop time: " << startStopTime/frameCount << std::endl;
        std::cout << "Colour classification: " << colourTime/frameCount << std::endl;
        std::cout << "Grey blur: " << blurTime/frameCount << std::endl;
        std::cout << "Edge detection: " << edgeTime/frameCount << std::endl << std::endl;
        startStopTime = 0;
        colourTime = 0;
        blurTime = 0;
        edgeTime = 0;
        frameCount = 0;
    }
#endif // FOVEA_TIMINGS
}

/*
 * Translates the y axis robot part stop array to a linear start stop array.
 */
std::pair<std::vector<int>*, std::vector<const uint8_t*>* >
                                                    Fovea::getStartStop_() const
{
    // The number of columns in the image.
    const int doubleCols = 2*(top*TOP_IMAGE_COLS+(!top)*BOT_IMAGE_COLS);

    // Create a startStop array.
    std::vector<int>* startStop = new std::vector<int>();
    std::vector<const uint8_t*>* startStopRaw =
                                              new std::vector<const uint8_t*>();
    startStop->reserve(bb.height()*4);

    // The number of x and y axis rows covered by this fovea.
    int xAxisCols = bb.width();
    int yAxisRows = bb.height();

    // The y axis stopping points relevant to this fovea, sorted by height.
    // Stored as y, x, in descending order.
    std::vector<std::pair<int, int> > sortedStops;

    // The set of starts and stops relevant at this y value.
    std::vector<int> relevantStartStops;

    // Iterator to the next y value to include.
    std::vector<std::pair<int, int> >::iterator highestY;

    // The first pixel in the fovea in the raw image.
    const uint8_t* firstPixel = _rawImage + bb.a.x()*density*2 +
                                                    bb.a.y()*density*doubleCols;

    // Create the sorted y axis stop array.
    sortedStops.reserve(xAxisCols);
    if(top)
    {
        for(int x=0; x<xAxisCols; ++x)
        {
            sortedStops.push_back(std::make_pair(
                std::max(combined_frame_->camera_to_rr_.getTopEndScanCoord(x*density)/density, 0),
                                                                            x));
        }
    }
    else
    {
        for(int x=0; x<xAxisCols; ++x)
        {
            sortedStops.push_back(std::make_pair(
                std::max(combined_frame_->camera_to_rr_.getBotEndScanCoord(x*density)/density, 0),
                                                                            x));
        }
    }
    std::sort(sortedStops.begin(), sortedStops.end());

    // Initialise the highestY pointer.
    highestY = sortedStops.begin();

    // Set up relevantStartStops.
    relevantStartStops.reserve(10);
    relevantStartStops.push_back(0);
    relevantStartStops.push_back(bb.width());

    // Build the startStop array.
    for(int y=0; y<yAxisRows; ++y)
    {
        // The linear location of the start of this row.
        int startVal = y*bb.width();
        const uint8_t* startValRaw = firstPixel+y*density*doubleCols;

        // Firstly, check for new y stops to include.
        while(highestY < sortedStops.end() && (*highestY).first <= y)
        {
            // A new stop needs to be added, work out where it goes.
            int newX = (*highestY).second;
            bool stopPoint = false; // Start stop of regular classification.
            for(unsigned int point=0; point<relevantStartStops.size(); ++point)
            {
                // Check if newX should extend the robot block to the right.
                if(!stopPoint && newX == relevantStartStops[point])
                {
                    // Skip as long as we're not at the end, the y value is
                    // the same, we haven't gone past the next point, and the
                    // next point is next to this one.
                    while(highestY < sortedStops.end()-1 &&
                            (*(highestY+1)).first == (*highestY).first &&
                            (point+1 >= relevantStartStops.size() ||
                            ((*(highestY+1)).second <
                            relevantStartStops[point+1] &&
                               (*(highestY+1)).second == (*highestY).second+1)))
                        ++highestY;
                    relevantStartStops[point] = (*highestY).second+1;

                    // Once the point is added break from the loop.
                    break;
                }

                // Otherwise check if newX should extend a robot block to the
                // left.
                else if(stopPoint && newX < relevantStartStops[point])
                {
                    // Firstly find the size of this group and check if it
                    // connects to the right.
                    int maxX[2];
                    maxX[0] = newX;
                    maxX[1] = newX;

                    // Skip as long as we're not at the end, the y value is
                    // the same, we haven't gone past the next point, and the
                    // next point is next to this one.
                    while(highestY < sortedStops.end()-1 &&
                            (*(highestY+1)).first == (*highestY).first &&
                            (point+1 >= relevantStartStops.size() ||
                            ((*(highestY+1)).second < relevantStartStops[point]
                            && (*(highestY+1)).second == (*highestY).second+1)))
                        ++highestY;
                    maxX[1] = (*highestY).second+1;

                    // Check for merging with the group on the right.
                    if(maxX[1] == relevantStartStops[point])
                    {
                        // Extend the group to the left.
                        relevantStartStops[point] = newX;

                        // Once the point is added break from the loop.
                        break;
                    }

                    // This group doesn't connect with an existing group, so
                    // insert a new one.
                    relevantStartStops.insert(relevantStartStops.begin()+point,
                                                                  maxX, maxX+2);

                    // Once the point is added break from the loop.
                    break;
                }

                // Toggle whether this is a start point.
                stopPoint = !stopPoint;
            }

            // Lastly check if any two groups should be merged.
            for(unsigned int point=2; point<relevantStartStops.size()-2;
                                                                       point+=2)
            {
                if(relevantStartStops[point] == relevantStartStops[point+1])
                {
                    // The groups can be merged. Just delete these two values.
                    relevantStartStops.erase(relevantStartStops.begin() +
                               (point), relevantStartStops.begin()+(point+2));

                    // Decrement point so that nothing is skipped.
                    point -= 2;
                }
            }

            // Move to the next highestY.
            ++highestY;
        }

        // Now that relevantStartStops is up to date, make use of it.
        for(unsigned int xVal=0; xVal<relevantStartStops.size(); ++xVal)
        {
            startStop->push_back(startVal+relevantStartStops[xVal]);
            startStopRaw->push_back(startValRaw +
                                            relevantStartStops[xVal]*density*2);
        }
    }

    // Return the result.
    return(std::make_pair(startStop, startStopRaw));
}

/**
 * Creates a colour image. Optimised for using the top image.
 */
void Fovea::makeColourTop_(const CombinedFrame& combined_frame,
    const ColourClassifier& colour_classifier,
    const std::vector<int>& startStop,
                                const std::vector<const uint8_t*>& startStopRaw)
{
    // Buffer variables for speed.
    const int doubleDensity = density*2;

    // If there is nothing to look at, return.
    if(startStop.size() == 0)
        return;

    // The current point in the saliency image.
    Colour* saliencyPixel = _colour;

    // The current point in the underlying image.
    const uint8_t* rawPixel;

    // The next change point in the saliency image.
    Colour* saliencyEnd;

    // The block currently being looked at.
    unsigned int block = 0;

    // The start needs to be classified as robot.
    if(startStop[0] != 0)
    {
        saliencyEnd = _colour + startStop[0];

        // Run mark this group of pixels as robot grey.
        for(; saliencyPixel<saliencyEnd; ++saliencyPixel)
            *saliencyPixel = cBODY_PART;
    }

    // Classify the blocks.
    while(block < startStop.size()-1)
    {
        // Even blocks use standard classification.
        saliencyPixel = _colour + startStop[block];
        saliencyEnd = _colour + startStop[block+1];
        rawPixel = startStopRaw[block];
        for(; saliencyPixel < saliencyEnd; ++saliencyPixel)
        {
            // Classify the pixel.
            *saliencyPixel = colour_classifier.classifyTop(rawPixel);

            // Move to the next raw pixel.
            rawPixel += doubleDensity;
        }

        // Move to the next block.
        ++block;
        if(!(block < startStop.size()-1))
            break;

        // Odd blocks are just robot parts.
        saliencyPixel = _colour + startStop[block];
        saliencyEnd = _colour + startStop[block+1];
        for(; saliencyPixel < saliencyEnd; ++saliencyPixel)
            *saliencyPixel = cBODY_PART;

        // Move to the next block.
        ++block;
    }

    // Classify any trailing values as robot parts.
    saliencyPixel = _colour + (*(startStop.end()-1));
    saliencyEnd = _colour + bb.height() * bb.width();
    for(; saliencyPixel<saliencyEnd; ++saliencyPixel)
        *saliencyPixel = cBODY_PART;
}

/**
 * Creates a colour image. Optimised for using the top image.
 */
void Fovea::makeColourBot_(const CombinedFrame& combined_frame,
    const ColourClassifier& colour_classifier,
    const std::vector<int>& startStop,
                                const std::vector<const uint8_t*>& startStopRaw)
{
    // Buffer variables for speed.
    const int doubleDensity = density*2;

    // If there is nothing to look at, return.
    if(startStop.size() == 0)
        return;

    // The current point in the saliency image.
    Colour* saliencyPixel = _colour;

    // The current point in the underlying image.
    const uint8_t* rawPixel;

    // The next change point in the saliency image.
    Colour* saliencyEnd;

    // The block currently being looked at.
    unsigned int block = 0;

    // The start needs to be classified as robot.
    if(startStop[0] != 0)
    {
        saliencyEnd = _colour + startStop[0];

        // Run mark this group of pixels as robot grey.
        for(; saliencyPixel<saliencyEnd; ++saliencyPixel)
            *saliencyPixel = cBODY_PART;
    }

    // Classify the blocks.
    while(block < startStop.size()-1)
    {
        // Even blocks use standard classification.
        saliencyPixel = _colour + startStop[block];
        saliencyEnd = _colour + startStop[block+1];
        rawPixel = startStopRaw[block];
        for(; saliencyPixel < saliencyEnd; ++saliencyPixel)
        {
            // Classify the pixel.
            *saliencyPixel = colour_classifier.classifyBot(rawPixel);

            // Move to the next raw pixel.
            rawPixel += doubleDensity;
        }

        // Move to the next block.
        ++block;
        if(!(block < startStop.size()-1))
            break;

        // Odd blocks are just robot parts.
        saliencyPixel = _colour + startStop[block];
        saliencyEnd = _colour + startStop[block+1];
        for(; saliencyPixel < saliencyEnd; ++saliencyPixel)
            *saliencyPixel = cBODY_PART;

        // Move to the next block.
        ++block;
    }

    // Classify any trailing values as robot parts.
    saliencyPixel = _colour + (*(startStop.end()-1));
    saliencyEnd = _colour + bb.height() * bb.width();
    for(; saliencyPixel<saliencyEnd; ++saliencyPixel)
        *saliencyPixel = cBODY_PART;
}

/**
 * Creates a binary image.
 * There are four alternative algorithms implemented.
 * 1. Global thresholding using blurred grayscale
 * 2. Global thresholding using raw image
 * 3. Adaptive thresholding using blurred grayscale
 * 4. Adaptive thresholding using raw image
 */
void Fovea::makeBinary_(const std::vector<int>& startStop, bool do_body_part)
{
    // The width and height of the bounding box.
    const int width = bb.width();
    const int height = bb.height();

    // The current pixel in the binary image
    // Binary* curr_pixel;
    Colour* curr_pixel;

    /* 
     * 1. Global Thresholding using blurred grayscale
     */
    // const int* curr_grey;
    // curr_pixel = _binary;
    // curr_grey = _grey;
    //
    // for(int y=0; y < height; ++y)
    // {
    //     for(int x=0; x < width; ++x)
    //     {
    //         const int threshValue = 125 * 16;
    //         if ((*curr_grey) < threshValue){
    //             (*curr_pixel) = bBlack;
    //         } else {
    //             (*curr_pixel) = bWhite;
    //         }
    //         ++curr_pixel;
    //         ++curr_grey;
    //     }
    // }

    /*
     * 2. Global Thresholding using raw image
     */
    // const uint8_t* curr_raw;
    // curr_raw = _rawImage;
    // curr_pixel = _binary;
    //
    // // The distance between two saliency density rows.
    // const int row_size = density*2*(TOP_IMAGE_COLS*top + BOT_IMAGE_COLS*(!top));
    //
    // for(int y=0; y < height; ++y)
    // {
    //     for (int x=0; x < width; ++x)
    //     {
    //         const int threshValue = 150;
    //         if ((*curr_raw) < threshValue){
    //             (*curr_pixel) = bBlack;
    //         } else {
    //             (*curr_pixel) = bWhite;
    //         }
    //         i++;
    //         ++curr_pixel;
    //         curr_raw += density*2;
    //     }
    //     curr_raw += row_size - width*density*2;
    // }

    /* 
     * 3. Adaptive Thresholding using blurred grayscale. Algorithm described in
     * http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.420.7883&rep=rep1&type=pdf
     */

    // // Create _intImg
    // int *const _intImg = new int[width * height];
    //
    // // Create pointers
    // const int* curr_grey;
    // int* curr_intImg;
    //
    // // Initialise pointers
    // curr_pixel = _binary;
    // curr_grey = _grey;
    // curr_intImg = _intImg;
    //
    // int sum;
    //
    // for(int i=0; i < height; ++i)
    // {
    //     sum = 0;
    //     for(int j=0; j < width; ++j)
    //     {
    //         sum += (*curr_grey);
    //         if (i == 0){
    //             _intImg[i*width+j] = sum;
    //         } else {
    //             _intImg[i*width+j] = _intImg[(i-1)*width+j] + sum;
    //         }
    //         ++curr_grey;
    //         ++curr_intImg;
    //     }
    // }
    //
    // // Initialise variables for second part
    // int x1, x2, y1, y2;
    // int count;
    // int s;
    // int t;
    //
    // // Reset pointers and variables that were used
    // curr_grey = _grey;
    // curr_intImg = _intImg;
    // sum = 0;
    //
    // // tunable parameters (should be tuned)
    // s = 51;  // window size
    // t = -20; // percentage (more positive - more white, more negative - less white)
    //
    // for(int i=0; i < height; ++i)
    // {
    //     for(int j=0; j < width; ++j)
    //     {
    //     
    //         // edge cases
    //         x1 = std::max(1, std::min(height,i - s/2)-2);
    //         x2 = std::max(1, std::min(height,i + s/2)-1);
    //         y1 = std::max(2, std::min(width, j - s/2)-2);
    //         y2 = std::max(2, std::min(width, j + s/2)-1);
    //
    //         count = (x2-x1) * (y2-y1);
    //         sum = _intImg[x2*width+y2] - _intImg[x2*width+(y1-1)] - _intImg[(x1-1)*width+y2] + _intImg[(x1-1)*width+(y1-1)];
    //      
    //         if ((*curr_grey * count) <= (sum * (100-t)/100)){
    //             *curr_pixel = bBlack;
    //         } else {
    //             *curr_pixel = bWhite;
    //         }
    //
    //         ++curr_grey;
    //         ++curr_pixel;
    //     }
    // }


    /* 
    * 4. Adaptive Thresholding using raw image. Algorithm from
    * http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.420.7883&rep=rep1&type=pdf
    */

    // Create _intImg
    int *const _intImg = new int[width * height];

    // The distance between two saliency density y values, as the raw array is
    // in the YUV422 format (YUYVYUYV...).
    const int double_density = density*2;

    // The distance between two saliency density rows.
    const int row_size = density*2*(TOP_IMAGE_COLS*top + BOT_IMAGE_COLS*(!top));

    // Create pointers
    const uint8_t* curr_raw;
    int* curr_intImg;
    int* upper_intImg; // the pixel just on the top of current pixel

    // Initialise pointers
    curr_pixel = _colour;
    curr_raw = _rawImage + bb.a.x()*double_density + bb.a.y()*row_size;
    curr_intImg = _intImg;
    upper_intImg = _intImg;

    int sum;

    for(int i=0; i < height; ++i)
    {
        sum = 0;
        for(int j=0; j < width; ++j)
        {
            sum += (*curr_raw);
            if (i == 0){
                *curr_intImg = sum;
            } else {
                *curr_intImg = *upper_intImg + sum;
                ++upper_intImg;
            }
            curr_raw += double_density;
            ++curr_intImg;
        }
        curr_raw += row_size - width * double_density;
    }

    // Initialise variables for second part
    int x1, x2, y1, y2;
    int count;
    int s;
    int t;
    int jumpUp, jumpLeft;


    // tunable parameters
    // s: window size
    // t: percentage (more positive - lower threshold for white (more white))
    s = windowSize;
    t = percentage;

    s = std::min(s, std::min(height,width)-1);
    if( s % 2 == 0) --s;// make s always odd

    // just calculate one time, to save time in loop
    int s_half = s/2;
    int * x2y2_intImg;


    // curr_raw = _rawImage + bb.a.x()*double_density + bb.a.y()*row_size;
    // curr_intImg = _intImg;
    // sum = 0;
    // for(int i=0; i < height; ++i)
    // {
    //     for(int j=0; j < width; ++j)
    //     {
    //         // edge cases
    //         x1 = std::max(1, std::min(height-2,i - s/2));
    //         x2 = std::max(2, std::min(height-1,i + s/2));
    //         y1 = std::max(1, std::min(width-2, j - s/2));
    //         y2 = std::max(2, std::min(width-1, j + s/2));

    //         count = (x2-x1 + 1) * (y2-y1 + 1);
    //         sum = _intImg[x2*width+y2] - _intImg[x2*width+(y1-1)] - _intImg[(x1-1)*width+y2] + _intImg[(x1-1)*width+(y1-1)];
            
    //         if ((*curr_raw * count) * 100 <= (sum * (100-t))){
    //             // *curr_pixel = bBlack;
    //             *curr_pixel = cGREEN;
    //         } else {
    //             // *curr_pixel = bWhite;
    //             *curr_pixel = cWHITE;
    //         }

    //         curr_raw += double_density;
    //         ++curr_pixel;
    //     }
    //     //std::cout<<std::endl;
    //     curr_raw += row_size - width * double_density;
    // }
    // std::cout<<std::endl;

    
    // raw image of the first pixel of this fovea
    const uint8_t * start_raw = _rawImage + bb.a.x()*double_density + bb.a.y()*row_size;


    // loop 1 for x1, y1 outside image
    // Reset pointers and variables that were used
    x1 = 1;
    x2 = s_half;
    y1 = 1;
    y2 = s_half;
    curr_raw = start_raw;// pointer to raw image data of (0,0)
    //curr_intImg = _intImg;// pointer to integral image data of (0,0)
    sum = 0;
    x2y2_intImg = _intImg + s_half * width + s_half;//curr_intImg + x2 * width + y2;
    jumpUp = (x2-x1+1) * width;
    jumpLeft = y2-y1+1;
    curr_pixel = _colour;
    
    for(int i=0; i < s_half+1; ++i)
    {
        for(int j=0; j < s_half+1; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            count = (x2-x1+1)*(y2-y1+1);
            if ((*curr_raw * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            }
            curr_raw += double_density;
            ++curr_pixel;
            ++x2y2_intImg;
            ++jumpLeft;
            ++y2;
        }
        curr_raw += row_size - (s_half + 1) * double_density;
        jumpUp += width;
        ++x2;
        y2-=(s_half+1);
        jumpLeft-=(s_half+1);
        x2y2_intImg += width - s_half - 1;
        curr_pixel += width - s_half - 1;
    }

    // loop 2 for x2, y1 outside image
    // Reset pointers and variables that were used
    x1 = height - 2 * s_half;
    x2 = height - 1;
    y1 = 1;
    y2 = s_half;
    curr_raw = start_raw + row_size * (height - s_half);// pointer to raw image data of (height-s_half,0)
    //curr_intImg = _intImg + (height - s_half) * width;// pointer to integral image data of (height-s_half,0)
    sum = 0;
    x2y2_intImg = _intImg + (height - 1) * width + s_half;
    jumpUp = (x2-x1+1) * width;
    jumpLeft = y2-y1+1;
    curr_pixel = _colour + width * (height - s_half);

    for(int i=height-s_half; i < height; ++i)
    {
        for(int j=0; j < s_half+1; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            count = (x2-x1+1)*(y2-y1+1);
            if ((*curr_raw * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            }
            curr_raw += double_density;
            ++curr_pixel;
            ++x2y2_intImg;
            ++jumpLeft;
            ++y2;
        }
        curr_raw += row_size - (s_half + 1) * double_density;
        jumpUp -= width;
        ++x1;
        y2-=(s_half+1);
        jumpLeft-=(s_half+1);
        x2y2_intImg -= s_half + 1;
        curr_pixel += width - s_half - 1;
    }

    // loop 3 for x1, y2 outside image
    // Reset pointers and variables that were used
    x1 = 1;
    x2 = s_half;
    y1 = width - 2 * s_half;
    y2 = width - 1;
    curr_raw = start_raw + (width - s_half) * double_density;// pointer to raw image data of (0,width-s_half)
    //curr_intImg = _intImg + width - s_half;// pointer to integral image data of (0,width-s_half)
    sum = 0;
    x2y2_intImg = _intImg + width * s_half + width - 1;
    jumpUp = (x2-x1+1) * width;
    jumpLeft = y2-y1+1;
    curr_pixel = _colour + width - s_half;

    for(int i=0; i < s_half+1; ++i)
    {
        for(int j=width-s_half; j < width; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            count = (x2-x1+1)*(y2-y1+1);
            if (((*curr_raw) * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            } 
            curr_raw += double_density;
            ++curr_pixel;
            --jumpLeft;
            ++y1;
        }
        curr_raw += row_size - s_half * double_density;
        jumpUp += width;
        ++x2;
        y1-=s_half;
        jumpLeft+=s_half;
        x2y2_intImg += width;
        curr_pixel += width - s_half;
    }
    
    // loop 4 for x2, y2 outside image
    // Reset pointers and variables that were used
    x1 = height - 2 * s_half;
    x2 = height - 1;
    y1 = width - 2 * s_half;
    y2 = width - 1;
    curr_raw = start_raw + row_size * (height - s_half) + double_density * (width - s_half);// pointer to raw image data of (height-s_half,width-s_half)
    //curr_intImg = _intImg + width * (height - s_half) + width -s_half;// pointer to integral image data of (geight-s_half,width-s_half)
    sum = 0;
    x2y2_intImg = _intImg + width * (height - 1) + width - 1;
    jumpUp = (x2-x1+1) * width;
    jumpLeft = y2-y1+1;
    curr_pixel = _colour + width * (height - s_half) + width - s_half;

    for(int i=height-s_half; i < height; ++i)
    {
        for(int j=width-s_half; j < width; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            count = (x2-x1+1)*(y2-y1+1);
            if ((*curr_raw * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            }
            curr_raw += double_density;
            ++curr_pixel;
            --jumpLeft;
            ++y1;
        }
        curr_raw += row_size - s_half * double_density;
        jumpUp -= width;
        ++x1;
        y1-=s_half;
        jumpLeft+=s_half;
        curr_pixel += width - s_half;
    }

    // loop 5 for x1 only is outside image
    // Reset pointers and variables that were used
    x1 = 1;
    x2 = s_half;
    y1 = 1;
    y2 = 2 * s_half + 1;
    curr_raw = start_raw + double_density * (s_half + 1);// pointer to raw image data of (0,s_half+1)
    //curr_intImg = _intImg + s_half + 1;// pointer to integral image data of (0,s_half+1)
    sum = 0;
    x2y2_intImg = _intImg + width * s_half + 2 * s_half + 1;
    jumpUp = (x2-x1+1) * width;
    jumpLeft = y2-y1+1;
    curr_pixel = _colour + s_half + 1;

    for(int i=0; i < s_half+1; ++i)
    {
        for(int j=s_half+1; j<width-s_half; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            count = (x2-x1+1)*(y2-y1+1);
            if ((*curr_raw * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            }
            curr_raw += double_density;
            ++curr_pixel;
            ++x2y2_intImg;
            ++y1;
            ++y2;
        }
        curr_raw += row_size - (width - 2 * s_half - 1) * double_density;
        jumpUp += width;
        ++x2;
        y1 = 1;
        y2 = 2 * s_half + 1;
        curr_pixel += 2 * s_half + 1;
        x2y2_intImg += 2 * s_half + 1;
    }

    // loop 6 for x2 only is outside image
    // Reset pointers and variables that were used
    x1 = height - 2 * s_half;
    x2 = height - 1;
    y1 = 1;
    y2 = 2 * s_half + 1;
    curr_raw = start_raw + row_size * (height - s_half) + double_density * (s_half + 1);// pointer to raw image data of (height-s_half,s_half+1)
    //curr_intImg = _intImg + width * (height - s_half) + s_half + 1;// pointer to integral image data of (height-s_half,s_half+1)
    sum = 0;
    x2y2_intImg = _intImg + width * (height - 1) + 2 * s_half + 1;
    jumpUp = (x2-x1+1) * width;
    jumpLeft = y2-y1+1;
    curr_pixel = _colour + width * (height - s_half) + s_half + 1;

    for(int i=height-s_half; i < height; ++i)
    {
        for(int j=s_half+1; j < width-s_half; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            count = (x2-x1+1)*(y2-y1+1);
            if ((*curr_raw * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            }
            curr_raw += double_density;
            ++curr_pixel;
            ++x2y2_intImg;
            ++y1;
            ++y2;
        }
        curr_raw += row_size - (width - 2 * s_half - 1) * double_density;
        jumpUp -= width;
        ++x1;
        y1 = 1;
        y2 = 2 * s_half + 1;
        curr_pixel += 2 * s_half + 1;
        x2y2_intImg -= width - 2 * s_half - 1;
    }

    // loop 7 for y1 only is outside image
    // Reset pointers and variables that were used
    x1 = 1;
    x2 = 2 * s_half + 1;
    y1 = 1;
    y2 = s_half;
    curr_raw = start_raw + row_size * (s_half + 1);// pointer to raw image data of (s_half+1,0)
    //curr_intImg = _intImg + width * (s_half + 1);// pointer to integral image data of (s_half+1,0)
    sum = 0;
    x2y2_intImg = _intImg + width * (2 * s_half + 1) + s_half;
    jumpUp = (x2-x1+1) * width;
    jumpLeft = y2-y1+1;
    curr_pixel = _colour + width * (s_half + 1);

    for(int i=s_half+1; i < height-s_half; ++i)
    {
        for(int j=0; j < s_half+1; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            count = (x2-x1+1)*(y2-y1+1);
            if ((*curr_raw * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            }
            curr_raw += double_density;
            ++curr_pixel;
            ++x2y2_intImg;
            ++y2;
            ++jumpLeft;
        }
        curr_raw += row_size - (s_half + 1) * double_density;
        jumpLeft -= s_half + 1;
        ++x1;
        ++x2;
        y2 = s_half;
        curr_pixel += width - s_half - 1;
        x2y2_intImg += width - s_half - 1;
    }

    // loop 8 for y2 only is outside image
    // Reset pointers and variables that were used
    x1 = 1;
    x2 = 2 * s_half + 1;
    y1 = width - 2 * s_half;
    y2 = width - 1;
    curr_raw = start_raw + row_size * (s_half + 1) + double_density * (width - s_half);// pointer to raw image data of (s_half+1,width-s_half)
    //curr_intImg = _intImg + width * (s_half + 1) + width - s_half;// pointer to integral image data of (s_half+1,width-s_half)
    sum = 0;
    x2y2_intImg = _intImg + width * (2 * s_half + 1) + width - 1;
    jumpUp = (x2-x1+1) * width;
    jumpLeft = y2-y1+1;
    curr_pixel = _colour + width * (s_half + 1) + width - s_half;

    for(int i=s_half+1; i < height-s_half; ++i)
    {
        for(int j=width-s_half; j < width; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            count = (x2-x1+1)*(y2-y1+1);
            if ((*curr_raw * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            }
            curr_raw += double_density;
            ++curr_pixel;
            ++y1;
            --jumpLeft;
        }
        curr_raw += row_size - s_half * double_density;
        ++x1;
        ++x2;
        y1 = width - 2 * s_half;
        x2y2_intImg += width;
        jumpLeft += s_half;
        curr_pixel += width - s_half;
    }

    // loop 9 for no-edge case
    // Reset pointers and variables that were used
    x1 = 1;
    x2 = 2 * s_half + 1;
    y1 = 1;
    y2 = 2 * s_half + 1;
    curr_raw = start_raw + row_size * (s_half + 1) + double_density * (s_half + 1);// pointer to raw image data of (s_half+1,s_half+1)
    //curr_intImg = _intImg + width *(s_half + 1) + s_half + 1;// pointer to integral image data of (s_half+1,width-s_half)
    sum = 0;
    x2y2_intImg = _intImg + width * (2 * s_half + 1) + 2 * s_half + 1;
    jumpUp = (2 * s_half + 1) * width;
    jumpLeft = 2 * s_half + 1;
    curr_pixel = _colour + width *(s_half + 1) + s_half + 1;
    count = (x2-x1+1)*(y2-y1+1);

    for(int i=s_half+1; i < height-s_half; ++i)
    {
        for(int j=s_half+1; j < width-s_half; ++j)
        {
            sum = *x2y2_intImg - *(x2y2_intImg - jumpLeft) - *(x2y2_intImg - jumpUp) + *(x2y2_intImg - jumpUp - jumpLeft);
            if ((*curr_raw * count) * 100 <= (sum * (100-t)))
            {
                *curr_pixel = cGREEN;
            } 
            else 
            {
                *curr_pixel = cWHITE;
            }
            curr_raw += double_density;
            ++curr_pixel;
            ++y1;
            ++y2;
            ++x2y2_intImg;
        }
        curr_raw += row_size - (width - 2 * s_half - 1) * double_density;
        ++x1;
        ++x2;
        y1 = 1;
        y2 = 2 * s_half + 1;
        x2y2_intImg += 2 * s_half + 1;
        curr_pixel += 2 * s_half + 1;
    }
    
    delete[] _intImg;


    if (do_body_part){
        // If there is nothing to look at, return.
        if(startStop.size() == 0)
            return;

        // The current point in the saliency image.
        Colour* saliencyPixel = _colour;

        // The next change point in the saliency image.
        Colour* saliencyEnd;

        // The block currently being looked at.
        unsigned int block = 0;

        // The start needs to be classified as robot.
        if(startStop[0] != 0)
        {
            saliencyEnd = _colour + startStop[0];

            // Run mark this group of pixels as robot grey.
            for(; saliencyPixel<saliencyEnd; ++saliencyPixel)
                *saliencyPixel = cBODY_PART;
        }

        // Classify the blocks.
        while(block < startStop.size()-1)
        {
            // Even blocks use standard classification.
            saliencyPixel = _colour + startStop[block];
            saliencyEnd = _colour + startStop[block+1];

            // Move to the next block.
            ++block;
            if(!(block < startStop.size()-1))
                break;

            // Odd blocks are just robot parts.
            saliencyPixel = _colour + startStop[block];
            saliencyEnd = _colour + startStop[block+1];
            for(; saliencyPixel < saliencyEnd; ++saliencyPixel)
                *saliencyPixel = cBODY_PART;

            // Move to the next block.
            ++block;
        }

        // Classify any trailing values as robot parts.
        saliencyPixel = _colour + (*(startStop.end()-1));
        saliencyEnd = _colour + bb.height() * bb.width();
        for(; saliencyPixel<saliencyEnd; ++saliencyPixel)
            *saliencyPixel = cBODY_PART;
    }
}

/**
 * Creates an edge image, where x values are x axis edges, y values are y axis
 * edges and positive edges are left low right high or top low bottom high.
 */

/*
 * Gets a new fovea at a different density to this fovea. This fovea will
 * handle memory management, such that the fovea lasts until next frame.
 */
Fovea* Fovea::getChildFovea(const BBox& bounding_box,
        const int density_to_raw, const bool top,
        const bool generate_fovea_colour, const int window_size, const int percentage)
{
    // Create the new fovea.
    Fovea* new_fovea = new Fovea(bounding_box, density_to_raw, top,
               generate_fovea_colour);

    // Record it for memory managment.
    child_fovea_.push_back(new_fovea);

    // Generate its saliency images.
    new_fovea->generate(*combined_frame_, window_size, percentage);

    // Return the newly generated fovea.
    return(new_fovea);
}
