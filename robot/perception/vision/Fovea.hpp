#ifndef PERCEPTION_VISION_FOVEA_H_
#define PERCEPTION_VISION_FOVEA_H_

#include "perception/vision/colour/ColourClassifierInterface.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "types/CombinedFrame.hpp"
#include "types/Point.hpp"
#include "types/BBox.hpp"


class Fovea {

public:

    /**
     * Creates a new fovea. bb is the bounds of the fovea in fovea density
     * pixels, density is the density of the fovea and top is whether this
     * fovea is in the top or bottom image. colour determine
     * whether the colourimages are generated respectively.
     */
    Fovea(BBox bb, int density, bool top, bool colour) :
        bb(bb), density(density), top(top), hasColour(colour),
        _colour(colour  ? new Colour[bb.width() * bb.height()] : NULL),
                                                       width(bb.b[0]-bb.a[0]) {}

    /**
     * Free the _colour arrays.
     */
    ~Fovea();

    /**
     * Updates the fovea to use the data in combined_frame and
     * adaptive thresholding window size and percentage
     */
    void generate(const CombinedFrame& combined_frame, 
                  const int win_size = DEFAULT_ADAPTIVE_THRESHOLDING_WINDOW_SIZE,
                  const int perc = DEFAULT_ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT,
                  bool do_body_part = false);

    Fovea& operator=(const Fovea& f) { return *this; }
    const Fovea& operator=(const Fovea& f) const { return *this; }

    /**
     * Returns a pointer to the raw pixel found at x, y within the fovea. Note
     * that the pixel pointed to will be of the form YU_V if (foveaYOffset+y)*
     * foveaWidth*foveaDensity*2 + (foveaXOffset+x)*foveaDensity*2 is even, and
     * UYV otherwise. The pointer will be to the Y value.
     */
    inline const uint8_t* getRawYUV(int x, int y) const
    {
        return(_rawImage+(bb.a[1]+y)*width*density*2 + (bb.a[0]+x)*density*2);
    }

    /**
     * Returns a pointer to the raw pixel found at linearPos, where linearPos is
     * a value equivalent to (foveaYOffset+y)*foveaWidth*foveaDensity*2 +
     * (foveaXOffset+x)*foveaDensity*2 and x and y are a 2D coordinate within
     * the fovea.
     */
    inline const uint8_t* getRawYUV(int linearPos) const
    {
        return(_rawImage+linearPos);
    }

    /**
     * Returns the colour classification of the requested pixel, relative to the
     * fovea bounds. Must be inside the fovea bounds.
     */
    inline const Colour getFoveaColour(int x, int y) const
    {
        return(_colour[x + y*width]);
    }

    /**
     * Get the fovea colour, where linearPos is y*foveaWidth+x and x and y are a
     * 2D coordinate within the fovea.
     */
    inline const Colour getFoveaColour(int linearPos) const
    {
        return(_colour[linearPos]);
    }

    /**
     * Convert image coord to fovea coord. Coordinates are such that the bottom
     * image's top row is at y = TOP_IMAGE_ROWS.
     */
    inline Point mapImageToFovea(Point p) const
    {
        if (!top) p.y() -= TOP_IMAGE_ROWS;
        return (p / density) - bb.a;
    }

    /**
     * Convert fovea coord to image coord. Coordinates are such that the bottom
     * image's top row is at y = TOP_IMAGE_ROWS.
     */
    inline Point mapFoveaToImage(Point p) const
    {
        if (!top) {
            Point temp = (p + bb.a) * density;
            temp.y() += TOP_IMAGE_ROWS;
            return temp;
        }
        return (p + bb.a) * density;
    }

    /**
     * Gets the colour array for blackboard
     */
     const Colour* getInternalColour() const {
         return _colour;
     }

    /**
     * Gets BBox for fieldLineDetection
     * Gets top for fieldLineDetection
     */

     const BBox getBBox() const { return bb; }
     const bool getTop() const { return top; }
     const int getDensity() const {return density; }

    /**
     * Gets the window size used in adaptive thresholding
     */

     const int getWindowSize() const {
         return windowSize;
     }

    /**
     * Gets the percentage used in adaptive thresholding
     */

     const int getPercentage() const {
         return percentage;
     }

    /**
     * Sets the window size used in adaptive thresholding
     */

     const void setWindowSize(int winSize) {
         windowSize = winSize;
     }

    /**
     * Sets the percentage used in adaptive thresholding
     */

     const void setPercentage(int per) {
        percentage = per;
    }


     /*
      * Gets a new fovea at a different density to this fovea. This fovea will
      * handle memory management, such that the fovea lasts until next frame.
      */
     Fovea* getChildFovea(const BBox& bounding_box,
         const int density_to_raw, const bool top,
         const bool generate_fovea_colour, const int window_size, const int percentage);
                                           

protected:
    /**
     * Gets a pointer to the first item in the colour array.
     * Temporary, for future speed comparison with iterator.
     */
    const Colour* getColourArray();

    /**
     * Gets a pointer to the first item in the raw image.
     * Temporary, for future speed comparison with iterator.
     */
    const uint8_t* getRawPixelArray();

private:

    // The bounds of the fovea in fovea density pixels.
    const BBox           bb;

    // The number of raw image pixels per classified pixel on each axis for this
    // fovea.
    const int            density;

    // Whether this fovea is in the top image.
    const bool           top;

    // The window size to be used by the adaptive thresholding algorithm
    int                  windowSize;

    // The percentage value to be used by the adaptive thresholding algorithm
    int                  percentage;

    // Whether this fovea has a colour image.
    const bool           hasColour;

    // The colour classified image.
     Colour      *const _colour;

    // The width of the fovea in density pixels.
    const int            width;

    // A pointer to the upper left pixel of the image this fovea is in.
    const uint8_t *     _rawImage;

    // These need to be available for generating child foveas.
    const CombinedFrame* combined_frame_;
    const ColourClassifier* colour_classifier_;

    // All this fovea's child fovea.
    std::vector<Fovea*> child_fovea_;

    /**
     * Creates a colour image. Optimised for using the top image.
     */
    void makeColourTop_(const CombinedFrame& combined_frame,
        const ColourClassifier& colour_classifier,
        const std::vector<int>& startStop,
                               const std::vector<const uint8_t*>& startStopRaw);

    /**
     * Creates a colour image. Optimised for using the bottom image.
     */
    void makeColourBot_(const CombinedFrame& combined_frame,
        const ColourClassifier& colour_classifier,
        const std::vector<int>& startStop,
                               const std::vector<const uint8_t*>& startStopRaw);

    /**
     * Creates a binary image.
     */
    void makeBinary_(const std::vector<int>& startStop, bool do_body_part);

    /**
     * Returns the raw grey value of the requested pixel, relative to the fovea
     * bounds. Must be inside the fovea bounds.
     */
    inline const uint8_t& getRawGrey_(int x, int y) const
    {
        static int imageCols;
        imageCols = top? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
        return (_rawImage[x*density*2 + y*imageCols*density*2]);
    }

    /*
     * Translates the y axis robot part stop array to a linear start stop array.
     */
    std::pair<std::vector<int>*, std::vector<const uint8_t*>* > getStartStop_()
                                                                          const;
};

#endif
