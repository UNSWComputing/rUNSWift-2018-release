#ifndef PERCEPTION_VISION_REGION_H_
#define PERCEPTION_VISION_REGION_H_

#include <list>
#include <map>

#ifndef REGION_TEST
#include "types/BBox.hpp"
#include "types/CombinedFovea.hpp"
#else
#define TOP_IMAGE_COLS 1024
#define BOT_IMAGE_COLS 1024
#define TOP_SALIENCY_DENSITY 8
#define BOT_SALIENCY_DENSITY 8
#include "Fovea.hpp"
#include "BBox.hpp"
#endif

#define DENSITY_MAINTAIN 0
#define DENSITY_INCREASE 1
#define DENSITY_DECREASE 2

/**
 * Enumerated type for certainty map
 * add more as required
 */
enum RegionIType {
    BALL,
    GOAL,
    ROBOT,
    N_TYPES     // keep this last
};

/**
 * Region is in essence a bounding box describing a region within the full fovea
 * There are a few key terms to get your head around:
 *  - region-space: a space where (x,y) refer to the  a bounding box and density
 *  - raw-space: a space where (x,y) refer to the absolute pixel positions on the raw
 *      image and the density is 1 because the image is at full sampling
 * These relationships can be summarised by:
 *  region-space.x = raw-space.x + region-space.x_offset + region-space.x * region-space.density
 *  region-space.y = raw-space.y + region-space.y_offset + region-space.y * region-space.density
 */
class RegionI {

public:

    //////////////////////////////////
    // Iterator sub class
    //////////////////////////////////

    class iterator {
    public:
        // Boilerplate typedefs for iterator
        typedef iterator self_type;
        typedef std::forward_iterator_tag iterator_category;
        typedef int different_type;

        /**
         * Constructor
         * @x raw-relative x coordinate
         * @y raw-relative y coordinate
         * @region region that we are iterating over
         */
        iterator(const RegionI* const region, int pos, int x_offset,
                 int y_offset, int x_density, int y_density, int x_total_width,
                                   int x_region_width)
           : region_(region),
           pos_(pos),
           x_offset_(x_offset),
           y_offset_(y_offset),
           x_density_(x_density),
           y_density_(y_density),
           x_total_width_(x_total_width),
           x_region_width_(x_region_width),
           x_region_width_rel_(x_region_width / x_density),
           row_begin_(x_offset_ + y_offset*x_total_width_ - x_density),
           row_end_(x_offset_ + y_offset*x_total_width_ + x_region_width_),
           n_raw_steps_to_jump_(x_total_width * y_density) {}

        /**
         * Prefix overload for ++
         * @return a reference to *this
         */
        inline self_type operator++() {
            self_type i = *this;
            iterate_();
            return i;
        }

        /**
         * Postfix overload for ++
         * @return a copy of *this prior to increment
         */
        inline self_type& operator++(int) {
            iterate_();
            return *this;
        }

        /**
         * Prefix overload for --
         * @return a reference to *this
         */
        inline self_type operator--() {
            self_type i = *this;
            iterate_back_();
            return i;
        }

        /**
         * Postfix overload for --
         * @return a copy of *this prior to increment
         */
        inline self_type& operator--(int) {
            iterate_back_();
            return *this;
        }

        /**
         * Jumps this iterator down by exactly one row.
         */
        inline self_type& next_row()
        {
            row_begin_ += n_raw_steps_to_jump_;
            row_end_ += n_raw_steps_to_jump_;
            pos_ += n_raw_steps_to_jump_;
            return *this;
        }

        /**
         * Jumps this iterator down by exactly one row.
         */
        inline self_type& last_row()
        {
            row_begin_ -= n_raw_steps_to_jump_;
            row_end_ -= n_raw_steps_to_jump_;
            pos_ -= n_raw_steps_to_jump_;
            return *this;
        }

        /**
         * Get the region-relative x value that this iterator is up to
         * @return region-relative x value that this iterator is up to
         */
        inline const int x() { return (xAbs() - x_offset_) / x_density_; }

        /**
         * Get the region-relative y value that this iterator is up to
         * @return region-relative y value that this iterator is up to
         */
        inline const int y() { return (yAbs() - y_offset_) / y_density_; }

        /**
         * Get the image-relative x value that this iterator is up to
         *  this applies to the full resolution - raw image.
         * @return image-relative x value that this iteraotr is up to
         */
        inline const int xAbs() { return pos_ % x_total_width_; }

        /**
         * Get the image-relative y value that this iterator is up to
         *  this applies to the full resolution - raw image.
         * @return image-relative y value that this iteraotr is up to
         */
        inline const int yAbs() { return (int)(pos_ / x_total_width_); }

        /**
         * Operator==
         */
        inline bool operator==(const self_type& rhs) { return pos_ == rhs.pos_; }

        /**
         * Operator!=
         */
        inline bool operator!=(const self_type& rhs) { return !operator==(rhs); }

        /**
         * Operator<
         */
        inline bool operator<(const self_type& rhs) {
            return pos_ < rhs.pos_;
        }

    protected:

        inline const int getLinearPos_() {
            return pos_;
        }

        inline const int getLeftPos_() {
            return pos_-x_density_;
        }

        inline const int getAbovePos_() {
            return pos_-x_total_width_*x_density_*y_density_;
        }

        inline const int getBelowPos_() {
            return pos_+x_total_width_*x_density_*y_density_;
        }

        inline const int getRightPos_() {
            return pos_+x_density_;
        }


        inline const RegionI* const getRegion_() {
            return region_;
        }

    private:

        inline void iterate_() {
            pos_ += x_density_;
            if (pos_ == row_end_) {
                row_begin_ += n_raw_steps_to_jump_;
                row_end_ += n_raw_steps_to_jump_;
                pos_ = row_end_ - x_region_width_;
            }
        }

        inline void iterate_back_() {
            pos_ -= x_density_;
            if (pos_ == row_begin_) {
                row_begin_ -= n_raw_steps_to_jump_;
                row_end_ -= n_raw_steps_to_jump_;
                pos_ = row_end_ - x_density_;
            }
        }

        // The linear position that our iterator is up to, the x/y densities,
        //  as well as the total width of columns, the number of columns within
        //  the region box, and the x, y offset from the top-left corner.
        const RegionI* const region_;
        int pos_;
        int x_offset_;
        int y_offset_;
        int x_density_;
        int y_density_;
        int x_total_width_;
        int x_region_width_;
        int x_region_width_rel_;

        // For region iteration
        int row_begin_;
        int row_end_;
        int n_raw_steps_to_jump_;
    };

    class iterator_raw : public iterator {
    public:

        iterator_raw(const RegionI* const region, int pos, int x_offset, int y_offset, int density,
                 int x_total_width, int x_region_width) :
            iterator(region, pos, x_offset*2, y_offset, density*2, density,
                 x_total_width*2, x_region_width*2) {}

        /**
         * Get the raw pixel values at this pixel iteration point
         * @return a pointer to the raw YUV value
         */
        inline const
        #ifdef REGION_TEST
        int
        #else
        uint8_t*
        #endif
        raw() { return getRegion_()->getPixelRaw_(getLinearPos_()); }

        /**
         * Get the raw pixel Y,U and V values at this pixel iteration point
         * @return the Y, U or V value
         */
        #ifdef REGION_TEST
        inline int
        #else
        inline uint8_t
        #endif
        getY() {return(*raw());}

        #ifdef REGION_TEST
        inline int
        #else
        inline uint8_t
        #endif
        getU() {return(*(raw()+(1+2*(getLinearPos_() & 1)))); }

        #ifdef REGION_TEST
        inline int
        #else
        inline uint8_t
        #endif
        getV() {return(*(raw()+(1+2*(getLinearPos_() & 0)))); }

    };

    class iterator_fovea : public iterator {
    public:

        iterator_fovea(const RegionI* const region, int pos, int x_offset, int y_offset, int density,
                 int x_total_width, int x_region_width) :
            iterator(region, pos, x_offset, y_offset, density, density,
                 x_total_width, x_region_width) {}

        /**
         * Get the colour enum at this pixel iteration point
         * @return colour enum
         */
    const
        #ifdef REGION_TEST
        inline int
        #else
        inline Colour
        #endif
        colour() { return getRegion_()->getPixelColour_(getLinearPos_()); }

        #ifdef REGION_TEST
        inline int
        #else
        inline Colour
        #endif
        colourAbove() { return getRegion_()->getPixelColour_(getAbovePos_()); }

        #ifdef REGION_TEST
        inline int
        #else
        inline Colour
        #endif
        colourBelow() { return getRegion_()->getPixelColour_(getBelowPos_()); }

        #ifdef REGION_TEST
        inline int
        #else
        inline Colour
        #endif
        colourLeft() { return getRegion_()->getPixelColour_(getLeftPos_()); }

        #ifdef REGION_TEST
        inline int
        #else
        inline Colour
        #endif
        colourRight() { return getRegion_()->getPixelColour_(getRightPos_()); }
    };

    /**
     * Empty constructor
     *  Used in serialisation.
     */
    RegionI() {}

    /**
     * Standard Constructor
     *  Produces a region-space
     * @bounding_box Bounding box reflecting the top-left and bottom-right points in raw-space
     * @is_top_camera True if this region will reflect the top camera, False if bottom camera
     * @foveae The relevant top or bottom  fovea containing meta on the raw image
     * @density the ZOOM_IN density of this region-space with respect to raw-space
     */
    RegionI(BBox bounding_box, bool is_top_camera, Fovea& fovea, int density);

    /**
     * Copy-Constructor
     *  Produces a region2-space from a region1-space
     * @region region1-space object
     * @box
     * @rel_density the magnitude of the density change between region1-space and region2-space
     *   such that region2-density = region1-density * rel_density if ZOOMING IN or ;
     *   such that region2-density = region1-density / rel_density if ZOOMING OUT or ;
     * @rel_density_multiplier a #define on whether we are ZOOMING IN or ZOOMING OUT
     * @regenerate_fovea_colour whether the fovea colour classification data should be regenerated at the new density.
     * @window_size adaptive thresholding window size
     * @percentage adaptive thresholding percentage
     */
    RegionI(const RegionI& region, BBox box, int rel_density = 1,
            int rel_density_multiplier = DENSITY_MAINTAIN,
            const bool regenerate_fovea_colour = false,
            int window_size = UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
            int percentage = UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE)
                           : is_top_camera_(region.is_top_camera_),
                             this_fovea_(region.this_fovea_),
                             raw_total_width_(region.raw_total_width_),
                             raw_to_fovea_density_(region.raw_to_fovea_density_)
    {
        init_(region, BBox(Point(box.a.x(), box.a.y()), Point(box.b.x(),
            box.b.y())), rel_density, rel_density_multiplier,
            window_size, percentage,
            regenerate_fovea_colour);
    }

    /**
     * Density change Copy-Constructor
     *  Produces a region2-space from a region1-space
     * @region region1-space object
     * @rel_density the magnitude of the density change between region1-space and region2-space
     *   such that region2-density = region1-density * rel_density if ZOOMING IN or ;
     *   such that region2-density = region1-density / rel_density if ZOOMING OUT or ;
     * @rel_density_multiplier a #define on whether we are ZOOMING IN or ZOOMING OUT
     * @regenerate_fovea_colour whether the fovea colour classification data should be regenerated at the new density.
     * @window_size adaptive thresholding window size
     * @percentage adaptive thresholding percentage
     */
    RegionI(const RegionI& region, int rel_density,
                     int rel_density_multiplier = DENSITY_MAINTAIN,
                     const bool regenerate_fovea_colour = false,
                     int window_size = UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
                     int percentage = UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE)
                           : is_top_camera_(region.is_top_camera_),
                             this_fovea_(region.this_fovea_),
                             raw_total_width_(region.raw_total_width_),
                             raw_to_fovea_density_(region.raw_to_fovea_density_)
    {
        init_(region, BBox(Point(0,0), Point(region.getCols(),
                  region.getRows())), rel_density, rel_density_multiplier,
                  window_size, percentage,
                  regenerate_fovea_colour);
    }

    /**
     * Overload: Copy-assignment
     */
    RegionI& operator=(const RegionI&);

    /**
     * Standard copy constructor
     */
    RegionI(const RegionI& region)
    {
        *this = region;
    }

    /**
     * For a region-relative (x,y) pixel, return the
     *  pointer to the raw YUV value
     * @x region-relative x coordinate
     * @y region-relative x coordinate
     * @return pointer to the raw YUV value of the pixel
     */
    inline const
    #ifdef REGION_TEST
    int
    #else
    uint8_t*
    #endif
    getPixelRaw(int x, int y) const {
        return getPixelRaw_(getLinearPosFromXYRaw_(x, y));
    }

    /**
     * For a region-relative (x,y) pixel, return the
     *  reference to the colour enum type
     * @x region-relative x coordinate
     * @y region-relative x coordinate
     * @return reference to the colour enum type of the pixel
     */
    inline const
    #ifdef REGION_TEST
    int
    #else
    Colour
    #endif
    getPixelColour(int x, int y) const {
        return getPixelColour_(getLinearPosFromXYFovea_(x, y));
    }

    /**
     * Returns true if the underlying image that this
     *  region is referencing is the top camera. False if not
     * @return True if top camera, false if bottom camera
     */
    inline const bool isTopCamera() const { return is_top_camera_; }

    /**
     * Get the number of columns this region represents in region-space
     * @return the number of columns this region represents in region-space
     */
    inline const int getCols() const { return (n_raw_cols_in_region_ / density_to_raw_); }

    /**
     * Get the number of rows this region represents in region-space
     * @return the number of rows this region represents in region-space
     */
    inline const int getRows() const { return (n_raw_rows_in_region_ / density_to_raw_); }

    /**
     * Get the density of this region-space with respect to the raw-space
     * @return density of this region-space with respect to the raw-space
     */
    inline const int getDensity() const { return density_to_raw_; }

    /**
     * Get the density of this region-space with respect to the raw-space
     * @return density of this region-space with respect to the raw-space
     */
    inline const int getFoveaDensity() const { return raw_to_fovea_density_; }

    /**
     * Get the bounding box that represents the top-left and bottom-right endpoints
     *  of this region-space
     * @return bounding box for the top-left and bottom-right endpoints  of this region-space
     */
    inline BBox getBoundingBoxRel() const { return bounding_box_rel_; }

    /**
     * Get the bounding box that represents the top-left and bottom-right endpoints
     *  of this region-space
     * @return bounding box for the top-left and bottom-right endpoints  of this region-space
     */
    inline BBox getBoundingBoxFovea() const { return bounding_box_fovea_; }

    /**
     * Get the bounding box that represents the top-left and bottom-right endpoints
     *  of this region-space
     * @return bounding box for the top-left and bottom-right endpoints  of this region-space
     */
    inline BBox getBoundingBoxRaw() const { return bounding_box_raw_; }

    /**
     * Returns iterator to the beginning of the region-space for raw values
     * @return iterator to the beginning of the region-space for raw values
     */
    iterator_raw begin_raw() const;

    /**
     * Returns iterator to the beginning of the region-space for fovea values
     * @return iterator to the beginning of the region-space for fovea values
     */
    iterator_fovea begin_fovea() const;

    /**
     * Returns iterator to one past the end of the region-space for raw values
     * @return iterator to one past the end of the region-space for raw values
     */
    inline iterator_raw end_raw() const {
        return iterator_raw(
            this,
            getLinearPosFromXYRaw_(0, n_raw_rows_in_region_),
            -1,-1,-1,-1,-1
        );
    }

    /**
     * Returns iterator to one past the end of the region-space for fovea values
     * @return iterator to one past the end of the region-space for fovea values
     */
    inline iterator_fovea end_fovea() const {
        return iterator_fovea(
            this,
            getLinearPosFromXYFovea_(0,
            n_raw_rows_in_region_/raw_to_fovea_density_),
            -1,-1,-1,-1,-1
        );
    }

    /**
     * Returns iterator to a point within a region for raw pixel data access.
     * @return iterator to a point within a region.
     */
    iterator_raw get_iterator_raw(Point point) const {
        return iterator_raw(
            this,
            getLinearPosFromXYRaw_(point.x(), point.y()),
            x_offset_raw_,
            y_offset_raw_,
            density_to_raw_,
            raw_total_width_,
            n_raw_cols_in_region_
        );
    }

    /**
     * Returns iterator to a point within a region for fovea data access.
     * @return iterator to a point within a region.
     */
    iterator_fovea get_iterator_fovea(Point point) const
    {
        return iterator_fovea(
            this,
            getLinearPosFromXYFovea_(point.x(), point.y()),
            bounding_box_fovea_.a.x(),
            bounding_box_fovea_.a.y(),
            (density_to_raw_/raw_to_fovea_density_),
            fovea_width_,
            (n_raw_cols_in_region_/raw_to_fovea_density_)
        );
    }

    /**
     * Returns a pointer to a underlying fovea. This is used for blackboard to
     * access the colour saliency arrays.
     */
    inline const Fovea* getInternalFovea() const {
        return this_fovea_;
    }

    /**
     * For vatnao
     */

    void setFovea(Fovea &fovea) {
        this_fovea_ = &fovea;
        raw_to_fovea_density_ = fovea.getDensity();
        bounding_box_fovea_.a = bounding_box_raw_.a/fovea.getDensity();
        bounding_box_fovea_.b = bounding_box_raw_.b/fovea.getDensity();
        fovea_width_ = this_fovea_->getBBox().width();
    };

    /*
     * Serialisation of regions so that they can be sent to offnao and other
     * remote programs.
     */
    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
        ar & is_top_camera_;
        ar & bounding_box_rel_;
        ar & bounding_box_fovea_;
        ar & bounding_box_raw_;
        ar & n_raw_cols_in_region_;
        ar & n_raw_rows_in_region_;
        ar & density_to_raw_;
        ar & y_offset_raw_;
        ar & x_offset_raw_;
        ar & raw_total_width_;
        ar & raw_to_fovea_density_;
        ar & fovea_width_;
    }

private:

    /**
     * Initialises a region with the specified information.
     */
    void init_(const RegionI& region, BBox box, int rel_density,
        int rel_density_multiplier, const int window_size, const int percentage,
        const bool regenerate_fovea_colour);

    /**
     * For a region-relative linear position (2d scan position
     *  from top left to bottom right, return the
     *  pointer to the raw YUV value
     * @linear_pos 2d scan position from top left to bottom right
     * @return pointer to the raw YUV value of the pixel
     */
    inline const
    #ifdef REGION_TEST
    int
    #else
    uint8_t*
    #endif
    getPixelRaw_(int linear_pos) const {
        return this_fovea_->getRawYUV(linear_pos);
    }

    /**
     * For a region-relative linear position (2d scan position
     *  from top left to bottom right, return the
     *  reference to the colour enum type
     * @linear_pos 2d scan position from top left to bottom right
     * @return reference to the colour enum type of the pixel
     */
    inline const
    #ifdef REGION_TEST
    int
    #else
    Colour
    #endif
    getPixelColour_(int linear_pos) const {
        return this_fovea_->getFoveaColour(linear_pos);
    }

    // TODO
    inline const int getLinearPosFromXYRaw_(int x, int y) const {
        return (x_offset_raw_ + x*density_to_raw_)*2 + raw_total_width_ * (y_offset_raw_ + y*density_to_raw_)*2;
    }

    // TODO
    inline const int getLinearPosFromXYFovea_(int x, int y) const {
        return(((x*density_to_raw_)/raw_to_fovea_density_ +
                bounding_box_fovea_.a.x()) +
               ((y*density_to_raw_)/raw_to_fovea_density_ +
                bounding_box_fovea_.a.y()) * this_fovea_->getBBox().width());
    }

    bool is_top_camera_;

    Fovea* this_fovea_;

    // Number of columns this region covers in terms of raw-space
    int n_raw_cols_in_region_;
    int n_raw_rows_in_region_;

    // Density of region-space with respect to raw-space
    int density_to_raw_;

    // (x,y) offsets of the top-left point of this region-space with respect to raw-space
    int y_offset_raw_;
    int x_offset_raw_;

    // Other information needed for class to operate
    int raw_total_width_;
    int raw_to_fovea_density_;

    // The width of the fovea. Needed for serialisation to not require the fovea.
    int fovea_width_;

    // For checking if the colour array has been correctally generated for the region
    bool colour_array_set_;

    BBox bounding_box_rel_;
    BBox bounding_box_fovea_;
    BBox bounding_box_raw_;

};

#endif
