#ifndef PERCEPTION_VISION_COLOUR_GREENYUVCLASSIFIER_H_
#define PERCEPTION_VISION_COLOUR_GREENYUVCLASSIFIER_H_

#include "perception/vision/other/otsu.hpp"
#include "perception/vision/other/YUV.hpp"
#include "perception/vision/colour/ColourClassifierInterface.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "types/VisionInfoIn.hpp"
#include "types/JointValues.hpp"
#include <list>
#include <cmath>

// NNMC dependencies
#include <bzlib.h>

#define SAMPLESIZE 30
#define Y_BITSHIFT 1
#define Y_MAX_POW 7
#define U_BITSHIFT 1
#define U_MAX_POW 7
#define V_BITSHIFT 1
#define V_MAX_POW 7

#define Y_RANGE 256
#define U_RANGE 256
#define V_RANGE 256

#define INITIAL_MAX_COVERAGE_TOP 0.8
#define INITIAL_MAX_COVERAGE_BOT 0.8

struct YUVTriple {
    int y_, u_, v_;

    YUVTriple() {}
    YUVTriple(int y, int u, int v) : y_(y), u_(u), v_(v) {}
};

typedef enum {
    ROBERTS_CROSS = 0,
    SOBEL = 1,
    NAIVE = 2,
    YUV_ISOTROPIC = 3
} Gradient_Operator;

class GreenYUVClassifier : public ColourClassifier {
friend class CalibrationTab;
public:
    GreenYUVClassifier(bool run_colour_calibration,
        bool load_nnmc,
        bool save_nnmc,
        std::string nnmc_filename);

    /*
     * Classify first sees if the pixel isGreen. If not, then it sees if the pixel isWhite.
     * If not, then the pixel is classified as cBACKGROUND
     */
    inline Colour classifyTop(const uint8_t *pixel) const {
        uint8_t y = 0;
        uint8_t u = 0;
        uint8_t v = 0;
        if ((size_t)pixel & 0x2) {
        //YV_U
            y = pixel[0];
            v = pixel[1];
        // _
            u = pixel[3];
        } else {
        //YU_V
            y = pixel[0];
            v = pixel[3];
        // _
            u = pixel[1];
        }

        return (Colour) nnmc_[((v >> V_BITSHIFT) << (Y_MAX_POW + U_MAX_POW)) | 
            ((u >> U_BITSHIFT) << Y_MAX_POW) | (y >> Y_BITSHIFT)];
    }

    inline Colour classifyBot(const uint8_t *pixel) const {
    // Repeat of top since we only have one calibration
        uint8_t y = 0;
        uint8_t u = 0;
        uint8_t v = 0;
        if ((size_t)pixel & 0x2) {
        //YV_U
            y = pixel[0];
            v = pixel[1];
        // _
            u = pixel[3];
        } else {
        //YU_V
            y = pixel[0];
            v = pixel[3];
        // _
            u = pixel[1];
        }

        return (Colour) nnmc_[((v >> V_BITSHIFT) << (Y_MAX_POW + U_MAX_POW)) | 
            ((u >> U_BITSHIFT) << Y_MAX_POW) | (y >> Y_BITSHIFT)];
    }

    void sampleImageScanLines(const uint8_t* image, bool top, int n_rows, int n_cols,
        const VisionInfoIn& info_in, int stepsize);

    /*
     * fillPoints iterates through our 3d histogram and classifies any YUVTriples that lie between
     * any green-classified YUVTriples as green
     *
     * Assumption here is that any YUVTriple between any two green-classified YUVTriple is green
     * Our histogram is a "solid"
     */

    void fillPoints(bool isGreen);

    // Load the classification result from file
    void loadNnmc(std::string filename);

    // Update colour classification from loaded nnmc
    void updateColours();
    
    int calculate_gradient(const uint8_t* image, int y_pos, int x_pos, int n_cols,
        Gradient_Operator g, 
        uint8_t (*getYUV)(const uint8_t*, int, int, int),
        int stepsize);

    void draw_edge(const uint8_t* image, int n_rows, int n_col, Gradient_Operator g);
 
    // Cache the result
    void saveClassification(void); 

    // Save the classification result to file 
    void saveNnmc(std::string filename);

    void resetNnmc();

    int calculate_threshold(const uint8_t* image, int n_rows, int n_cols, Gradient_Operator grad_op,
        uint8_t (*getYUV)(const uint8_t*, int, int, int));

    uint8_t nnmc_[(Y_RANGE >> Y_BITSHIFT) * (U_RANGE >> U_BITSHIFT) *
        (V_RANGE >> V_BITSHIFT)];
    
private:
    std::vector< std::vector < std::vector <int> > > yuv_counts_;
    std::vector< std::vector < std::vector <bool> > > is_white_;
    std::vector< std::vector < std::vector <bool> > > is_green_;
    std::vector< std::vector < std::vector <bool> > > is_black_;

    long long y_sum_;
    int y_sampled_;
    int y_mean_;
    bool first_sample_;
    int total_count_;
    int max_peak_count_;

    // Command line arguemtn configs
    bool run_colour_calibration_;
    bool save_nnmc_;

    bool nnmc_loaded_;

    // Create a list of candidate bins
    std::list< YUVTriple > YUV_candidate_list_;

    /*
     * From a particular y value down (for the top frame, we use getTopFrameStartScan to calculate 
     * this particular y value) we generate a histogram on bitshifted YUV values
     * (y >> Y_BITSHIFT) (u >> U_BITSHIFT) (v >> V_BITSHIFT)
     *
     * (Bitshifting is done to reduce our memory profile, but also improving the efficiency of our
     * algorithm.)
     *
     * With this histogram, we call expandVerticalPeak to generate our definition of green
     */ 

    void sampleImage(const uint8_t* image, bool top, int n_rows, int n_cols,
        const VisionInfoIn& info_in);

    double getCoverage(void) ;

    /*
     * Given a particular head_yaw, we create a conservative estimate (lower than accurate) of the
     * field boundary so that when we call sampleImage, our assumption (that x% of the image is green
     * holds true)
     * 
     * The idea is that if we aren't looking straigh ton, the scan should start lower down in the frame 
     * since the side field lines are in view
     */

    double getTopFrameStartScan(double head_yaw);

    /*
     * With this histogram, we assume INITIAL_MAX_COVERAGE_TOP / INITIAL_MAX_COVERAGE_BOT % of the 
     * sampled frame is green so starting from either the peak (if this is the first sample, or we wish
     * to start from a new peak (new_peak == TRUE)), we classify neighbours as green, giving preference
     * to bins with higher frequencies.
     *
     * We continue doing this until we reach the required level of coverage
     * Occasionally we run out of nearest neighbour bins and so we may call expandVerticalPeak with 
     * new_peak = True if we observe a green that we've not seen before
     */

    double expandVerticalPeak(double coverage, bool new_peak);

    /*
     * Given a YUVTriple, returns a vector of adjacent (differ in y, u and v by at most 1) YUVTriples
     */

    std::vector< YUVTriple > getBinNeighbours(const YUVTriple &yuv);

    /*
     * Checks to see if the robot is walking
     * Used for lab testing when we aren't using a ready skill behaviour with the appropriate head
     * turns
     */

    bool amWalking(const ActionCommand::All &commands);

    // Add y u v combination to classification with sphere of radius "radius"
    void classifyYUV(uint8_t y, uint8_t u, uint8_t v, Colour colour, int radius);

    inline bool isGreen(uint8_t y_val, uint8_t u_val, uint8_t v_val, bool top) const {
        return is_green_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT];
    }

    inline bool isBlack(uint8_t y_val, uint8_t u_val, uint8_t v_val) const {
        return is_black_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT];
    }
    
    inline bool isWhite(uint8_t y_val, uint8_t u_val, uint8_t v_val) const {
        return is_white_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT];



        /*
         * White pixels should have a high y value, and
         * be in the center area of the u-v plane -
         * We will use the lp1 definition of the norm to check this.
         */

        if (y_val <= y_mean_) return false;

        int lp1_norm = abs(u_val - 128) + abs(v_val - 128);

        return (lp1_norm < y_val - 64);
    }
};

#endif
