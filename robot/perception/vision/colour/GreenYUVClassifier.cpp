#include "perception/vision/colour/GreenYUVClassifier.hpp"

// Consider just adjacent neighbours (not diagonal)
static const int di[] = { 0, 0, 0, 0,-1, 1};
static const int dj[] = { 0, 0,-1, 1, 0, 0};
static const int dk[] = {-1, 1, 0, 0, 0, 0};

//static const std::string nnmc_filename = "/home/nao/data/green_yuv_classifier.nnmc.bz2";
static const std::string nnmc_filename_tmp = "/home/nao/data/green_yuv_classifier_tmp.nnmc.bz2";

static const char bzip_magic[] = {'B', 'Z', 'h'};

//static const int di[] = {-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//static const int dj[] = {-1,-1,-1, 0, 0, 0, 1, 1, 1,-1,-1,-1, 0, 0, 1, 1, 1,-1,-1,-1, 0, 0, 0, 1, 1, 1};
//static const int dk[] = {-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1};

GreenYUVClassifier::GreenYUVClassifier(bool run_colour_calibration,
    bool load_nnmc,
    bool save_nnmc,
    std::string nnmc_filename) : run_colour_calibration_(run_colour_calibration), save_nnmc_(save_nnmc) {
    yuv_counts_ = std::vector< std::vector < std::vector <int> > > (Y_RANGE >> Y_BITSHIFT,
                    std::vector < std::vector <int> > (U_RANGE >> U_BITSHIFT, 
                        std::vector <int> (V_RANGE >> V_BITSHIFT, 0)));

    is_white_ = std::vector< std::vector < std::vector <bool> > > (Y_RANGE >> Y_BITSHIFT,
                    std::vector < std::vector <bool> > (U_RANGE >> U_BITSHIFT, 
                        std::vector <bool> (V_RANGE >> V_BITSHIFT, false)));

    is_green_ = std::vector< std::vector < std::vector <bool> > > (Y_RANGE >> Y_BITSHIFT,
                    std::vector < std::vector <bool> > (U_RANGE >> U_BITSHIFT, 
                        std::vector <bool> (V_RANGE >> V_BITSHIFT, false)));

    is_black_ = std::vector< std::vector < std::vector <bool> > > (Y_RANGE >> Y_BITSHIFT,
                    std::vector < std::vector <bool> > (U_RANGE >> U_BITSHIFT, 
                        std::vector <bool> (V_RANGE >> V_BITSHIFT, false)));

    // Just so we don't load up an nnmc

    y_sum_ = 0;
    y_sampled_ = 0;
    y_mean_ = 100;
    first_sample_ = !load_nnmc;

    FILE *nnmc_file = fopen(nnmc_filename.c_str(), "rb");

    if (load_nnmc && nnmc_file) {
        nnmc_loaded_ = true;
        loadNnmc(nnmc_filename);
        updateColours();
        saveClassification();
    }
    else {
        nnmc_loaded_ = false;

        for (uint8_t y = 0; y < (Y_RANGE >> Y_BITSHIFT); y++) {
            for (uint8_t u = 0; u < (U_RANGE >> U_BITSHIFT); u++) {
                for (uint8_t v = 0; v < (V_RANGE >> V_BITSHIFT); v++) {
                    if (isWhite(y << Y_BITSHIFT, u << U_BITSHIFT,
                            v << V_BITSHIFT)) {
                        nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                            (u << Y_MAX_POW) | y] = (uint8_t) cWHITE;
                    }
                    else {
                        nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                            (u << Y_MAX_POW) | y] = (uint8_t) cBACKGROUND;
                    }
                }
            }
        }
    } 
}

void GreenYUVClassifier::classifyYUV(uint8_t y, uint8_t u, uint8_t v, Colour colour, int radius) {
    uint8_t min_y = std::max((y - radius) >> Y_BITSHIFT, 0);
    uint8_t max_y = std::min((y + radius) >> Y_BITSHIFT, (Y_RANGE - 1) >> Y_BITSHIFT);
    uint8_t min_u = std::max((u - radius) >> U_BITSHIFT, 0);
    uint8_t max_u = std::min((u + radius) >> U_BITSHIFT, (U_RANGE - 1) >> U_BITSHIFT);
    uint8_t min_v = std::max((v - radius) >> V_BITSHIFT, 0);
    uint8_t max_v = std::min((v + radius) >> V_BITSHIFT, (V_RANGE - 1) >> V_BITSHIFT);

    for (uint8_t y = min_y; y <= max_y; y++) {
        for (uint8_t u = min_u; u <= max_u; u++) {
            for (uint8_t v = min_v; v <= max_v; v++) {
                nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                    (u << Y_MAX_POW) | y] = (uint8_t) colour;
            }
        }
    }
}

void GreenYUVClassifier::sampleImageScanLines(const uint8_t* image, bool top, int n_rows, int n_cols, 
        const VisionInfoIn& info_in, int stepsize) {

   // For future samplings (of the bottom frame), we should use isGreen / isWhite to determine what our starting pixel is
   // for cases where the line we are standing on or our shoulder is in view

   int inc = 1;

   // These should be based on what kernel we use for our gradient calculation

   //int x_buffer = 1;
   //int y_buffer = 1; 
   int x_buffer = stepsize;
   int y_buffer = stepsize; 

   // We assume for the first sample that the first pixel is green

   // We flip between Green and white until we observe at least 1 transition
   // This needs to change for alternate head_yaw angles since sometimes there isn't any transition
   // Shouldn't be much of a problem since this logic is fitted for the initial head on sample

   int start_y;
   int num_transitions;
   bool detectingGreen;
   int transition_threshold = 3;
   bool noEdge = false;

   int num_preceding_consecutive_edges = 0;
   bool isEdge;
   // Green -> White -> Green -> White / Background

   Gradient_Operator grad_op = SOBEL;

   /*if (radioNAIVE->isChecked ()) {
      grad_op = NAIVE;
   } else if (radioSOBEL->isChecked ()) {
      grad_op = SOBEL;
   } else if (radioROBERTS->isChecked()) {
      grad_op = ROBERTS_CROSS;
   } else {
      grad_op = YUV_ISOTROPIC;
   }*/

   int threshold_buffer = 20;

   int threshold_otsu = calculate_threshold(image, n_rows, n_cols, grad_op, &gety);
   int threshold_y = threshold_otsu;
    //int threshold_u = calculate_threshold(image, n_rows, n_cols, grad_op, &gety);
    //int threshold_v = calculate_threshold(image, n_rows, n_cols, grad_op, &gety);

   noEdge = threshold_y < 30;

   for (int x_pos = x_buffer; x_pos < n_cols - x_buffer; x_pos += inc) {
      detectingGreen = true;
      num_transitions = 0;
      num_preceding_consecutive_edges = 0;
        // Start from the bottom of the image

        /*
        if (top) {
            start_y = info_in.cameraToRR.getTopEndScanCoord(x_pos) - 1;
        }
        else {
            start_y = info_in.cameraToRR.getBotEndScanCoord(x_pos) - 1;
        }
        */

      start_y = n_rows - 1;

      int firstWhiteY = 0;
      int lastGreenY = 0;

      for (int y_pos = start_y - y_buffer; y_pos >= y_buffer; y_pos -= inc) {
         // Make threshold more strict the further up you are in the image
         if (top) {
            if (y_pos > 6 * start_y / 10) {
               threshold_y = threshold_otsu;
            }
            else if (y_pos > 5 * start_y / 10) {
               threshold_y = std::max(0, threshold_otsu - 30);
            }
            else if (y_pos > 4 * start_y / 10) {
               threshold_y = std::max(0, threshold_otsu - 60);
            }
            else {
               threshold_y = std::max(0, threshold_otsu - 90);
            }
         }
         else {
            threshold_y = threshold_otsu;
         }
         
         if (noEdge) {
           int y_val = gety(image, y_pos, x_pos, n_cols);
           int u_val = getu(image, y_pos, x_pos, n_cols);
           int v_val = getv(image, y_pos, x_pos, n_cols);

           //q_image->setPixel(x_pos, y_pos, QColor("green").rgb()); 
           is_green_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT] = true;
        }
        else {
            int grad_y = calculate_gradient(image, y_pos, x_pos, n_cols, grad_op, &gety, stepsize);
            //int grad_u = calculate_gradient(image, y_pos, x_pos, n_cols, grad_op, &getu, stepsize);
            //int grad_v = calculate_gradient(image, y_pos, x_pos, n_cols, grad_op, &getv, stepsize);

            bool exceeded_y = grad_y > threshold_y;
            //bool exceeded_u = grad_u > threshold.u_;
            //bool exceeded_v = grad_v > threshold.v_;

            bool exceeded_thresh = exceeded_y;
            int y_val = gety(image, y_pos, x_pos, n_cols);
            int u_val = getu(image, y_pos, x_pos, n_cols);
            int v_val = getv(image, y_pos, x_pos, n_cols);

                /*
                    Proposed algorithm
                    Assume Green->White is ok
                    Store the y value corresponding to the first detected "white" pixel
                    When we see subsequent edges in white, check if the first value after the edge is < this
                    If it is, then assume we have transitioned back to green

                */ 

            if (exceeded_thresh) {
               isEdge = true;
               num_preceding_consecutive_edges++;
            }
            else {
               isEdge = false;

               if (num_preceding_consecutive_edges != 0) {
                  num_transitions++;
                  if (detectingGreen) {
                     if (y_val > lastGreenY + threshold_buffer) {
                        // Begin detecting white
                        detectingGreen = false;
                        firstWhiteY = y_val;
                        //num_transitions++;
                     }
                  }
                  else {
                     if (y_val < firstWhiteY - threshold_buffer) {
                        detectingGreen = true;
                        //num_transitions++;
                     } 
                  }

                  if (num_transitions > transition_threshold) {
                     break;
                  }

                  num_preceding_consecutive_edges = 0;
               }

                /* 
                    Some problems
                    Edge may extend for multiple pixels

                    We only blindly detect the first transition                        
                        Subsequent ones are informed by our current classification
                 */

                /*
                if (exceeded_thresh) {
                    isEdge = true;

                    if (num_transitions == 0) {
                        // We only blindly detect the first transition                        
                        detectingGreen = !detectingGreen;
                        num_transitions++;
                    }
                    else {
                        // Subsequent ones are informed by our current classification
                        // If we are detecting one colour, reach an edge and it is another colour, then switch
                        if (num_preceding_consecutive_edges == 0) {
                            if (detectingGreen && isWhite(y_val, u_val, v_val)) {
                                detectingGreen = false;
                                num_transitions++;
                            }
                            else if (!detectingGreen && isGreen(y_val, u_val, v_val, true)) {
                                detectingGreen = true;
                                num_transitions++;
                            }
                            if (num_transitions > transition_threshold) {
                                break;
                            }
                        }
                    }
                    num_preceding_consecutive_edges++;
                } 
                else {
                    isEdge = false;
                    num_preceding_consecutive_edges = 0;
                }
                */

               if (!isEdge) {
                  if (detectingGreen) {
                     //q_image->setPixel(x_pos, y_pos, QColor("green").rgb()); 
                     is_green_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT] = true;
                     is_white_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT] = false;
                     lastGreenY = y_val;
                  }
                  else {
                     //q_image->setPixel(x_pos, y_pos, QColor("white").rgb()); 
                     is_white_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT] = true;
                     is_green_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT] = false;
                  }
               }
               else {
                  //q_image->setPixel(x_pos, y_pos, QColor("red").rgb());
               }
            }
         }
      }
   }
}

int GreenYUVClassifier::calculate_threshold(const uint8_t* image, int n_rows, int n_cols, Gradient_Operator grad_op,
        uint8_t (*getYUV)(const uint8_t*, int, int, int)) {
    int x_buffer = 1;
    int y_buffer = 1;
    int inc = 1;
    int start_y;

    std::vector <int> gradients = std::vector <int> (256, 0);

    for (int x_pos = x_buffer; x_pos < n_cols - x_buffer; x_pos += inc) {
        start_y = n_rows - 1;

        for (int y_pos = start_y - y_buffer; y_pos >= y_buffer; y_pos -= inc) {
            gradients[std::min(calculate_gradient(image, y_pos, x_pos, n_cols, grad_op, getYUV, 1), 255)]++;
        }
    }

    double intra_class_var;
    int thresh = getThresholdValueOtsu(gradients, intra_class_var);
    std::cout << "Gradient: " << grad_op << " thresh: " << thresh << "\n";
    return thresh;
}

int GreenYUVClassifier::calculate_gradient(const uint8_t* image, int y_pos, int x_pos, int n_cols,
    Gradient_Operator g, uint8_t (*getYUV)(const uint8_t*, int, int, int), int stepsize) {

    long long grad_x, grad_y;
    long long grad = 0;

    switch (g) {
        case ROBERTS_CROSS:
            /* Roberts Cross */
            /*   G_x    G_y
             * [1  0] [ 0 1]
             * [0 -1] [-1 0]
             *
             * In the image, we are at X
             * ---> increasing x
             *
             * [   X] | increasing y
             * [    ] V
             *
             */
            grad_x = getYUV(image, y_pos,             x_pos - stepsize, n_cols) - getYUV(image, y_pos + stepsize, x_pos, n_cols);
            grad_y = -getYUV(image, y_pos + stepsize, x_pos - stepsize, n_cols) + getYUV(image, y_pos,            x_pos, n_cols);
            grad = grad_x * grad_x + grad_y * grad_y;            
            break;

        case SOBEL:
            /* Sobel */
            /*    G_x        G_y
             * [1  0 -1] [ 1  2  1]
             * [2  0 -2] [ 0  0  0]
             * [1  0 -1] [-1 -2 -1]
             *
             * In the image, we are at X
             * -------> increasing x
             *
             * [       ] | increasing y
             * [   X   ] |
             * [       ] V
             * 
             */
            grad_x =      getYUV(image, y_pos - stepsize, x_pos - stepsize, n_cols) + 
                     2 *  getYUV(image, y_pos           , x_pos - stepsize, n_cols) +
                          getYUV(image, y_pos + stepsize, x_pos - stepsize, n_cols) +
                     -    getYUV(image, y_pos - stepsize, x_pos + stepsize, n_cols) +
                     -2 * getYUV(image, y_pos           , x_pos + stepsize, n_cols) +
                     -    getYUV(image, y_pos + stepsize, x_pos + stepsize, n_cols);
            grad_y =      getYUV(image, y_pos - stepsize, x_pos - stepsize, n_cols) + 
                     2 *  getYUV(image, y_pos - stepsize, x_pos           , n_cols) +
                          getYUV(image, y_pos - stepsize, x_pos + stepsize, n_cols) +
                     -    getYUV(image, y_pos + stepsize, x_pos - stepsize, n_cols) +
                     -2 * getYUV(image, y_pos + stepsize, x_pos           , n_cols) +
                     -    getYUV(image, y_pos + stepsize, x_pos + stepsize, n_cols);
            grad = grad_x * grad_x + grad_y * grad_y;            
            break;

        case NAIVE:
            grad = abs(getYUV(image, y_pos + stepsize, x_pos, n_cols)
                  -getYUV(image, y_pos    , x_pos, n_cols));
            break;

        case YUV_ISOTROPIC: 
            {
                /* Y_Isotropic */
                /*    G_v        G_h       G_n        G_s
                 * [1  0 -1] [ 1  2  1] [2  1  0] [ 0  1  2]
                 * [2  0 -2] [ 0  0  0] [1  0 -1] [-1  0  1]
                 * [1  0 -1] [-1 -2 -1] [0 -1 -2] [-2 -1  0]
                 *
                 * In the image, we are at X
                 * -------> increasing x
                 *
                 * [       ] | increasing y
                 * [   X   ] |
                 * [       ] V
                 * 
                 */
                int grad_v = abs(     getYUV(image, y_pos - stepsize, x_pos - stepsize, n_cols) +
                                  2 * getYUV(image, y_pos           , x_pos - stepsize, n_cols) +
                                      getYUV(image, y_pos + stepsize, x_pos - stepsize, n_cols) +
                                 -1 * getYUV(image, y_pos - stepsize, x_pos + stepsize, n_cols) +
                                 -2 * getYUV(image, y_pos           , x_pos + stepsize, n_cols) +
                                 -1 * getYUV(image, y_pos + stepsize, x_pos + stepsize, n_cols));
                int grad_h = abs(     getYUV(image, y_pos - stepsize, x_pos - stepsize, n_cols) +
                                  2 * getYUV(image, y_pos - stepsize, x_pos           , n_cols) +
                                      getYUV(image, y_pos - stepsize, x_pos + stepsize, n_cols) +
                                 -1 * getYUV(image, y_pos + stepsize, x_pos - stepsize, n_cols) +
                                 -2 * getYUV(image, y_pos + stepsize, x_pos           , n_cols) +
                                 -1 * getYUV(image, y_pos + stepsize, x_pos + stepsize, n_cols));
                int grad_n = abs( 2 * getYUV(image, y_pos - stepsize, x_pos - stepsize, n_cols) +
                                      getYUV(image, y_pos - stepsize, x_pos           , n_cols) +
                                      getYUV(image, y_pos           , x_pos - stepsize, n_cols) +
                                 -2 * getYUV(image, y_pos           , x_pos + stepsize, n_cols) +
                                 -1 * getYUV(image, y_pos + stepsize, x_pos           , n_cols) +
                                 -1 * getYUV(image, y_pos + stepsize, x_pos + stepsize, n_cols));
                int grad_s = abs(     getYUV(image, y_pos - stepsize, x_pos           , n_cols) +
                                  2 * getYUV(image, y_pos - stepsize, x_pos + stepsize, n_cols) +
                                 -1 * getYUV(image, y_pos           , x_pos - stepsize, n_cols) +
                                      getYUV(image, y_pos           , x_pos + stepsize, n_cols) +
                                 -2 * getYUV(image, y_pos + stepsize, x_pos - stepsize, n_cols) +
                                 -1 * getYUV(image, y_pos + stepsize, x_pos           , n_cols));

                grad = std::max(grad_v, std::max(grad_h, std::max(grad_n, grad_s)));
            }
            break; 
        default:
            break;
    }

    return std::sqrt(grad);
}

void GreenYUVClassifier::draw_edge(const uint8_t* image, int n_rows, int n_col, Gradient_Operator g) {
    uint8_t image_edge[n_rows*n_col*3];
    memset(image_edge, 0, n_rows * n_col * 3 * sizeof(uint8_t)); 

    std::string gradient;

    // Thanks stack overflow 
    // http://stackoverflow.com/questions/2654480/writing-bmp-image-in-pure-c-c-without-other-libraries

    int filesize = 54 + (3*n_col*n_rows);

    // Little endian

    // Set header info
    unsigned char bmpfileheader_top[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader_top[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};

    bmpfileheader_top[ 2] = (unsigned char)(filesize    );
    bmpfileheader_top[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader_top[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader_top[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader_top[ 4] = (unsigned char)(       n_col    );
    bmpinfoheader_top[ 5] = (unsigned char)(       n_col>> 8);
    bmpinfoheader_top[ 6] = (unsigned char)(       n_col>>16);
    bmpinfoheader_top[ 7] = (unsigned char)(       n_col>>24);
    bmpinfoheader_top[ 8] = (unsigned char)(       n_rows    );
    bmpinfoheader_top[ 9] = (unsigned char)(       n_rows>> 8);
    bmpinfoheader_top[10] = (unsigned char)(       n_rows>>16);
    bmpinfoheader_top[11] = (unsigned char)(       n_rows>>24);

    for (int i = 0; i < 3; i++) {
        std::ostringstream s;

        std::string type;
        int normalising = 1;

        s << "Edge_image_" << gradient << "_";

        int threshold;

        switch (i) {
            case 0:
                type = "y";
                threshold = calculate_threshold(image, n_rows, n_col, g, &gety);
                s << type << "_" << (int) threshold;
                break;
            case 1:
                type = "u";
                threshold = calculate_threshold(image, n_rows, n_col, g, &getu);
                s << type << "_" << (int) threshold;
                break;
            default:
                type = "v";
                threshold = calculate_threshold(image, n_rows, n_col, g, &getv);
                s << type << "_" << (int) threshold;
                break;
        }


        s << "_" << normalising << ".bmp";

        FILE *dumpFile_ = fopen(s.str().c_str(), "w");
        int max_grad = 0;
        std::cout << type << gradient;

        // bmp rows stored backwards
        for (int row = 3; row < n_rows; ++row) {
            for (int col = 3; col < n_col; ++col) {
                int grad;

                switch (i) {
                    case 0:
                        grad = calculate_gradient(image, row, col, n_col, g, &gety, 1);
                        break;
                    case 1:
                        grad = calculate_gradient(image, row, col, n_col, g, &getu, 1);
                        break;
                    default:
                        grad = calculate_gradient(image, row, col, n_col, g, &getv, 1);
                        break;
                }

                bool exceeded_thresh = grad > threshold;
                max_grad = std::max(max_grad, grad);

                grad /= normalising;
                grad = std::min(grad, 255);

                if (exceeded_thresh) {
                    // Hot pink
                    image_edge[((((n_rows-row-1)*n_col)+col)*3)+0] = 180;
                    image_edge[((((n_rows-row-1)*n_col)+col)*3)+1] = 105;
                    image_edge[((((n_rows-row-1)*n_col)+col)*3)+2] = 255;
                }
                else {
                    image_edge[((((n_rows-row-1)*n_col)+col)*3)+0] = grad;
                    image_edge[((((n_rows-row-1)*n_col)+col)*3)+1] = grad;
                    image_edge[((((n_rows-row-1)*n_col)+col)*3)+2] = grad;
                }
            }
        }

        std::cout << max_grad << std::endl;

        // Write header info
        fwrite(bmpfileheader_top,1,14,dumpFile_);
        fwrite(bmpinfoheader_top,1,40,dumpFile_);

        // Write image data
        fwrite(&image_edge, n_col*n_rows*3, 1, dumpFile_);
        fflush(dumpFile_);
        fclose(dumpFile_); 
    }
}

void GreenYUVClassifier::resetNnmc() {
    for (uint8_t y = 0; y < (Y_RANGE >> Y_BITSHIFT); y++) {
        for (uint8_t u = 0; u < (U_RANGE >> U_BITSHIFT); u++) {
            for (uint8_t v = 0; v < (V_RANGE >> V_BITSHIFT); v++) {
                nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                    (u << Y_MAX_POW) | y] = (uint8_t) cBACKGROUND;
                is_green_[y][u][v] = false;
                is_white_[y][u][v] = false;
                is_black_[y][u][v] = false;
            }
        } 
    } 
}

void GreenYUVClassifier::sampleImage(const uint8_t* image, bool top, int n_rows, int n_cols, 
        const VisionInfoIn& info_in) {
    float head_yaw = info_in.cameraToRR.values.joints.angles[Joints::HeadYaw];

    if ((top && fabs(head_yaw) < M_PI / 6) || 
            (!top && fabs(head_yaw) < M_PI / 4)) {
        if (!first_sample_) {
            for (unsigned int y = 0; y < yuv_counts_.size(); y++) {
                for (unsigned int u = 0; u < yuv_counts_[0].size(); u++) {
                    for (unsigned int v = 0; v < yuv_counts_[0][0].size(); v++) {
                        yuv_counts_[y][u][v] = 0;
                    }
                }
            } 
        }

        int start_y, end_y, start_x, end_x, inc;
        double target_coverage;
        total_count_ = 0;

        end_y = n_rows;
        end_x = n_cols;
        start_x = 0;
        if(top){
            start_y = n_rows * getTopFrameStartScan(head_yaw);
            target_coverage = INITIAL_MAX_COVERAGE_TOP;
            inc = 1;
        }
        else {
            start_y = 0;
            target_coverage = INITIAL_MAX_COVERAGE_BOT;
            inc = 1;
        }

        for (int y_pos = start_y; y_pos < end_y; y_pos += inc) {
            for (int x_pos = start_x; x_pos < end_x; x_pos += inc) {
                // Ignore body parts
                if (top && y_pos >= info_in.cameraToRR.getTopEndScanCoord(x_pos)) {
                    break;
                }
                else if (!top && y_pos >= info_in.cameraToRR.getBotEndScanCoord(x_pos)) {
                    break;
                }

                int y_val = gety(image, y_pos, x_pos, n_cols);
                int u_val = getu(image, y_pos, x_pos, n_cols);
                int v_val = getv(image, y_pos, x_pos, n_cols);
                total_count_++;

                y_sum_ += y_val;
                y_sampled_++;

                yuv_counts_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT]++;
            }
        }

        y_mean_ = y_sum_ / y_sampled_;            

        double current_coverage;
        bool expandedCoverage = false;

        if (first_sample_) {
            current_coverage = 0;
        }
        else {
            current_coverage = getCoverage();
        }

        while (current_coverage < target_coverage) {
            expandedCoverage = true;

            double expanded_coverage = expandVerticalPeak(target_coverage - current_coverage, first_sample_);

            if (fabs(expanded_coverage) < 0.00001) {
                first_sample_ = true;
                expanded_coverage = expandVerticalPeak(target_coverage - current_coverage, true);
            }
            current_coverage += expanded_coverage;
        }      

        // Only perform this if we have expanded coverage
        // Otherwise this is a waste of time
        if (expandedCoverage) {
            fillPoints(true);
        }
    }
}

double GreenYUVClassifier::getTopFrameStartScan(double head_yaw) {
    if (abs(head_yaw) < 0.5) {
        return 0.6; 
    }
    else {
        return 0.7;
    }
}

double GreenYUVClassifier::getCoverage() {
    double coverage = 0;
    max_peak_count_ = 0;

    for (unsigned int y = 0; y < yuv_counts_.size(); y++) {
        for (unsigned int u = 0; u < yuv_counts_[0].size(); u++) {
            for (unsigned int v = 0; v < yuv_counts_[0][0].size(); v++) {
                if (yuv_counts_[y][u][v] > max_peak_count_) {
                    max_peak_count_ = yuv_counts_[y][u][v];
                } 

                if (is_green_[y][u][v]) {
                    coverage += yuv_counts_[y][u][v];
                }
            }
        }
    } 

    return coverage / total_count_;
}

double GreenYUVClassifier::expandVerticalPeak(double coverage, bool new_peak) {
    if (new_peak) {
        // Get highest peak in histogram 
        YUVTriple max_peak_bin(-1, -1, -1);
        max_peak_count_ = 0;

        for (unsigned int y = 0; y < yuv_counts_.size(); y++) {
            for (unsigned int u = 0; u < yuv_counts_[0].size(); u++) {
                for (unsigned int v = 0; v < yuv_counts_[0][0].size(); v++) {
                    if (!is_green_[y][u][v] && yuv_counts_[y][u][v] > max_peak_count_) {
                        max_peak_bin.y_ = y;
                        max_peak_bin.u_ = u; 
                        max_peak_bin.v_ = v; 

                        max_peak_count_ = yuv_counts_[y][u][v];
                    } 
                }
            }
        } 

        YUV_candidate_list_.push_back(max_peak_bin);;
        first_sample_ = false;
    }

    int target_count = total_count_ * coverage;
    int validated_count = 0;

    int curr_peak_thresh = max_peak_count_;

    // Starting from our peak, keep expanding to bins with counts > curr_peak_thresh
    // If we cannot progress with a particular curr_peak_thresh, we decrement it and
    // search again

    while (curr_peak_thresh > 0.1 * max_peak_count_) {
        // bins_todo continues to grow within the loop

        std::list < YUVTriple >::iterator it = YUV_candidate_list_.begin();

        while (it != YUV_candidate_list_.end()) {
            // We need to do this check because although when a bucket was added, it may not have been green
            // So may be added twice
            if (is_green_[it->y_][it->u_][it->v_]) {
                it = YUV_candidate_list_.erase(it);
            }
            else if (yuv_counts_[it->y_][it->u_][it->v_] >= curr_peak_thresh) {
                is_green_[it->y_][it->u_][it->v_] = true;
                validated_count += yuv_counts_[it->y_][it->u_][it->v_];

                if (validated_count > target_count) {
                    return 1.0 * validated_count / total_count_;
                }

                std::vector< YUVTriple > neighbours = getBinNeighbours(*it);

                for (unsigned int j = 0; j < neighbours.size(); j++) {
                    YUVTriple curr_neigh = neighbours[j];

                    if (!is_green_[curr_neigh.y_][curr_neigh.u_][curr_neigh.v_]) {
                        YUV_candidate_list_.push_back(curr_neigh);
                    }
                }

                // We are done with this bin
                it = YUV_candidate_list_.erase(it);
            }
            else {
                it++;
            }
        }

        curr_peak_thresh -= 0.1 * max_peak_count_; 
    }

    return 1.0 * validated_count / total_count_;
}

void GreenYUVClassifier::fillPoints(bool isGreen) {
    std::vector< std::vector < std::vector <bool> > > *classif;

    if (isGreen) {
        classif = &is_green_;
    }
    else {
        classif = &is_white_;
    }

    // Fill in for v, u, then y axis
    unsigned int y, u, v;
    unsigned int start, end;

    // Fill in v
    for (y = 0; y < (*classif).size(); y++) {
        for (u = 0; u < (*classif)[0].size(); u++) {
            for (v = 0; v < (*classif)[0][0].size() && !(*classif)[y][u][v]; v++) {}

            start = v;

            for (v = (*classif)[0][0].size() - 1; v > start && !(*classif)[y][u][v]; v--) {}

            end = v;

            for (v = start; v <= end; v++) {
                (*classif)[y][u][v] = true;
            }
        }
    }         

    // Fill in u
    for (y = 0; y < (*classif).size(); y++) {
        for (v = 0; v < (*classif)[0][0].size(); v++) {
            for (u = 0; u < (*classif)[0].size() && !(*classif)[y][u][v]; u++) {}

            start = u;

            for (u = (*classif)[0].size() - 1; u > start && !(*classif)[y][u][v]; u--) {}

            end = u;

            for (u = start; u <= end; u++) {
                (*classif)[y][u][v] = true;
            }
        }
    }  

    // Fill in y
    for (u = 0; u < (*classif)[0].size(); u++) {
        for (v = 0; v < (*classif)[0][0].size(); v++) {
            for (y = 0; y < (*classif).size() && !(*classif)[y][u][v]; y++) {}

            start = y;

            for (y = (*classif).size() - 1; y > start && !(*classif)[y][u][v]; y--) {}

            end = y;

            for (y = start; y <= end; y++) {
                (*classif)[y][u][v] = true;
            }
        }
    }  
}

std::vector< YUVTriple > GreenYUVClassifier::getBinNeighbours(const YUVTriple &yuv) {
    std::vector <YUVTriple> res;

    for (int k = 0; k < 6; k++) {
        if ((yuv.y_ + di[k] >= 0) && (yuv.y_ + di[k] < (Y_RANGE >> Y_BITSHIFT)) 
                && (yuv.u_ + dj[k] >= 0) && (yuv.u_ + dj[k] < (U_RANGE >> U_BITSHIFT))
                && (yuv.v_ + dk[k] >= 0) && (yuv.v_ + dk[k] < (V_RANGE >> V_BITSHIFT))) {
            res.push_back(YUVTriple(yuv.y_ + di[k], yuv.u_ + dj[k], yuv.v_ + dk[k])); 
        } 
    }

    return res;
}

bool GreenYUVClassifier::amWalking(const ActionCommand::All &commands) {
    return abs(commands.body.forward) > 0 || abs(commands.body.left) > 0 || abs(commands.body.turn) > 0;
}

void GreenYUVClassifier::saveClassification(void) {
    for (uint8_t y = 0; y < (Y_RANGE >> Y_BITSHIFT); y++) {
        for (uint8_t u = 0; u < (U_RANGE >> U_BITSHIFT); u++) {
            for (uint8_t v = 0; v < (V_RANGE >> V_BITSHIFT); v++) {
                if (isGreen(y << Y_BITSHIFT, u << U_BITSHIFT, 
                        v << V_BITSHIFT, true)) {
                    nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] = (uint8_t) cGREEN;
                } 
                else if (isWhite(y << Y_BITSHIFT, 
                        u << U_BITSHIFT, v << V_BITSHIFT)) {
                    nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] = (uint8_t) cWHITE;
                }
                else if (isBlack(y << Y_BITSHIFT, 
                        u << U_BITSHIFT, v << V_BITSHIFT)) {
                    nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] = (uint8_t) cBLACK;
                }
                else {
                    nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] = (uint8_t) cBACKGROUND;
                }
            }
        } 
    } 
}

void GreenYUVClassifier::saveNnmc(std::string filename) {
    FILE *f = fopen(filename.c_str(), "wb");
    int bzerror;
    BZFILE *bf = BZ2_bzWriteOpen(&bzerror, f, 9, 0, 100);
    if (! bzerror == BZ_OK) {
        throw std::runtime_error("error opening nnmc file for compression");
    }
    BZ2_bzWrite (&bzerror, bf, &nnmc_, sizeof (nnmc_));
    if (! bzerror == BZ_OK) {
        throw std::runtime_error("error compressing nnmc file");
    }
    BZ2_bzWriteClose(&bzerror, bf, 0, 0, 0);
    if (! bzerror == BZ_OK) {
        throw std::runtime_error("error closing nnmc file");
    }
    fclose(f);
}

void GreenYUVClassifier::loadNnmc(std::string filename) {
    size_t nnmc_size = (Y_RANGE >> Y_BITSHIFT) * (U_RANGE >> U_BITSHIFT) *
        (V_RANGE >> V_BITSHIFT);
    FILE *f = fopen(filename.c_str(), "rb");

    char magic[ sizeof (bzip_magic) ];
    if (fread(magic, 1, sizeof(magic), f) != sizeof(magic)) {
        throw std::runtime_error("error openning nnmc file");
    }
    fseek(f, 0, SEEK_SET);

    /* Note: as our calibration files only contains bytes in [0..9] it
     * is impossible to have a false magic number on an uncompressed file
     */
    if (memcmp(magic, bzip_magic, sizeof(magic)) == 0) {
        /* File is bzip'd */
        int bzerror;
        BZFILE *bf = BZ2_bzReadOpen(&bzerror, f, 0, 0, 0, 0);
        if (! bzerror == BZ_OK) {
           throw std::runtime_error("error openning nnmc file for decompression");
        }
        BZ2_bzRead (&bzerror, bf, nnmc_, nnmc_size);
        if (! bzerror == BZ_OK && ! bzerror == BZ_STREAM_END) {
            throw std::runtime_error("error decompressing nnmc file");
        }
        BZ2_bzReadClose(&bzerror, bf);
        if (! bzerror == BZ_OK) {
           throw std::runtime_error("error closing nnmc file");
        }
    } 
    else {
        /* Read uncompressed file, faster loading */
        size_t ret;
        ret = fread(nnmc_, nnmc_size, 1, f);
    }
    fclose(f);

    /*
    // For converting between old and new nnmc (since we have changed cPLANE_COLOURS)
    for (uint8_t y = 0; y < (Y_RANGE >> Y_BITSHIFT); y++) {
        for (uint8_t u = 0; u < (U_RANGE >> U_BITSHIFT); u++) {
            for (uint8_t v = 0; v < (V_RANGE >> V_BITSHIFT); v++) {
                if (nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] == 5) {
                    nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] = (uint8_t) cGREEN;
                }
                else if (nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] == 6) {
                    nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] = (uint8_t) cWHITE;
                }
                else if (nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] == 7) {
                    nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] = (uint8_t) cBLACK;
                }
                else {
                    nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) | 
                        (u << Y_MAX_POW) | y] = (uint8_t) cBACKGROUND;
                }
            }
        } 
    } 
    */
}

void GreenYUVClassifier::updateColours() {
    for (uint8_t y = 0; y < (Y_RANGE >> Y_BITSHIFT); y++) {
        for (uint8_t u = 0; u < (U_RANGE >> U_BITSHIFT); u++) {
            for (uint8_t v = 0; v < (V_RANGE >> V_BITSHIFT); v++) {
                if (nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) |
                        (u << Y_MAX_POW) | y] == (uint8_t) cGREEN) {
                    is_green_[y][u][v] = true;
                }
                else if (nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) |
                        (u << Y_MAX_POW) | y] == (uint8_t) cWHITE) {
                    is_white_[y][u][v] = true;
                }
                else if (nnmc_[(v << (Y_MAX_POW + U_MAX_POW)) |
                        (u << Y_MAX_POW) | y] == (uint8_t) cBLACK) {
                    is_black_[y][u][v] = true;
                }
            }
        } 
    } 
}
