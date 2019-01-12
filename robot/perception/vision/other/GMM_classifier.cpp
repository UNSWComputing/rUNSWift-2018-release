#include "perception/vision/other/GMM_classifier.hpp"
#include <iostream>

GaussianDistribution::GaussianDistribution()
{
}

GaussianDistribution::GaussianDistribution(int n_features, const Vectorxf &mean, const Matrixxf &precision, double log_normalise_factor)
{
    mean_ = Vectorxf(mean);
    precision_ = Matrixxf(precision);
    log_normalise_factor_ = log_normalise_factor;
}

double GaussianDistribution::logprob(const Vectorxf &X)
{
    Vectorxf X_ = X - mean_;
    return log_normalise_factor_ - 0.5 * (X_ * precision_ * X_.transpose())(0);
}

Estimator::Estimator(ClassifierType type) : 
  pca_(type), ccgmm_(type)
{
    n_features_ = pca_.getNFeatures();
    input_width_ = pca_.getInputWidth();
    input_height_ = pca_.getInputHeight();
}

int Estimator::predict(const RegionI& region)
{
    const Matrixxf img = convert(region);

    /*
    NOTE:
    not retruning the real probability
    the probability of the most likely class is 1 and other classes are all 0
    */
    Vectorxf X = preprocessor(img);
    Vectorxf Z = pca_.transform(X);
    
    double max_likelihood = -std::numeric_limits<double>::infinity();
    int argmax = 0;
    for (int i = 0; i < ccgmm_.getNClasses(); i++)
    {
        double likelihood = ccgmm_.logprob(i, Z);
        if (likelihood > max_likelihood)
        {
            max_likelihood = likelihood;
            argmax = i;
        }
    }
    
    return argmax;
    
}

Matrixxf Estimator::convert(const RegionI& region)
{
    RegionI::iterator_fovea cur_point = region.begin_fovea();
    // Track the literal location of the iterators.
    int x = 0;
    int y = 0;
    // The number of rows and columns in the region.
    int rows = region.getRows();
    int cols = region.getCols();
    Matrixxf dst(rows, cols);
    // Loop
    for(int pixel = 0; pixel < cols * rows; ++pixel)
    {
        // std::cout<<cur_point.colour() << " ";
        if (cur_point.colour() == cWHITE){
            dst(y, x) = 255;
        }
        else{
            dst(y, x) = 0;
        }
        cur_point++;
        x++;
        if (x == cols){
            x = 0;
            ++y;
        }
    }
    return dst;
}

Vectorxf Estimator::preprocessor(const Matrixxf &img)
{
    return reshape(resize(img, input_height_, input_width_));
}

Matrixxf Estimator::resize(const Matrixxf &img, int resize_height, int resize_width)
{
    /*
	Resize the input image with Bilinear Interpolation method.

	*/
	int rows = img.rows();
    int cols = img.cols();
    float deltaX = (float)(rows) / resize_width;
    float lastX = (0.5 * rows) / resize_width - 0.5;
    // float x = lastX;
    float deltaY = (float)(cols) / resize_height;
    float lastY = (0.5 * cols) / resize_height - 0.5;
    float y = lastY;
    Matrixxf dst(resize_width, resize_height);
    for(int i = 0; i < resize_width; ++i) {
        //float x = (i+0.5)*rows/NEW_SIZE-0.5;
        float x = lastX;
        lastX += deltaX;
        int fx = (int)x;
        x -= fx;
        short x1 = (1.f - x) * 2048;
        short x2 = 2048 - x1;
        if (fx >= rows - 1){
            fx = rows - 2;
        }
        lastY = (0.5 * cols) / resize_height -0.5;
        for(int j = 0; j < resize_height; ++j) {
            //float y = (j+0.5)*cols/NEW_SIZE-0.5;
            y = lastY;
            lastY += deltaY;
            
            int fy = (int)y;
            y -= fy;
            if (fy >= cols - 1){
                fy = cols - 2;
            }
            short y1 = (1.f - y) * 2048;
            short y2 = 2048 - y1;
            dst(i, j) = ((int)img(fx, fy) * x1 * y1 + (int)img(fx + 1, fy) * x2 * y1
                        + (int)img(fx, fy + 1) * x1 * y2 + (int)img(fx + 1, fy + 1) * x2 * y2) >> 22;
            
        }
    }
    return dst;
}

Vectorxf Estimator::reshape(const Matrixxf &img)
{
    // Reshape the matrix to be 1-dim vector
    int rows = img.rows();
    int cols = img.cols(); 
	Vectorxf result(rows * cols);
    for(int i = 0; i < cols; i++)
        for(int j = 0; j < rows; j++)
            result(i * rows + j) = img(j, i);
    return result;
}

PCA::PCA(ClassifierType type)
{
    /*
    // PCA MODEL DUMP FORMAT
    input_data_width(int) input_data_height(int) pca_components(int)
    pca_means[input_data_features](double)
    pca_components_transposed[input_data_features, pca_components](double)
    */

    std::string pca_model_dir = "";

    switch(type) {
        case ball:
            pca_model_dir = PCA_MODEL_DIR_BALL; break;
        case penaltySpot:
            pca_model_dir = PCA_MODEL_DIR_PENALTY_SPOT; break;
        default:
            break;
    }
    FILE *file = fopen(pca_model_dir.c_str(), "rb");
    if(file != NULL)
    {
        fread(&input_width_, sizeof(int), 1, file);
        fread(&input_height_, sizeof(int), 1, file);
        fread(&n_components_, sizeof(int), 1, file);
        n_features_ = input_width_ * input_height_;
        means_ = Vectorxf(n_features_);
        components_T_ = Matrixxf(n_features_, n_components_);
        // read means
        for (int i = 0; i < n_features_; i++)
        {
            double val;
            fread(&val, sizeof(double), 1, file);
            means_(i) = val;
        }
        // read transform matrix
        for (int i = 0; i < n_features_; i++)
            for (int j = 0; j < n_components_; j++)
            {
                double val;
                fread(&val, sizeof(double), 1, file);
                components_T_(i, j) = val;
            }
        fclose(file);
    }
    else
        printf("ERROR: failed to read PCA\n");
}

Vectorxf PCA::transform(const Vectorxf &X)
{
    return (X - means_) * components_T_;
}

int PCA::getNComponent()
{
    return n_components_;
}

int PCA::getNFeatures()
{
    return n_features_;
}

int PCA::getInputWidth()
{
    return input_width_;
}

int PCA::getInputHeight()
{
    return input_height_;
}

ClassConditionalGMM::ClassConditionalGMM(ClassifierType type)
{
    /*
    // GMM MODEL DUMP FORMAT
    gmm_features(int), gmm_components(int), gmm_classes(int)
    log_gmm_priors[gmm_classes](double)
    for each gmm_class:
        for each gmm_component:
            log_gaussian_component_weights(double)
            gaussian_component_means[gmm_features](double)
            log_gaussian_normalise_factor(double)
            gaussian_component_precision_matrix[gmm_features, gmm_features](double)
    */
    std::string gmm_model_dir = "";

    switch(type) {
        case ball:
            gmm_model_dir = GMM_MODEL_DIR_BALL; break;
        case penaltySpot:
            gmm_model_dir = GMM_MODEL_DIR_PENALTY_SPOT; break;
        default:
            break;
    }
    FILE *file = fopen(gmm_model_dir.c_str(), "rb");

    if(file != NULL)
    {
        fread(&n_features_, sizeof(int), 1, file);
        fread(&n_components_, sizeof(int), 1, file);
        fread(&n_classes_, sizeof(int), 1, file);

        log_priors_ = std::vector<double>(n_classes_);
        log_weights_ = std::vector<std::vector<double> >(n_classes_);
        gaussians_ = std::vector<std::vector<GaussianDistribution> >(n_classes_);
        // read priors
        for (int i = 0; i < n_classes_; i++)
        {
            double val;
            fread(&val, sizeof(double), 1, file);
            log_priors_.push_back(val);
        }
        for (int cls = 0; cls < n_classes_; cls++)
        {
            double val;

            log_weights_[cls] = std::vector<double>(n_components_);
            gaussians_[cls] = std::vector<GaussianDistribution>(n_components_);
            for (int i = 0; i < n_components_; i++)
            {
                // read weights
                fread(&val, sizeof(double), 1, file);
                log_weights_[cls][i] = val;

                // read means
                Vectorxf mean(n_features_);
                for (int j = 0; j < n_features_; j++)
                {
                    fread(&val, sizeof(double), 1, file);
                    mean(j) = val;
                }

                // read covariance matrix
                double log_normalise_factor;
                fread(&log_normalise_factor, sizeof(double), 1, file);

                // read precision matrix
                Matrixxf precision(n_features_, n_features_);
                for (int j = 0; j < n_features_; j++)
                    for (int k = 0; k < n_features_; k++)
                    {
                        fread(&val, sizeof(double), 1, file);
                        precision(j, k) = val;
                    }

                gaussians_[cls][i] = GaussianDistribution(n_features_, mean, precision, log_normalise_factor);
            }
        }
        fclose(file);
    }
    else
        printf("ERROR: failed to read GMM\n");
}

double ClassConditionalGMM::logprob(int cls, const Vectorxf &X)
{
    double log_likelihood = 0.0, likelihood = 0.0;
    double *component_log_likelihood = new double[n_components_];
    double max_ll = -std::numeric_limits<double>::infinity();
    for(int i = 0; i < n_components_; i++)
    {
        component_log_likelihood[i] = log_weights_[cls][i] + gaussians_[cls][i].logprob(X);
        if(component_log_likelihood[i] > max_ll)
            max_ll = component_log_likelihood[i];
    }
    for(int i = 0; i < n_components_; i++) 
        likelihood += exp(component_log_likelihood[i] - max_ll);
    
    log_likelihood = max_ll + log(likelihood);
    delete []component_log_likelihood;
    
    return log_priors_[cls] + log_likelihood;
}

int ClassConditionalGMM::getNClasses()
{
    return n_classes_;
}

int ClassConditionalGMM::getNComponent()
{
    return n_components_;
}

int ClassConditionalGMM::getNFeatures()
{
    return n_features_;
}