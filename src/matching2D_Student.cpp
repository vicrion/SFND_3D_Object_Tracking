
#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F || descRef.type() != CV_32F)
        {
            // Convert binary descriptors to CV_32F for FLANN
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        // k-Nearest Neighbors (k=2)
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descSource, descRef, knnMatches, 2);

        // Filter matches using the ratio test
        float ratioThreshold = 0.8f; // Lowe's ratio test
        for (const auto &knnMatch : knnMatches)
        {
            if (knnMatch.size() == 2 && knnMatch[0].distance < ratioThreshold * knnMatch[1].distance)
            {
                matches.push_back(knnMatch[0]);
            }
        }
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        int bytes = 32; // length of the descriptor in bytes (16, 32, or 64)
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes);
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        int nfeatures = 500;                                  // number of features to retain
        float scaleFactor = 1.2f;                             // pyramid decimation ratio
        int nlevels = 8;                                      // number of pyramid levels
        int edgeThreshold = 31;                               // size of the border where features are not detected
        int firstLevel = 0;                                   // level of the pyramid to put the source image
        int WTA_K = 2;                                        // number of points to compare (2, 3, or 4)
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE; // Harris or FAST score
        int patchSize = 31;                                   // size of the patch used by the BRIEF descriptor
        int fastThreshold = 20;                               // FAST threshold
        extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        bool orientationNormalized = true; // enable orientation normalization
        bool scaleNormalized = true;       // enable scale normalization
        float patternScale = 22.0f;        // scaling of the description pattern
        int nOctaves = 4;                  // number of octaves covered by the detected keypoints
        extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves);
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        cv::AKAZE::DescriptorType descriptorType = cv::AKAZE::DESCRIPTOR_MLDB; // descriptor type
        int descriptorSize = 0;                                                // size of the descriptor
        int descriptorChannels = 3;                                            // number of channels in the descriptor
        float threshold = 0.001f;                                              // detector response threshold
        int nOctaves = 4;                                                      // maximum octave evolution
        int nOctaveLayers = 4;                                                 // default number of sublevels per scale level
        extractor = cv::AKAZE::create(descriptorType, descriptorSize, descriptorChannels, threshold, nOctaves, nOctaveLayers);
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        int nfeatures = 0;               // number of best features to retain
        int nOctaveLayers = 3;           // number of layers within each octave
        float contrastThreshold = 0.04; // filter out weak features
        float edgeThreshold = 10;       // filter out edge-like features
        float sigma = 1.6;              // Gaussian smoothing
        extractor = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    }
    else
    {
        throw invalid_argument("Unknown descriptor type: " + descriptorType);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    // Pointer to the detector
    cv::Ptr<cv::FeatureDetector> detector;

    // Choose the detector based on the input type
    if (detectorType == "FAST")
    {
        detector = cv::FastFeatureDetector::create();
    }
    else if (detectorType == "BRISK")
    {
        detector = cv::BRISK::create();
    }
    else if (detectorType == "ORB")
    {
        detector = cv::ORB::create();
    }
    else if (detectorType == "AKAZE")
    {
        detector = cv::AKAZE::create();
    }
    else if (detectorType == "SIFT")
    {
        detector = cv::SIFT::create();
    }
    else
    {
        std::cerr << "Detector type " << detectorType << " is not supported!" << std::endl;
        return;
    }

    // Detect keypoints
    detector->detect(img, keypoints);
}