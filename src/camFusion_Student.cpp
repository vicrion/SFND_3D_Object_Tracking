
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> validMatches;
    std::vector<double> distances;

    // filter matches based on ROI
    for (const auto &match : kptMatches) {
        const auto &prevPt = kptsPrev[match.queryIdx].pt; // Keypoint in previous frame
        const auto &currPt = kptsCurr[match.trainIdx].pt; // Keypoint in current frame

        if (boundingBox.roi.contains(currPt)) {
            validMatches.push_back(match);
            distances.push_back(cv::norm(currPt - prevPt));
        }
    }

    // compute robust mean and remove outliers
    if (!distances.empty()) {
        double meanDist = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        double distSum = 0.0;
        for (double d : distances) {
            distSum += (d - meanDist) * (d - meanDist);
        }
        double stddev = std::sqrt(distSum / distances.size());
        std::cout << "CAMERA: Extracted mean=" <<  std::to_string(meanDist) << ", stddev=" << std::to_string(stddev) << ".\n";

        // Keep matches with distance within standard deviations
        for (size_t i = 0; i < validMatches.size(); ++i) {
            if (std::fabs(distances[i] - meanDist) <=  1.5 * stddev) {
                boundingBox.kptMatches.push_back(validMatches[i]);
            }
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    std::vector<double> ratios;

    // Step 1: Compute distance ratios between keypoints
    for (size_t i = 0; i < kptMatches.size(); ++i) {
        const cv::KeyPoint &kpOuterPrev = kptsPrev[kptMatches[i].queryIdx];
        const cv::KeyPoint &kpOuterCurr = kptsCurr[kptMatches[i].trainIdx];

        for (size_t j = i + 1; j < kptMatches.size(); ++j) {
            const cv::KeyPoint &kpInnerPrev = kptsPrev[kptMatches[j].queryIdx];
            const cv::KeyPoint &kpInnerCurr = kptsCurr[kptMatches[j].trainIdx];

            // Compute distances
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);

            // Avoid division by zero
            if (distPrev > 1e-6 && distCurr > 1e-6) {
                double ratio = distCurr / distPrev;
                ratios.push_back(ratio);
            }
        }
    }

    // Step 2: Handle outliers using the median of ratios
    if (ratios.empty()) {
        std::cerr << "A set of ratios is empty.\n";
        TTC = NAN;
        return;
    }

    std::nth_element(ratios.begin(), ratios.begin() + ratios.size() / 2, ratios.end());
    double medianRatio = ratios[ratios.size() / 2];

    // Step 3: Compute TTC
    if (medianRatio <= 1e-6) {
        TTC = NAN; // Avoid invalid TTC
        std::cerr << "Median ration is too small.\n";
        return;
    } 
    
    TTC = -1.0 / (frameRate * (1.0 - medianRatio));

    std::cout << "CAMERA: num ratios=" << std::to_string(ratios.size()) << ", median(ratio)=" << std::to_string(medianRatio) << ", TTC=" << std::to_string(TTC) << "\n";
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // lidar distances for the closest points in X direction (object in front)
    std::vector<double> prevDistances, currDistances;
    for (const auto &point : lidarPointsPrev) {
        if (point.x > 0.0) // points in front of the sensor
            prevDistances.push_back(point.x);
    }
    for (const auto &point : lidarPointsCurr) {
        if (point.x > 0.0) // points in front of the sensor
            currDistances.push_back(point.x);
    }

    if (prevDistances.empty() || currDistances.empty()) {
        TTC = NAN;
        std::cerr << "Warning: Not enough Lidar points to compute TTC." << std::endl;
        return;
    }

    // median distances
    std::nth_element(prevDistances.begin(), prevDistances.begin() + prevDistances.size() / 2, prevDistances.end());
    double d0 = prevDistances[prevDistances.size() / 2];

    std::nth_element(currDistances.begin(), currDistances.begin() + currDistances.size() / 2, currDistances.end());
    double d1 = currDistances[currDistances.size() / 2];

    // division by zero or invalid TTC values
    if (d0 <= 0 || d1 <= 0 || std::fabs(d0 - d1) < 1e-6) {
        TTC = NAN;
        std::cerr << "Warning: Invalid Lidar distances to compute TTC." << std::endl;
        return;
    }

    // TTC using constant velocity model
    TTC = d1 / (frameRate * (d0 - d1));

    std::cout << "LIDAR: median(prev)=" << std::to_string(d0) << ", median(curr)=" << std::to_string(d1) << ", TTC=" << std::to_string(TTC) << "\n";
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // map to store the counts of keypoint matches between bounding box pairs
    std::map<std::pair<int, int>, int> bbMatchCounts;

    // for all keypoint matches
    for (const auto &match : matches) 
    {
        // indices of the matched keypoints
        int prevIdx = match.queryIdx; 
        int currIdx = match.trainIdx; 

        // store the bounding box IDs for the matched keypoints
        std::vector<int> prevBBIds, currBBIds;

        // which bounding boxes in the previous frame contain the keypoint
        for (const auto &prevBB : prevFrame.boundingBoxes) 
        {
            if (prevBB.roi.contains(prevFrame.keypoints[prevIdx].pt)) 
            {
                prevBBIds.push_back(prevBB.boxID);
            }
        }

        // which bounding boxes in the current frame contain the keypoint
        for (const auto &currBB : currFrame.boundingBoxes) 
        {
            if (currBB.roi.contains(currFrame.keypoints[currIdx].pt)) 
            {
                currBBIds.push_back(currBB.boxID);
            }
        }

        // inc match counts for all valid bounding box pairs
        for (int prevBBId : prevBBIds) 
        {
            for (int currBBId : currBBIds) 
            {
                bbMatchCounts[std::make_pair(prevBBId, currBBId)]++;
            }
        }
    }

    // id the best matches
    std::map<int, int> bestMatches; // tmp map to store the best match for each previous bounding box
    for (const auto &prevBB : prevFrame.boundingBoxes) 
    {
        int prevBBId = prevBB.boxID;
        int bestMatchId = -1;
        int maxCount = 0;

        for (const auto &currBB : currFrame.boundingBoxes) 
        {
            int currBBId = currBB.boxID;
            int count = bbMatchCounts[std::make_pair(prevBBId, currBBId)];

            if (count > maxCount) 
            {
                maxCount = count;
                bestMatchId = currBBId;
            }
        }

        // Save the best match if it exists
        if (bestMatchId != -1) 
        {
            bbBestMatches[prevBBId] = bestMatchId;
        }
    }
}
