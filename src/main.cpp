#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

struct TrackedPoint
{
    cv::Point2d lastPosition{};
    double distance{0};
    bool isTracked{true};
};

struct MultiTracker
{
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect2d ROI;
};

int main()
{
    cv::Mat frame;

    std::string video = "../video/translation.mp4";
    cv::VideoCapture cap(video);

    cap >> frame;

    //Tracker initialization
    constexpr unsigned int roiSize = 30;

    auto findCenter = [](const cv::Rect2d& rect) -> cv::Point2d
    { return (rect.br() + rect.tl()) * 0.5; };

    cv::Mat gray_frame;

    constexpr unsigned int nbTrackersMax = 1000;
    std::vector<cv::Point2d> corners;

    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(gray_frame, corners, nbTrackersMax, 0.05, 4, cv::Mat(), 7, false, 0.04);

    // I didn't use cv::MultiTracker because I needed to know if the tracking is lost
    std::vector<MultiTracker> multiTracker{};

    for(const auto& c : corners)
    {
        multiTracker.push_back(MultiTracker{/*.tracker =*/ cv::TrackerMOSSE::create(),
                /*.ROI =*/ cv::Rect2d(cv::Point2d(c.x - roiSize / 2., c.y - roiSize / 2.), cv::Size2i(roiSize, roiSize))});
        multiTracker.back().tracker->init(frame, multiTracker.back().ROI);
    }

    std::cout << "Start the tracking process, press ESC to quit." << std::endl;

    const std::size_t nbTrackers = corners.size();

    //centers is a linearized array containing all centers of trackers for all frames
    std::vector<cv::Point2d> centers{};
    auto ROIsCenters = std::vector<cv::Point2d>(nbTrackers);
    auto distances = std::vector<double>(nbTrackers, .0);
    auto trackedPoints = std::vector<TrackedPoint>(nbTrackers);

    cv::Mat lastFrame;

    for(;;)
    {
        cap >> frame;

        if(frame.empty())
            break;

        lastFrame = frame.clone();

        for(int i = 0; i < multiTracker.size(); i++)
        {
            if(!multiTracker[i].tracker->update(frame, multiTracker[i].ROI))
                trackedPoints[i].isTracked = false;
            ROIsCenters[i] = findCenter(multiTracker[i].ROI);
        }

        double anglesSum = 0;
        double distSum = 0;
        unsigned int nbPoints = 0;

        centers.insert(centers.end(), ROIsCenters.begin(), ROIsCenters.end());
        for(std::size_t i = 0; i < centers.size() - nbTrackers; i += nbTrackers)
        {
            //Take only the last frame centers
            anglesSum = 0;
            distSum = 0;

            for(std::size_t j = 0; j < nbTrackers; j++)
            {
                const auto& P1 = centers[i + j];
                const auto& P2 = centers[i + nbTrackers + j];
                if(P1 != P2)
                    cv::line(frame, P1, P2, cv::Scalar(0, 255, 0), 2, 1);

                trackedPoints[j].distance += cv::norm(P1 - P2);
                trackedPoints[j].lastPosition = P1;

                if(trackedPoints[j].isTracked)
                {
                    distSum += trackedPoints[j].distance;
                    anglesSum += std::atan((P2.x - P1.x + P1.x) / (P2.y - P1.y + P1.y));
                    nbPoints++;
                }
            }
        }

        double angleRad = anglesSum / nbPoints;
        double avgDist = distSum / nbPoints;
        std::cout << avgDist << std::endl;
        std::cout << angleRad * (180.0 / 3.141592653589793238463) << " deg / " << angleRad << " rad" << std::endl;
        auto P1Arrow = cv::Point2d(50, 50);
        unsigned int length = int(avgDist) * 5;
        auto P2Arrow = cv::Point2d(P1Arrow.x + length * std::cos(angleRad), P1Arrow.y + length * std::sin(angleRad));
        cv::arrowedLine(frame, P1Arrow, P2Arrow, cv::Scalar(0, 0, 255), 2, 1);


        cv::imshow("tracker", frame);

        //quit on ESC button
        if(cv::waitKey(1) == 27)
            break;

    }

    std::sort(trackedPoints.begin(), trackedPoints.end(), [](const TrackedPoint& p1, const TrackedPoint& p2)
    { return p1.distance < p2.distance; });

    //TODO: if the last point is not tracked, it may be a problem
    double distanceMax = trackedPoints.back().distance;

    for(const auto& trackedPoint : trackedPoints)
    {
        if(trackedPoint.isTracked)
        {
            //Todo: by using linear interpolation, any distance from the camera can be found
            unsigned int gray = int((trackedPoint.distance / distanceMax) * 255);
            cv::circle(lastFrame, trackedPoint.lastPosition, roiSize / 4, cv::Scalar(gray, gray, gray), -1, 1);
        }
    }

    imshow("depth", lastFrame);

    if(cv::waitKey() == 27)
        return 0;
    return 0;
}