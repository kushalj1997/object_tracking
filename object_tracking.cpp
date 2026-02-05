#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

class ObjectTracker {
    public:
        ObjectTracker();
        ~ObjectTracker();
        void track(const std::vector<cv::Mat>& frames);
    private:
        cv::Ptr<cv::Tracker> tracker;
};

int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
