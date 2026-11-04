#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) return -1;

    cv::Mat frame, gray, edges;
    std::cout << "Shape detection initialized. Press 'q' to exit." << std::endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 1. Preprocessing (Grayscale, Blur, and Canny Edges)
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5, 1.5);
        cv::Canny(gray, edges, 50, 150);

        // 2. Find Contours
        // This connects the raw white edge pixels into continuous boundary curves.
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 3. Analyze each contour to determine its shape
        for (size_t i = 0; i < contours.size(); i++) {
            
            // Filter out tiny contours to save processing power and ignore background noise
            if (cv::contourArea(contours[i]) < 1000) continue;

            // Approximate the contour to a polygon
            // This smooths out rough edge lines into distinct vertices (corners).
            std::vector<cv::Point> approx;
            double perimeter = cv::arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], approx, 0.02 * perimeter, true);

            std::string shapeName = "Unknown";
            
            // 4. Identify the shape based on the number of corners
            if (approx.size() == 3) {
                shapeName = "Triangle";
            } 
            else if (approx.size() == 4) {
                // Differentiate between a square and a rectangle using aspect ratio
                cv::Rect rect = cv::boundingRect(approx);
                float aspectRatio = (float)rect.width / rect.height;
                if (aspectRatio >= 0.95 && aspectRatio <= 1.05) {
                    shapeName = "Square";
                } else {
                    shapeName = "Rectangle";
                }
            } 
            else if (approx.size() == 5) {
                shapeName = "Pentagon";
            } 
            else if (approx.size() == 6) {
                shapeName = "Hexagon";
            }
            else if (approx.size() > 7) {
                // A circle approximated by a polygon will have many small vertices
                shapeName = "Circle";
            }

            // 5. Draw the results on the original color frame
            if (shapeName != "Unknown") {
                // Draw a green outline around the shape
                cv::drawContours(frame, contours, (int)i, cv::Scalar(0, 255, 0), 2);
                
                // Calculate the center of the shape to place the text label
                cv::Moments m = cv::moments(contours[i]);
                if (m.m00 != 0) {
                    cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
                    // Draw the shape name in blue text at the center
                    cv::putText(frame, shapeName, center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
                }
            }
        }

        cv::imshow("Drone Vision - Shape Detection", frame);

        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}