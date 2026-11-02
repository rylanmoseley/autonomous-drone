#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

// --- Distance Calculation Constants ---
const double FOCAL_LENGTH = 566.8;
const double KNOWN_WIDTH = 3.0; // Assuming a standard 3x3 inch Post-it note

// Advanced classification using Circularity, Fill Ratio, and aspect ratio checks
std::string classifyShape(const std::vector<cv::Point>& contour) {
    double area = cv::contourArea(contour);
    if (area < 300) return "Unknown";

    double perim = cv::arcLength(contour, true);
    if (perim == 0) return "Unknown";

    double circularity = 4.0 * CV_PI * area / (perim * perim);

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);
    double circleArea = CV_PI * radius * radius;
    double fillRatio = (circleArea > 0) ? (area / circleArea) : 0;

    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * perim, true);
    size_t n = approx.size();

    if (n >= 7 && circularity > 0.75 && fillRatio > 0.7) {
        return "Circle";
    }

    if (n == 3) {
        return "Triangle";
    } else if (n == 4) {
        cv::Rect rect = cv::boundingRect(approx);
        float ar = (float)rect.width / rect.height;
        if (ar > 0.9 && ar < 1.1) return "Square";
        else return "Rectangle";
    } else if (n == 5) {
        cv::Rect rect = cv::boundingRect(approx);
        float ar = (float)rect.width / rect.height;
        if (ar > 0.7 && ar < 1.3 && circularity > 0.3) return "Pentagon";
    } else if (n == 6) {
        cv::Rect rect = cv::boundingRect(approx);
        float ar = (float)rect.width / rect.height;
        if (ar > 0.7 && ar < 1.3 && circularity > 0.3) return "Hexagon";
    }

    return "Unknown";
}

double calculateDistance(double knownWidth, double focalLength, double perWidthPixels) {
    return (knownWidth * focalLength) / perWidthPixels;
}

// Global variables for the color tuning trackbars
// These defaults are roughly in the range for pink/magenta in OpenCV's HSV scale (0-179 for Hue)
int hMin = 140, sMin = 50, vMin = 50;
int hMax = 175, sMax = 255, vMax = 255;

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera." << std::endl;
        return -1;
    }

    cv::Mat frame, hsv, mask;

    // Create a special window just for tuning the color ranges
    cv::namedWindow("Color Tuning Mask", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Hue Min", "Color Tuning Mask", &hMin, 179);
    cv::createTrackbar("Hue Max", "Color Tuning Mask", &hMax, 179);
    cv::createTrackbar("Sat Min", "Color Tuning Mask", &sMin, 255);
    cv::createTrackbar("Sat Max", "Color Tuning Mask", &sMax, 255);
    cv::createTrackbar("Val Min", "Color Tuning Mask", &vMin, 255);
    cv::createTrackbar("Val Max", "Color Tuning Mask", &vMax, 255);

    std::cout << "--- Color-Based Vision Tracking ---" << std::endl;
    std::cout << "Adjust the sliders in the 'Color Tuning Mask' window until ONLY your post-it note is white." << std::endl;
    std::cout << "Press 'q' to exit." << std::endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 1. Convert the frame from BGR to HSV Color Space
        // HSV separates color (Hue) from lighting/shadows (Value), making it MUCH better for color tracking
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 2. Threshold the image to ONLY keep pixels that fall within our slider ranges
        cv::Scalar lowerBound(hMin, sMin, vMin);
        cv::Scalar upperBound(hMax, sMax, vMax);
        cv::inRange(hsv, lowerBound, upperBound, mask);

        // Clean up the mask using morphological operations
        // Erode removes tiny random noise dots, Dilate grows the actual object back to its full size
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::erode(mask, mask, kernel);
        cv::dilate(mask, mask, kernel);

        // 3. Find Contours on the purely black-and-white mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 4. Identify the shapes of all valid color blobs in the frame
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            // Must be reasonably large to avoid tracking random pink specks
            if (area > 300) {
                std::string shapeName = classifyShape(contour);
                
                // Only draw and track if it matches a known shape
                if (shapeName != "Unknown") {
                    cv::Rect box = cv::boundingRect(contour);
                    
                    // Draw a bounding box around the tracked color object
                    cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 3);
                    cv::putText(frame, "Pink " + shapeName, cv::Point(box.x, box.y - 10), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

                    // Calculate and display real-time distance using your focal length
                    double distance = calculateDistance(KNOWN_WIDTH, FOCAL_LENGTH, box.width);
                    std::string distText = std::to_string(distance).substr(0, 5) + " units";
                    cv::putText(frame, distText, cv::Point(box.x, box.y + box.height + 20), 
                                cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 100, 0), 2);
                }
            }
        }

        // Display results
        cv::imshow("Color Tracker", frame);
        cv::imshow("Color Tuning Mask", mask); // Watch this window while adjusting the sliders!

        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
