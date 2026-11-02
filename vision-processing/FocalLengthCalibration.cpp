#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

// --- Configuration ---
// Adjust these to match the real-world object you are holding up for calibration!
// For example, a standard piece of US Letter paper held horizontally is 11 inches wide.
const double KNOWN_WIDTH = 11.25; // physical width of your object (e.g., inches)
const double KNOWN_DISTANCE = 58.5; // distance from camera during calibration (e.g., inches)

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

// The magic formula: Distance = (Known Width * Focal Length) / Perceived Pixel Width
double calculateDistance(double knownWidth, double focalLength, double perWidthPixels) {
    return (knownWidth * focalLength) / perWidthPixels;
}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera." << std::endl;
        return -1;
    }

    cv::Mat frame, gray, blur, thresh;
    double focalLength = 0.0; // 0 means not calibrated yet

    std::cout << "--- Focal Length & Distance Calibration ---" << std::endl;
    std::cout << "1. Hold an object that is " << KNOWN_WIDTH << " units wide." << std::endl;
    std::cout << "2. Place it exactly " << KNOWN_DISTANCE << " units away from the camera lens." << std::endl;
    std::cout << "3. Press 'c' to Calibrate the focal length." << std::endl;
    std::cout << "4. Press 'q' to quit." << std::endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Basic preprocessing to find the outline of your object
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blur, cv::Size(7, 7), 0);
        cv::Canny(blur, thresh, 50, 150);
        
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::dilate(thresh, thresh, kernel, cv::Point(-1, -1), 1);

        std::vector<std::vector<cv::Point>> contours;
        // Using RETR_LIST so it finds ALL contours (including the inside of the square)
        // instead of just the outermost boundaries.
        cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        cv::Point2f frameCenter(frame.cols / 2.0f, frame.rows / 2.0f);
        double minDistanceToCenter = 1e9;
        cv::Rect bestBox;
        bool found = false;

        // Find the square closest to the center of the screen
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            
            if (area > 1000) {
                std::string shape = classifyShape(contour);

                // Use the robust advanced vision logic to make sure it's actually a square/rectangle
                if (shape == "Square" || shape == "Rectangle") {
                    cv::Moments m = cv::moments(contour);
                    if (m.m00 != 0) {
                        // Find the center of this shape
                        cv::Point2f shapeCenter(m.m10 / m.m00, m.m01 / m.m00);
                        
                        // Calculate how far it is from the center of the camera frame
                        double dist = cv::norm(shapeCenter - frameCenter);

                        if (dist < minDistanceToCenter) {
                            minDistanceToCenter = dist;
                            bestBox = cv::boundingRect(contour);
                            found = true;
                        }
                    }
                }
            }
        }

        if (found) {
            // Draw the bounding box
            cv::rectangle(frame, bestBox, cv::Scalar(0, 255, 0), 2);
            
            // Show perceived pixel width
            std::string widthText = "Width (px): " + std::to_string(bestBox.width);
            cv::putText(frame, widthText, cv::Point(bestBox.x, bestBox.y - 10), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            // If we have a focal length, calculate and show the real-time distance!
            if (focalLength > 0) {
                double distance = calculateDistance(KNOWN_WIDTH, focalLength, bestBox.width);
                std::string distText = "Distance: " + std::to_string(distance).substr(0, 5) + " units";
                cv::putText(frame, distText, cv::Point(30, 50), 
                            cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 100, 0), 2);
            } else {
                cv::putText(frame, "Press 'c' to lock Focal Length!", cv::Point(30, 50), 
                            cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }
        }

        cv::imshow("Focal Length Calibration", frame);
        cv::imshow("Threshold Mask", thresh); // Helpful window for debugging lighting and edges

        char key = (char)cv::waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 'c' && found) {
            // Focal Length = (Pixels x Distance) / Width
            focalLength = (bestBox.width * KNOWN_DISTANCE) / KNOWN_WIDTH;
            std::cout << "\n[SUCCESS] Calibrated!" << std::endl;
            std::cout << "Perceived Width: " << bestBox.width << " px" << std::endl;
            std::cout << "Your Camera's Focal Length: " << focalLength << std::endl;
            std::cout << "Move the object backward and forward to see the distance update!" << std::endl;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
