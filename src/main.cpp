#include <librealsense/rs.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <atomic>
#include <map>
#include <cmath>

#include "concurrency.hpp"

double fx = 275.538; double inv_fx = 1.0f / fx;
double fy = 255.714; double inv_fy = 1.0f / fy;
double cx = 324.254;
double cy = 235.932;
double fov = 0.923872;

void UndistortImage(cv::Mat &img, cv::Mat &undist_img)
{
    // iterate over all the pixels
    for(int y = 0; y < img.rows; y++)
        for(int x = 0; x < img.cols; x++)
        {
            double X = (x - cx) * inv_fx;
            double Y = (y - cy) * inv_fy;
            double r_d = sqrt(X*X + Y*Y);
            double r_u = tan(r_d*fov)/(2*tan(0.5*fov));
            double X_u = r_u * X / r_d;
            double Y_u = r_u * Y / r_d;
            double x_u = X_u * fx + cx;
            double y_u = Y_u * fy + cy;
            if((int)y_u < 480 && (int)y_u >= 0 
            && (int)x_u < 640 && (int)x_u >= 0)
                undist_img.at<uchar>((int)y_u, (int)x_u) = img.at<uchar>(y, x);
        }

    // 

}

int main()
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
 
    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    const int streams = 5;
    single_consumer_queue<rs::frame> frame_buffer[streams];

    // store the fisheye value on the buffer
    std::atomic<bool> running(true);

    struct resolution
    {
        int width;
        int height;
        rs::format format;
    };

    std::map<rs::stream, resolution> resolutions;

    // set fisheye frame callback function
    int index = (int)rs::stream::fisheye;
    dev->set_frame_callback(rs::stream::fisheye, [dev, &running, &frame_buffer, &resolutions, index](rs::frame frame)
    {
        if(running)
            frame_buffer[(int)rs::stream::fisheye].enqueue(std::move(frame));
    });
    
    // Enable IMU and Fisheye stream
    dev->enable_stream(rs::stream::fisheye, rs::preset::best_quality);

    // get the stream image resolution
    resolutions[rs::stream::fisheye] = {dev->get_stream_width(rs::stream::fisheye), dev->get_stream_height(rs::stream::fisheye)};

    dev->start();

    std::cout << "Start Capture Image" << std::endl;

    // iteratively capture the image data and convert to opencv image format
    while(cv::waitKey(1) != ' ')
    {
        rs::frame frame;

        auto res = resolutions[rs::stream::fisheye];

        // the realsense fisheye image format are raw10
        cv::Mat image(res.height, res.width, CV_8UC1);

        //  
        if(frame_buffer[(int)rs::stream::fisheye].try_dequeue(&frame))
        {
            // convert the data to opencv image
            std::memcpy(image.data, frame.get_data(), res.height*res.width*sizeof(char));
            cv::imshow("New_Image", image);
            cv::waitKey(1);

            cv::Mat undist_img = image.clone();
            UndistortImage(image, undist_img);
            cv::imshow("Undistorted_Image", undist_img);
            cv::waitKey(1);
        }

    }

    dev->stop();

    if(dev->is_stream_enabled(rs::stream::fisheye))
        dev->disable_stream(rs::stream::fisheye);

    return 0;
}