#include <librealsense/rs.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <atomic>
#include <map>
#include <cmath>
#include <string>

#include "concurrency.hpp"

double fx = 275.538; double inv_fx = 1.0f / fx;
double fy = 255.714; double inv_fy = 1.0f / fy;
double cx = 324.254;
double cy = 235.932;
double fov = 0.923872;

// forward method
// the result show that there are many errors
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

// backward method
// use the billinear interplation methods
void UndistortImageWithInterplation(cv::Mat &img, cv::Mat &undist_img)
{
    for(int y = 0; y < undist_img.rows;y++)
        for(int x = 0; x < undist_img.cols; x++)
        {
            double X_u = (x - cx) * inv_fx;
            double Y_u = (y - cy) * inv_fy;

            double r_u = sqrt(X_u*X_u+Y_u*Y_u);
            double r_d = 1.0/fov * atan(2*r_u*tan(0.5*fov));

            double X = r_d * X_u / r_u;
            double Y = r_d * Y_u / r_u;

            double x_u = X*fx + cx;
            double y_u = Y*fy + cy; 

            if(y_u < 480 - 1 && y_u > 0 
            && x_u < 640 - 1 && x_u > 0)
            {
                // bilinear interplation
                int x_low = (int)x_u;
                int y_low = (int)y_u;
                int x_up = x_low+1;
                int y_up = y_low+1;

                double inter1 = (x_u - x_low) * img.at<uchar>(y_low, x_low) + 
                                (x_up - x_u) * img.at<uchar>(y_low, x_up) ;
                double inter2 = (x_u - x_low) * img.at<uchar>(y_up, x_low) + 
                                (x_up - x_u) * img.at<uchar>(y_up, x_up) ;

                double res = (y_u - y_low) * inter1 + (y_up - y_u) * inter2;
                undist_img.at<uchar>(y, x) = (unsigned char)res;
            }
        }
} 

// 
std::mutex mm_mutex;
rs::motion_data m_gyro_data;
rs::motion_data m_acc_data;

void on_motion_event(rs::motion_data entry)
{
    std::lock_guard<std::mutex> lock(mm_mutex);
    if(entry.timestamp_data.source_id == RS_EVENT_IMU_ACCEL)
    {   
        std::cout << "New Accelerator Data: " << "TimeStamp is: " << entry.timestamp_data.timestamp << " " << 
         entry.axes[0] << " " << entry.axes[1] << " " << entry.axes[2] << std::endl;
        m_acc_data = entry;
    }
    if(entry.timestamp_data.source_id == RS_EVENT_IMU_GYRO)
    {
        std::cout << "New Gyroscope Data: " << "TimeStamp is: " << entry.timestamp_data.timestamp << " "
        << entry.axes[0] << " " << entry.axes[1] << " " << entry.axes[2] << std::endl;
        m_gyro_data= entry;
    }
}

void on_timestamp_event(rs::timestamp_data entry)
{
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
    
    std::atomic<bool> running(true);
    
    // enable reading imu data
    bool has_motion_module = dev->supports(rs::capabilities::motion_events);
    if(has_motion_module)
    {
        std::cout << "This Device Support Motion Module!" << std::endl;
        dev->enable_motion_tracking(on_motion_event, on_timestamp_event);
    }
    else{
        std::cout << "This Device Does not Support Motion Module!" << std::endl;
    }

    const int streams = 5;
    single_consumer_queue<rs::frame> frame_buffer[streams];
    
    // Enable IMU and Fisheye stream
    dev->enable_stream(rs::stream::fisheye, rs::preset::best_quality);
    dev->set_option(rs::option::fisheye_strobe, 1);

    auto frame_callback = [dev, &running, &frame_buffer](rs::frame frame){
        std::cout << "Image Frame TimeStamps is " << frame.get_timestamp() << std::endl;
        frame_buffer[(int)rs::stream::fisheye].enqueue(std::move(frame));
    };

    dev->set_frame_callback(rs::stream::fisheye, frame_callback);
    
    dev->start(rs::source::all_sources);

    // std::cout << "Start Capture Image" << std::endl;

    // iteratively capture the image data and convert to opencv image format
    int saved_num = 0;
    while(cv::waitKey(1) != ' ')
    {
        rs::frame frame;

        // the realsense fisheye image format are raw10
        cv::Mat image(480, 640, CV_8UC1);

        //  
        if(frame_buffer[(int)rs::stream::fisheye].try_dequeue(&frame))
        {
            // convert the data to opencv image
            std::cout << "New Image Frame: " << "TimeStamps is: " << frame.get_timestamp() << std::endl;
            std::memcpy(image.data, frame.get_data(), 480 * 640 * sizeof(char));
            cv::imshow("New_Image", image);
            // presss key s to save image
            if(cv::waitKey(10) == 's'){
                saved_num++;
                std::string img_name = "image" + std::to_string(saved_num) + ".jpg";
                cv::imwrite(img_name, image);
                std::cout << img_name << " is saved!" << std::endl;
            }

            // cv::Mat undist_img(res.height, res.width, CV_8UC1);
            // UndistortImageWithInterplation(image, undist_img);
            // cv::imshow("Undistorted_Image", undist_img);
            // cv::waitKey(1);
        }

    }

    dev->stop(rs::source::all_sources);

    if(dev->is_stream_enabled(rs::stream::fisheye))
        dev->disable_stream(rs::stream::fisheye);

    if(has_motion_module)
    {
        dev->disable_motion_tracking();
    }

    return 0;
}