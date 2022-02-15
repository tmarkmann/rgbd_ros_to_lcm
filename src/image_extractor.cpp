//#include <ros/ros.h>
#include <boost/filesystem.hpp>

//#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <rgbd_ros_to_lcm/jpeg_utils.h>

#include <zlib.h>

#include <lcm/lcm-cpp.hpp>
#include <bot_core/image_t.hpp>
#include <bot_core/images_t.hpp>


class ImageExtractor
{
    std::string lcm_logfile_path_;
    std::string lcm_channel_;
    std::string output_folder_;

    int width;
    int height;
    int num_pixels_;

    unsigned char * depth_read_buf_;
    unsigned char * image_read_buf_;
    int32_t depth_size_;
    int32_t image_size_;

public:
  ImageExtractor() : depth_read_buf_(0),
                     image_read_buf_(0)
  {
    std::string lcm_logfile_path_ = "/root/labelfusion/data/raw/logs2/trimmed_log.lcmlog";
    std::string output_folder_ = "/root/labelfusion/data/raw/logs2/extracted";
    std::string lcm_channel_ = "OPENNI_FRAME";

    boost::filesystem::create_directories(output_folder_);

    lcm::LogFile* logfile = new lcm::LogFile(lcm_logfile_path_, "r");

    const lcm::LogEvent* event = nullptr;
    int i = 0;
    while (true)
    {
        event = logfile->readNextEvent();
        if (event == nullptr) {
            //$ reached the end of the log
            break;
        }
        if (event->channel == lcm_channel_) {
            //$ only consider events that are on the desired channel
            bot_core::images_t message;
            message.decode(event->data, 0, event->datalen);
            if (i == 0) {
                initializeDimensions(message);
            }
            std::cout << "Save image: " << i << std::endl;
            decompressAndSaveImages(message);
            i++;
        }
    }
  }

  ~ImageExtractor()
  {
  }

  void initializeDimensions(bot_core::images_t message)
  {
    bot_core::image_t color_image = message.images[0];
    width = color_image.width;
    height = color_image.height;
    num_pixels_ = width * height;
    depth_read_buf_ = new unsigned char[num_pixels_ * 2];
    image_read_buf_ = new unsigned char[num_pixels_ * 3];
  }

  void decompressAndSaveImages(bot_core::images_t message)
  {
    int64_t timestamp {message.utime};
    std::cout << "Timestamp: " << timestamp << std::endl;

    bot_core::image_t color_image = message.images[0];
    bot_core::image_t depth_image = message.images[1];

    depth_size_ = depth_image.size;
    image_size_ = color_image.size;

    if (depth_size_ == num_pixels_ * 2)
    {
        //$ copy uncompressed depth image
        memcpy(&depth_read_buf_[0], depth_image.data.data(), num_pixels_ * 2);
    }
    else
    {
        //$ decompress zlib compressed depth
        unsigned long decomp_length = num_pixels_ * 2;
        uncompress(&depth_read_buf_[0], (unsigned long *)&decomp_length, (const Bytef *)depth_image.data.data(), depth_size_);
    }

    if (image_size_ == num_pixels_ * 3)
    {
        //$ copy uncompressed rgb image
        memcpy(&image_read_buf_[0], color_image.data.data(), num_pixels_ * 3);
    }
    else if (image_size_ > 0)
    {
        //$ decompress jpeg compressed rgb image
        size_t w = color_image.width;
        size_t h = color_image.height;
        std::cout << "Decompress width: " << w << std::endl;
        jpegijg_decompress_8u_rgb(color_image.data.data(), color_image.size, (unsigned char *)&image_read_buf_[0], w, h, w*3);
    }

    std::cout << "Save image"<< std::endl;
    cv::Mat image_mat(height, width, CV_8UC3, &image_read_buf_[0]); 
    //$ flip RGB -> BGR channel order for saving with OpenCV
    cv::cvtColor(image_mat, image_mat, CV_RGB2BGR); 

    cv::imshow("image", image_mat);
    waitKey(30);    

    cv::Mat test(cv::Size(1, 49), CV_64FC1, Scalar(0));
    std::string test_name = output_folder_ + "/" + std::to_string(timestamp) + "_test.png";
    cv::imwrite(test_name, test);

    std::string image_name = output_folder_ + "/" + std::to_string(timestamp) + "_image.png";
    cv::imwrite(image_name, image_mat);

    std::string depth_name = output_folder_ + "/" + std::to_string(timestamp) + "_depth.png";
    cv::Mat depth_mat(height, width, CV_16UC1, &depth_read_buf_[0]); 
    cv::imwrite(depth_name, depth_mat);
    std::cout << "Image done"<< std::endl;
  }

};

int main(int argc, char** argv)
{
  ImageExtractor extractor;
}
