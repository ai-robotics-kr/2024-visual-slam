#include "rgbd_vio_node.hpp"
#include<opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdVioNode::RgbdVioNode()
:   Node("VIO"), m_VIO(nullptr)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "image_raw/right");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdVioNode::GrabRGBD, this);
}

RgbdVioNode::~RgbdVioNode()
{
    // Stop all threads
    m_VIO->Shutdown();

    // Save camera trajectory
    // m_VIO->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdVioNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    cv::Mat gray_RGB, gray_Depth;
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        cv::cvtColor(cv_ptrRGB->image, gray_RGB, cv::COLOR_RGB2GRAY);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
        cv::cvtColor(cv_ptrRGB->image, gray_Depth, cv::COLOR_RGB2GRAY);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    if (m_VIO) {
        m_VIO->Step(gray_RGB, gray_Depth);
    } else {
        RCLCPP_ERROR(this->get_logger(), "VIO is not set");
    }
}