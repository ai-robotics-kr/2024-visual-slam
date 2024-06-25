#include "rgbd_vio_node.hpp"
#include<opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdVioNode::RgbdVioNode(VisualOdometry* vio)
:   Node("VIO"),
    m_VIO(vio)
{
    m_VIO->Init();

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(std::shared_ptr<rclcpp::Node>(this), "image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(std::shared_ptr<rclcpp::Node>(this), "image_raw/right");

    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(std::shared_ptr<rclcpp::Node>(this), "left/image_raw_color");
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(std::shared_ptr<rclcpp::Node>(this), "right/image_raw_color");

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
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
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
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    m_VIO->Step(cv_ptrRGB->image, cv_ptrD->image);
}