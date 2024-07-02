#include "rgbd_vio_node.hpp"
#include<opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdVioNode::RgbdVioNode()
:   Node("VIO"), m_VIO(nullptr)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "image_raw/right");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "image_raw/right");
    std::cout << "make image subscriber instance" << std::endl;

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdVioNode::GrabRGBD, this);
    std::cout << "register callback" << std::endl;
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
    std::cout << "vio before step" << std::endl;
    if (m_VIO) {
        m_VIO->Step(cv_ptrRGB->image, cv_ptrD->image);
    } else {
        RCLCPP_ERROR(this->get_logger(), "VIO is not set");
    }
    std::cout << "vio after step" << std::endl;
}