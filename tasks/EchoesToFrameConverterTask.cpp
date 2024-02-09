/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "EchoesToFrameConverterTask.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace radar_base;
using namespace cv;
using namespace base::samples::frame;
EchoesToFrameConverterTask::EchoesToFrameConverterTask(std::string const& name)
    : EchoesToFrameConverterTaskBase(name)
{
}

EchoesToFrameConverterTask::~EchoesToFrameConverterTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See EchoesToFrameConverterTask.hpp for more detailed
// documentation about them.

bool EchoesToFrameConverterTask::configureHook()
{
    if (!EchoesToFrameConverterTaskBase::configureHook())
        return false;
    auto config = _export_config.get();
    configureOutput(config);
    updateLookUpTable(config);
    return true;
}
bool EchoesToFrameConverterTask::startHook()
{
    if (!EchoesToFrameConverterTaskBase::startHook())
        return false;
    return true;
    m_last_sample = base::Time::now();
}
void EchoesToFrameConverterTask::updateHook()
{
    EchoesToFrameConverterTaskBase::updateHook();
    auto config = _export_config.get();

    base::Angle yaw_correction = base::Angle::fromRad(0);
    base::samples::RigidBodyState sensor2ref_pose;
    if (_sensor2ref_pose.read(sensor2ref_pose) != RTT::NoData) {
        yaw_correction = base::Angle::fromRad(sensor2ref_pose.getYaw());
    }

    Radar radar_echo;
    while (_echo.read(radar_echo, false) == RTT::NewData) {
        m_current_sweep_size = radar_echo.sweep_length;
        m_current_num_angles = 2 * M_PI / abs(radar_echo.step_angle.getRad());
        if (!m_lut->hasMatchingConfiguration(m_current_num_angles,
                m_current_sweep_size,
                config.beam_width,
                config.output_image_size)) {
            updateLookUpTable(config);
        }
        if (m_current_range != radar_echo.range) {
            m_current_range = radar_echo.range;
            m_echoes =
                std::vector<uint8_t>(m_current_sweep_size * m_current_num_angles, 0);
        }
        addEchoesToFrame(radar_echo, yaw_correction);
    }

    auto deadline = m_last_sample + config.time_between_frames;
    if (base::Time::now() > deadline) {
        publishFrame();
        m_last_sample = base::Time::now();
    }
}

void EchoesToFrameConverterTask::errorHook()
{
    EchoesToFrameConverterTaskBase::errorHook();
}
void EchoesToFrameConverterTask::stopHook()
{
    EchoesToFrameConverterTaskBase::stopHook();
}
void EchoesToFrameConverterTask::cleanupHook()
{
    EchoesToFrameConverterTaskBase::cleanupHook();
}

void EchoesToFrameConverterTask::updateLookUpTable(RadarFrameExportConfig config)
{
    LOG_INFO_S << "Updating LookUpTable";
    m_lut.reset(new EchoToImageLUT(m_current_num_angles,
        m_current_sweep_size,
        config.beam_width,
        config.output_image_size));
    m_echoes = std::vector<uint8_t>(m_current_sweep_size * m_current_num_angles, 0);
}

void EchoesToFrameConverterTask::addEchoesToFrame(Radar const& echo,
    base::Angle yaw_correction)
{
    LOG_INFO_S << "Adding echoes...";
    Radar::updateEchoes(echo, yaw_correction, m_echoes);
}

void EchoesToFrameConverterTask::publishFrame()
{
    m_cv_frame = 0;
    LOG_INFO_S << "Creating a frame...";
    (*m_lut).drawImageFromEchoes(m_echoes, m_cv_frame);
    LOG_INFO_S << "Publishing Frame";
    Mat output;
    cv::cvtColor(m_cv_frame, output, cv::COLOR_BGR2GRAY);
    Frame* out_frame = m_output_frame.write_access();
    out_frame->time = base::Time::now();
    out_frame->received_time = out_frame->time;
    out_frame->setImage(output.data, output.total() * output.elemSize());
    out_frame->setStatus(STATUS_VALID);
    m_output_frame.reset(out_frame);
    _frame.write(m_output_frame);
}

void EchoesToFrameConverterTask::configureOutput(RadarFrameExportConfig config)
{
    LOG_INFO_S << "Configuring output";
    m_cv_frame = Mat::zeros(config.output_image_size, config.output_image_size, CV_8UC3);

    Frame* frame = new Frame(config.output_image_size,
        config.output_image_size,
        8U,
        base::samples::frame::frame_mode_t::MODE_GRAYSCALE,
        0,
        0);
    m_output_frame.reset(frame);
}