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

    m_export_config = _export_config.get();
    configureOutput(m_export_config);

    return true;
}
bool EchoesToFrameConverterTask::startHook()
{
    if (!EchoesToFrameConverterTaskBase::startHook())
        return false;

    m_yaw_correction = base::Angle::unknown();
    m_current_range = base::unknown<float>();
    m_lut.reset(nullptr);
    m_frame_output_deadline = base::Time::now() + m_export_config.time_between_frames;

    return true;
}

void EchoesToFrameConverterTask::updateHook()
{
    EchoesToFrameConverterTaskBase::updateHook();

    base::Time now = base::Time::now();
    if (m_lut && now >= m_frame_output_deadline) {
        m_frame_output_deadline = now + m_export_config.time_between_frames;
        publishFrame();
    }
}

void EchoesToFrameConverterTask::echoCallback(const base::Time& ts,
    const radar_base::Radar& echo_sample)
{
    if (echo_sample.sweep_data.empty() || base::isUnknown(m_yaw_correction)) {
        return;
    }

    unsigned int current_sweep_size = echo_sample.sweep_length;
    unsigned int current_num_angles = 2 * M_PI / abs(echo_sample.step_angle.getRad());

    if (!m_lut) { // initialization
        updateLookUpTable(current_num_angles, current_sweep_size, m_export_config);
        m_current_range = echo_sample.range;
    }
    else {
        if (!m_lut->hasMatchingConfiguration(current_num_angles,
                current_sweep_size,
                m_export_config.beam_width,
                m_export_config.output_image_size)) {
            updateLookUpTable(current_num_angles, current_sweep_size, m_export_config);
        }

        if (m_current_range != echo_sample.range) {
            m_current_range = echo_sample.range;
            resetEchoMemory(current_sweep_size * current_num_angles);
        }
    }

    addEchoesToFrame(echo_sample, m_yaw_correction);
}

void EchoesToFrameConverterTask::sensor2ref_poseCallback(const base::Time& ts,
    const base::samples::RigidBodyState& sensor2ref_pose_sample)
{
    m_yaw_correction = base::Angle::fromRad(sensor2ref_pose_sample.getYaw());
}

void EchoesToFrameConverterTask::resetEchoMemory(std::size_t size)
{
    m_echoes.resize(size);
    m_echoes.assign(size, 0);
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

void EchoesToFrameConverterTask::updateLookUpTable(unsigned int num_sweeps,
    unsigned int sweep_size,
    RadarFrameExportConfig const& config)
{
    LOG_INFO_S << "Updating LookUpTable";
    m_lut.reset(new EchoToImageLUT(num_sweeps,
        sweep_size,
        config.beam_width,
        config.output_image_size));
    resetEchoMemory(num_sweeps * sweep_size);
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
    m_lut->drawImageFromEchoes(m_echoes, m_cv_frame);
    LOG_INFO_S << "Publishing Frame";
    cv::cvtColor(m_cv_frame, m_cv_frame_monochrome, cv::COLOR_BGR2GRAY);
    Frame* out_frame = m_output_frame.write_access();
    out_frame->time = base::Time::now();
    out_frame->received_time = out_frame->time;
    out_frame->setImage(m_cv_frame_monochrome.data,
        m_cv_frame_monochrome.total() * m_cv_frame_monochrome.elemSize());
    out_frame->setStatus(STATUS_VALID);
    m_output_frame.reset(out_frame);
    _frame.write(m_output_frame);
}

void EchoesToFrameConverterTask::configureOutput(RadarFrameExportConfig const& config)
{
    LOG_INFO_S << "Configuring output";
    m_cv_frame = Mat::zeros(config.output_image_size, config.output_image_size, CV_8UC3);
    m_cv_frame_monochrome =
        Mat::zeros(config.output_image_size, config.output_image_size, CV_8UC1);

    Frame* frame = new Frame(config.output_image_size,
        config.output_image_size,
        8U,
        base::samples::frame::frame_mode_t::MODE_GRAYSCALE,
        0,
        0);
    m_output_frame.reset(frame);
}