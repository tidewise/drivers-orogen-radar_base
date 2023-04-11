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
    configureOutput();
    updateLookUpTable();
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

    Radar radar_echo;
    if (_echo.read(radar_echo, false) == RTT::NewData) {
        base::Angle yaw_correction = base::Angle::fromRad(0);
        base::samples::RigidBodyState sensor2ref_pose;
        if (_sensor2ref_pose.read(sensor2ref_pose) != RTT::NoData) {
            yaw_correction = base::Angle::fromRad(sensor2ref_pose.getYaw());
        }

        do {
            if (m_current_sweep_size != radar_echo.sweep_length ||
                m_current_num_angles != (2 * M_PI / radar_echo.step_angle.getRad()) ||
                m_current_range != radar_echo.range) {
                m_current_sweep_size = radar_echo.sweep_length;
                m_current_num_angles = 2 * M_PI / radar_echo.step_angle.getRad();
                m_current_range = radar_echo.range;
                updateLookUpTable();
            }
            addEchoesToFrame(radar_echo, yaw_correction);
        } while (_echo.read(radar_echo, false) == RTT::NewData);

        if (base::Time::now() - m_last_sample >
            _export_config.get().time_between_frames) {
            publishFrame();
        }
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

void EchoesToFrameConverterTask::updateLookUpTable()
{
    LOG_INFO_S << "Updating LookUpTable";
    auto config = _export_config.get();
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
    double angle = (echo.start_heading + yaw_correction).getRad();
    if (angle < 0) {
        angle = 2 * M_PI + angle;
    }

    int start_angle_unit = round(angle / echo.step_angle.getRad());
    int angles_in_a_frame = round(2 * M_PI / echo.step_angle.getRad());
    for (int current_angle = 0;
         current_angle < static_cast<int>(echo.sweep_timestamps.size());
         current_angle++) {
        std::copy(echo.sweep_data.begin() + current_angle * echo.sweep_length,
            echo.sweep_data.begin() + (current_angle + 1) * echo.sweep_length,
            m_echoes.begin() + echo.sweep_length * ((start_angle_unit + current_angle) %
                                                       angles_in_a_frame));
    }
}

void EchoesToFrameConverterTask::publishFrame()
{
    LOG_INFO_S << "Creating a frame...";
    for (long i = 0; i < static_cast<long>(m_echoes.size()); i++) {
        m_lut->updateImage(m_cv_frame,
            i / m_current_sweep_size,
            i % m_current_sweep_size,
            m_echoes.at(i));
    }
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
    m_last_sample = base::Time::now();
    m_cv_frame = 0;
}

void EchoesToFrameConverterTask::configureOutput()
{
    LOG_INFO_S << "Configuring output";
    auto config = _export_config.get();
    m_cv_frame = Mat::zeros(config.output_image_size, config.output_image_size, CV_8UC3);

    Frame* frame = new Frame(config.output_image_size,
        config.output_image_size,
        8U,
        base::samples::frame::frame_mode_t::MODE_GRAYSCALE,
        0,
        0);
    m_output_frame.reset(frame);
}