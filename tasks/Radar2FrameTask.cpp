/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Radar2FrameTask.hpp"
#include <base-logging/Logging.hpp>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace radar_base;
using namespace cv;
using namespace base::samples::frame;
Radar2FrameTask::Radar2FrameTask(std::string const& name)
    : Radar2FrameTaskBase(name)
{
}

Radar2FrameTask::~Radar2FrameTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Radar2FrameTask.hpp for more detailed
// documentation about them.

bool Radar2FrameTask::configureHook()
{
    if (!Radar2FrameTaskBase::configureHook())
        return false;
    configureOutput();
    updateLookUpTable();
    return true;
}
bool Radar2FrameTask::startHook()
{
    if (!Radar2FrameTaskBase::startHook())
        return false;
    return true;
}
void Radar2FrameTask::updateHook()
{
    Radar2FrameTaskBase::updateHook();
    Radar radar_echo;
    if (_echo.read(radar_echo, false) != RTT::NewData) {
        LOG_INFO_S << "No data";
        return;
    }

    if (m_current_sweep_size != radar_echo.sweep_length ||
        m_current_num_angles != (2 * M_PI / radar_echo.step_angle.getRad()) ||
        m_current_range != radar_echo.range) {
        m_current_sweep_size = radar_echo.sweep_length;
        m_current_num_angles = 2 * M_PI / radar_echo.step_angle.getRad();
        m_current_range = radar_echo.range;
        updateLookUpTable();
        m_cv_frame = 0;
        m_number_of_echoes_collected = 0;
    }

    addEchoesToFrame(radar_echo);
}
void Radar2FrameTask::errorHook()
{
    Radar2FrameTaskBase::errorHook();
}
void Radar2FrameTask::stopHook()
{
    Radar2FrameTaskBase::stopHook();
}
void Radar2FrameTask::cleanupHook()
{
    Radar2FrameTaskBase::cleanupHook();
}

void Radar2FrameTask::updateLookUpTable()
{
    LOG_INFO_S << "Updating LookUpTable";
    auto config = _radar_frame_export_config.get();
    m_lut.reset(new EchoToImageLUT(m_current_num_angles,
        m_current_sweep_size,
        config.beam_width,
        config.window_size));
}

void Radar2FrameTask::addEchoesToFrame(Radar const& echo)
{
    LOG_INFO_S << "Adding Echoes to frame";
    int start_angle_unit = echo.start_heading.getRad() / echo.step_angle.getRad();
    int angles_in_a_frame = 2 * M_PI / echo.step_angle.getRad();
    LOG_INFO_S << "Start angle unit: " << start_angle_unit << "angles in a frame "
               << angles_in_a_frame;
    for (int current_angle = 0;
         current_angle < static_cast<int>(echo.sweep_timestamps.size());
         current_angle++) {
        for (int echo_index = 0; echo_index < echo.sweep_length; echo_index++) {
            m_lut->updateImage(m_cv_frame,
                (start_angle_unit + current_angle) % angles_in_a_frame,
                echo_index,
                echo.sweep_data[current_angle * echo.sweep_length + echo_index]);
        }
        m_number_of_echoes_collected++;
        if (m_number_of_echoes_collected >= 2 * M_PI / echo.step_angle.getRad()) {
            publishFrame();
        }
    }
}

void Radar2FrameTask::publishFrame()
{
    LOG_INFO_S << "Publishing Frame";
    cv::cvtColor(m_cv_frame, m_cv_frame, cv::COLOR_BGR2GRAY);
    Frame* out_frame = m_output_frame.write_access();
    out_frame->time = base::Time::now();
    out_frame->received_time = out_frame->time;
    out_frame->setImage(m_cv_frame.data, m_cv_frame.total() * m_cv_frame.elemSize());
    out_frame->setStatus(STATUS_VALID);
    m_output_frame.reset(out_frame);
    _frame.write(m_output_frame);
    _range.write(m_current_range);

    m_number_of_echoes_collected = 0;
    m_cv_frame = 0;
}

void Radar2FrameTask::configureOutput()
{
    LOG_INFO_S << "Configuring output";
    auto config = _radar_frame_export_config.get();
    m_cv_frame = Mat::zeros(config.window_size, config.window_size, CV_8UC3);

    Frame* frame = new Frame(config.window_size,
        config.window_size,
        8U,
        base::samples::frame::frame_mode_t::MODE_GRAYSCALE,
        0,
        0);
    m_output_frame.reset(frame);
}