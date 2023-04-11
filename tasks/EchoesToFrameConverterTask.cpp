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
    m_last_sample = base::Time::now();
    return true;
}
void EchoesToFrameConverterTask::updateHook()
{
    EchoesToFrameConverterTaskBase::updateHook();

    double yaw_correction = 0;
    base::samples::RigidBodyState sensor2ref_pose;
    if (_sensor2ref_pose.read(sensor2ref_pose) != RTT::NoData) {
        yaw_correction = sensor2ref_pose.getYaw();
    }


    Radar radar_echo;
    while (_echo.read(radar_echo, false) == RTT::NewData) {
        if (m_current_sweep_size != radar_echo.sweep_length ||
            m_current_num_angles != (2 * M_PI / radar_echo.step_angle.getRad()) ||
            m_current_range != radar_echo.range) {
            m_current_sweep_size = radar_echo.sweep_length;
            m_current_num_angles = 2 * M_PI / radar_echo.step_angle.getRad();
            m_current_range = radar_echo.range;
            updateLookUpTable();
            m_echoes.clear();
        }

        addEchoesToFrame(radar_echo, yaw_correction);
    }
    if (base::Time::now() - m_last_sample > _export_config.get().time_between_frames) {
        publishFrame();
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
        config.window_size));
}

void EchoesToFrameConverterTask::addEchoesToFrame(Radar const& echo,
    double yaw_correction)
{
    LOG_INFO_S << "Adding Echoes to frame";
    double angle = echo.start_heading.getRad() + yaw_correction;
    if (angle < 0) {
        angle = 2 * M_PI + angle;
    }

    int start_angle_unit = round(angle / echo.step_angle.getRad());
    int angles_in_a_frame = 2 * M_PI / echo.step_angle.getRad();
    LOG_INFO_S << "ANGLE MESSAGE-> start: " << start_angle_unit
               << "YAW Correction: " << yaw_correction
               << " end: " << start_angle_unit + echo.sweep_timestamps.size()
               << " Sweep length: " << echo.sweep_length;
    for (int current_angle = 0;
         current_angle < static_cast<int>(echo.sweep_timestamps.size());
         current_angle++) {
        LOG_INFO_S << "ANGLE LOOP-> Index: " << current_angle << " angle message: "
                   << start_angle_unit + current_angle % angles_in_a_frame << "/"
                   << angles_in_a_frame << " sweep: " << current_angle * echo.sweep_length
                   << "/" << (current_angle + 1) * echo.sweep_length
                   << " total sweep length: " << echo.sweep_data.size();

        m_echoes[(start_angle_unit + current_angle) % angles_in_a_frame] =
            std::make_unique<std::vector<uint8_t>>(echo.sweep_data.begin() +
                                                       current_angle * echo.sweep_length,
                echo.sweep_data.begin() + (current_angle + 1) * echo.sweep_length);
    }
}

int EchoesToFrameConverterTask::discretizeAngle(double theta_rad, int num_angles)
{
    while (theta_rad < 0) {
        theta_rad += 2 * M_PI;
    }
    double angle_step = 2 * M_PI / num_angles;
    return round(theta_rad / angle_step);
}

void EchoesToFrameConverterTask::publishFrame()
{

    LOG_INFO_S << "Adding echoes into a frame...";
    for (const auto& [angle, sweep] : m_echoes) {
        for (int echo_index = 0; echo_index < m_current_sweep_size; echo_index++) {
            m_lut->updateImage(m_cv_frame,
                (angle) % m_current_num_angles,
                echo_index,
                sweep->at(echo_index));
        }
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
    _range.write(m_current_range);
    m_last_sample = base::Time::now();
    m_cv_frame = 0;
}

void EchoesToFrameConverterTask::configureOutput()
{
    LOG_INFO_S << "Configuring output";
    auto config = _export_config.get();
    m_cv_frame = Mat::zeros(config.window_size, config.window_size, CV_8UC3);

    Frame* frame = new Frame(config.window_size,
        config.window_size,
        8U,
        base::samples::frame::frame_mode_t::MODE_GRAYSCALE,
        0,
        0);
    m_output_frame.reset(frame);
}