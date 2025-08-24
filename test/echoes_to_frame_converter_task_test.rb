# frozen_string_literal: true

require "syskit/aggregator/test_helpers"

using_task_library "radar_base"
import_types_from "radar_base"
import_types_from "base"

DEG2RAD = Math::PI / 180

describe OroGen.radar_base.EchoesToFrameConverterTask do
    run_live

    include Syskit::Aggregator::TestHelpers

    attr_reader :task
    before do
        @task = create_configure_task
    end

    it "starts and outputs single radar data" do
        syskit_start(task)

        write_pose(task, @sensor2ref_pose)
        output = write_echo(task, @echo)

        expected = File.binread(File.join(__dir__, "image1.bin"))

        assert_equal expected, output.image.to_a.to_s,
                     "single radar data output differs from expected image"
    end

    it "starts and outputs multiple radar data for a single frame" do
        syskit_start(task)

        write_pose(task, @sensor2ref_pose)
        now = Time.now
        write_echo(task, @echo_part1, now)
        output = write_echo(task, @echo_part2, now + 1)

        expected = File.binread(File.join(__dir__, "image2.bin"))

        assert_equal expected, output.image.to_a.to_s,
                     "multiple radar data output differs from expected image"
    end

    it "starts and outputs radar data with a negative stepsize" do
        syskit_start(task)
        write_pose(task, @sensor2ref_pose)
        output = write_echo(task, @echo_inverted)
        expected = File.binread(File.join(__dir__, "image1.bin"))

        assert_equal expected, output.image.to_a.to_s,
                     "single radar data output differs from expected image"
    end

    it "it rotates sample 90 degrees 5 times" do
        syskit_start(task)

        arrow = []
        64.times { arrow.concat [0] }
        arrow[0..7] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation.sweep_data = arrow
        @sensor2ref_pose.orientation =
            Eigen::Quaternion.from_angle_axis(0, Eigen::Vector3.UnitZ)

        write_pose(task, @sensor2ref_pose)
        output1 = write_echo(task, @echo_rotation)

        arrow[0..7] = [0, 0, 0, 0, 0, 0, 0, 0]
        arrow[48..55] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation.sweep_data = arrow
        @sensor2ref_pose.orientation =
            Eigen::Quaternion.from_angle_axis(90 * DEG2RAD, Eigen::Vector3.UnitZ)

        write_pose(task, @sensor2ref_pose)
        output2 = write_echo(task, @echo_rotation)

        arrow[48..55] = [0, 0, 0, 0, 0, 0, 0, 0]
        arrow[32..39] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation.sweep_data = arrow
        @sensor2ref_pose.orientation =
            Eigen::Quaternion.from_angle_axis(180 * DEG2RAD, Eigen::Vector3.UnitZ)

        write_pose(task, @sensor2ref_pose)
        output3 = write_echo(task, @echo_rotation)

        arrow[32..39] = [0, 0, 0, 0, 0, 0, 0, 0]
        arrow[16..23] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation.sweep_data = arrow
        @sensor2ref_pose.orientation =
            Eigen::Quaternion.from_angle_axis(270 * DEG2RAD, Eigen::Vector3.UnitZ)

        write_pose(task, @sensor2ref_pose)
        output4 = write_echo(task, @echo_rotation)

        arrow[16..23] = [0, 0, 0, 0, 0, 0, 0, 0]
        arrow[0..7] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation.sweep_data = arrow
        @sensor2ref_pose.orientation =
            Eigen::Quaternion.from_angle_axis(0, Eigen::Vector3.UnitZ)

        write_pose(task, @sensor2ref_pose)
        output5 = write_echo(task, @echo_rotation)

        assert_equal output1.image.to_a, output2.image.to_a, "image 1 and 2 differ"
        assert_equal output1.image.to_a, output3.image.to_a, "image 1 and 3 differ"
        assert_equal output1.image.to_a, output4.image.to_a, "image 1 and 4 differ"
        assert_equal output1.image.to_a, output5.image.to_a, "image 1 and 5 differ"
    end

    def create_task # rubocop:disable Metrics/AbcSize, Metrics/MethodLength
        task = syskit_deploy(
            OroGen.radar_base
                  .EchoesToFrameConverterTask
                  .deployed_as("radar_base_radar_2_frame_task")
        )
        samples = 4
        sweep_length = 8
        task.properties.export_config = {
            time_between_frames: Time.at(0),
            output_image_size: 512,
            beam_width: 1 / samples * 2 * Math::PI
        }
        task.properties.stream_aligner_status_period = 0
        task.properties.echo_period = 0.02
        task.properties.sensor2ref_pose_period = 0.01
        pattern = [0, 255]
        pattern1 = []
        pattern2 = []
        (sweep_length / 2).times { pattern1.concat(pattern) }
        (sweep_length / 2).times { pattern2.concat(pattern.reverse) }
        time = Time.now
        times_array = []
        samples.times { times_array << time }
        data = []
        input = []
        input_inverted = []
        data = pattern1 + pattern2
        (samples / 2).times { input.concat(data) }
        (samples / 2).times { input_inverted.concat(pattern2 + pattern1) }
        @echo = Types.radar_base.Radar.new(
            timestamp: Time.now,
            range: 2.0,
            step_angle: {
                rad: 2 * Math::PI / samples
            },
            start_heading: {
                rad: 0
            },
            sweep_length: sweep_length,
            sweep_timestamps: times_array,
            sweep_data: input
        )

        @echo_inverted = Types.radar_base.Radar.new(
            timestamp: Time.now,
            range: 2.0,
            step_angle: {
                rad: -2 * Math::PI / samples
            },
            start_heading: {
                rad: 0
            },
            sweep_length: sweep_length,
            sweep_timestamps: times_array,
            sweep_data: input
        )

        @echo_part1 = Types.radar_base.Radar.new(
            timestamp: Time.now,
            range: 2.0,
            step_angle: {
                rad: 2 * Math::PI / (2 * samples)
            },
            start_heading: {
                rad: 0
            },
            sweep_length: sweep_length,
            sweep_timestamps: times_array,
            sweep_data: input
        )

        @echo_part2 = Types.radar_base.Radar.new(
            timestamp: Time.now,
            range: 2.0,
            step_angle: {
                rad: 2 * Math::PI / (2 * samples)
            },
            start_heading: {
                rad: Math::PI
            },
            sweep_length: sweep_length,
            sweep_timestamps: times_array,
            sweep_data: input
        )
        @echo_rotation = Types.radar_base.Radar.new(
            timestamp: Time.now,
            range: 2.0,
            step_angle: {
                rad: 2 * Math::PI / (2 * samples)
            },
            start_heading: {
                rad: 0
            },
            sweep_length: sweep_length,
            sweep_timestamps: times_array * 2,
            sweep_data: []
        )

        @sensor2ref_pose = Types.base.samples.RigidBodyState.Invalid
        @sensor2ref_pose.sourceFrame = "world"
        @sensor2ref_pose.targetFrame = "radar"
        @sensor2ref_pose.time = time
        @sensor2ref_pose.position = Eigen::Vector3.new(0, 0, 0)
        @sensor2ref_pose.orientation =
            Eigen::Quaternion.from_angle_axis(0, Eigen::Vector3.UnitZ)
        task
    end

    def create_configure_task
        task = create_task
        syskit_configure(task)
        task
    end

    def write_pose(task, pose)
        pose.time = Time.now
        stream_aligner_write(
            task,
            task.sensor2ref_pose_port,
            pose,
            sample_time_field: "time"
        )
    end

    def write_echo(task, echo, time = Time.now)
        echo.timestamp = time
        stream_aligner_write(
            task,
            task.echo_port,
            echo,
            sample_time_field: "timestamp"
        ) do
            have_one_new_sample task.frame_port
        end
    end
end
