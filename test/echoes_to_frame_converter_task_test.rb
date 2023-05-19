# frozen_string_literal: true

using_task_library "radar_base"
import_types_from "radar_base"
import_types_from "base"

describe OroGen.radar_base.EchoesToFrameConverterTask do
    run_live

    it "starts and outputs single radar data" do
        task = create_configure_task
        syskit_start(task)

        output = expect_execution do
            syskit_write task.echo_port, @echo
        end.to do
            have_one_new_sample(task.frame_port)
        end
        expected = File.binread(File.join(__dir__, "image1.bin"))

        assert_equal expected, output.image.to_a.to_s,
                     "single radar data output differs from expected image"
    end

    it "starts and outputs multiple radar data for a single frame" do
        task = create_configure_task
        task.poll do
            syskit_write task.echo_port, @echo_part1
            syskit_write task.echo_port, @echo_part2
        end
        syskit_start(task)

        output = expect_execution.to do
            have_one_new_sample(task.frame_port)
        end
        expected = File.binread(File.join(__dir__, "image2.bin"))

        assert_equal expected, output.image.to_a.to_s,
                     "multiple radar data output differs from expected image"
    end

    it "starts and outputs radar data with a negative stepsize" do
        task = create_configure_task
        task.poll do
            syskit_write task.echo_port, @echo_inverted
        end
        syskit_start(task)

        output = expect_execution.to do
            have_one_new_sample(task.frame_port)
        end
        expected = File.binread(File.join(__dir__, "image1.bin"))

        assert_equal expected, "#{output.image.to_a}",
                     "single radar data output differs from expected image"
    end

    it "it rotates sample 90 degrees 5 times" do
        task = create_configure_task
        syskit_start(task)

        arrow = []
        64.times { arrow.concat [0] }
        arrow[0..7] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation[:sweep_data] = arrow
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(1, 0, 0, 0)

        syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        output1 = expect_execution do
            syskit_write task.echo_port, @echo_rotation
        end.to do
            have_one_new_sample(task.frame_port)
        end

        arrow[0..7] = [0, 0, 0, 0, 0, 0, 0, 0]
        arrow[48..55] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation[:sweep_data] = arrow
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(0.7071068, 0, 0, 0.7071068)

        syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        output2 = expect_execution do
            syskit_write task.echo_port, @echo_rotation
        end.to do
            have_one_new_sample(task.frame_port)
        end

        arrow[48..55] = [0, 0, 0, 0, 0, 0, 0, 0]
        arrow[32..39] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation[:sweep_data] = arrow
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(0, 0, 0, 1)

        syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        output3 = expect_execution do
            syskit_write task.echo_port, @echo_rotation
        end.to do
            have_one_new_sample(task.frame_port)
        end

        arrow[32..39] = [0, 0, 0, 0, 0, 0, 0, 0]
        arrow[16..23] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation[:sweep_data] = arrow
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(-0.7071068, 0, 0, 0.7071068)

        syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        output4 = expect_execution do
            syskit_write task.echo_port, @echo_rotation
        end.to do
            have_one_new_sample(task.frame_port)
        end

        arrow[16..23] = [0, 0, 0, 0, 0, 0, 0, 0]
        arrow[0..7] = [0, 255, 0, 255, 0, 255, 0, 255]
        @echo_rotation[:sweep_data] = arrow
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(1, 0, 0, 0)

        syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        output5 = expect_execution do
            syskit_write task.echo_port, @echo_rotation
        end.to do
            have_one_new_sample(task.frame_port)
        end
        assert_equal output1.image.to_a, output2.image.to_a, "image 1 and 2 differ"
        assert_equal output1.image.to_a, output3.image.to_a, "image 1 and 3 differ"
        assert_equal output1.image.to_a, output4.image.to_a, "image 1 and 4 differ"
        assert_equal output1.image.to_a, output5.image.to_a, "image 1 and 5 differ"
    end

    # rubocop: disable Metrics/AbcSize
    def create_task
        task = syskit_deploy(
            OroGen.radar_base
                  .EchoesToFrameConverterTask
                  .deployed_as("radar_base_radar_2_frame_task")
        )
        samples = 4
        sweep_length = 8
        task.properties.export_config = {
            time_between_frames: Time.at(0.0001),
            output_image_size: 512,
            beam_width: 1 / samples * 2 * Math::PI
        }
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
        @echo = Types.radar_base.Radar.new
        @echo = {
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
        }

        @echo_inverted = Types.radar_base.Radar.new
        @echo_inverted = {
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
        }

        @echo_part1 = Types.radar_base.Radar.new
        @echo_part1 = {
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
        }

        @echo_part2 = Types.radar_base.Radar.new
        @echo_part2 = {
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
        }
        @echo_rotation = Types.radar_base.Radar.new
        @echo_rotation = {
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
        }

        @sensor2ref_pose = Types.base.samples.RigidBodyState.Invalid
        @sensor2ref_pose.sourceFrame = "world"
        @sensor2ref_pose.targetFrame = "radar"
        @sensor2ref_pose.time = time
        @sensor2ref_pose.position = Eigen::Vector3.new(0, 0, 0)
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(0.7071068, 0, 0, 0.7071068)
        task
    end
    # rubocop: enable Metrics/AbcSize

    def create_configure_task
        task = create_task
        syskit_configure(task)
        task
    end
end
