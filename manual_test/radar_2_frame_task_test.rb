# frozen_string_literal: true

using_task_library "radar_base"
import_types_from "radar_base"
import_types_from "base"

describe OroGen.radar_base.Radar2FrameTask do
    run_live

    it "starts and outputs radar data" do
        task = create_configure_and_start_task
        sleep(2)
        expect_execution do
            syskit_write task.echo_port, @echo
        end.to do
            have_one_new_sample(task.frame_port)
        end

        # pp output
        puts "Does the output looks OK ?"
        ask_ok
    end

    it "starts and outputs multiple radar data for a single frame" do
        task = create_configure_and_start_task
        sleep(2)
        expect_execution do
            syskit_write task.echo_port, @echo_part1
            sleep(0.5)
            syskit_write task.echo_port, @echo_part2
        end.to do
            have_one_new_sample(task.frame_port)
        end

        puts "Does the output looks OK ?"
        ask_ok
    end

    it "it rotates sample 90 degrees" do
        task = create_task
        task.properties.export_config.use_heading_correction = true
        syskit_configure_and_start(task)
        sleep(2)
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(1, 0, 0, 0)
        expect_execution do
            syskit_write task.echo_port, @echo_rotation
            syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        end.to do
            have_one_new_sample(task.frame_port)
        end
        sleep(0.5)
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(0.7071068, 0, 0, 0.7071068)
        expect_execution do
            syskit_write task.echo_port, @echo_rotation
            syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        end.to do
            have_one_new_sample(task.frame_port)
        end
        sleep(0.5)
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(0, 0, 0, 1)
        expect_execution do
            syskit_write task.echo_port, @echo_rotation
            syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        end.to do
            have_one_new_sample(task.frame_port)
        end
        sleep(0.5)
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(0.7071068, 0, 0, -0.7071068)
        expect_execution do
            syskit_write task.echo_port, @echo_rotation
            syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        end.to do
            have_one_new_sample(task.frame_port)
        end
        sleep(0.5)
        @sensor2ref_pose.orientation = Eigen::Quaternion.new(1, 0, 0, 0)
        expect_execution do
            syskit_write task.echo_port, @echo_rotation
            syskit_write task.sensor2ref_pose_port, @sensor2ref_pose
        end.to do
            have_one_new_sample(task.frame_port)
        end
        puts "Does the output looks OK ?"
        ask_ok
    end

    def ask_ok
        puts "OK ? (y/n)"
        value = STDIN.readline.chomp
        raise "test failed" unless value == "y"
    end

    # rubocop: disable Metrics/AbcSize
    def create_task
        task = syskit_deploy(
            OroGen.radar_base
                  .Radar2FrameTask
                  .deployed_as("radar_base_radar_2_frame_task")
        )
        samples = 4
        sweep_length = 8
        task.properties.export_config = {
            time_between_frames: Time.at(0.2),
            use_heading_correction: false,
            window_size: 1024,
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
        data = pattern1 + pattern2
        (samples / 2).times { input.concat(data) }
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
        arrow = []
        4.times { arrow.concat [255, 0] }
        56.times { arrow.concat [0] }
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
            sweep_timestamps: times_array*2,
            sweep_data: arrow
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

    def create_configure_and_start_task
        task = create_task
        syskit_configure_and_start(task)
        task
    end
end
