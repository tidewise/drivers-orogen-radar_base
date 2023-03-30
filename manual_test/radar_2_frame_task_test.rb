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
            syskit_write task.echo_port, @echo_part2
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
        task.properties.radar_frame_export_config = {
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
        data = []
        samples.times { times_array << time }
        data = pattern1 + pattern2
        (samples / 2).times { data.concat(data) }
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
            sweep_data: data
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
            sweep_data: data
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
            sweep_data: data
        }

        task
    end
    # rubocop: enable Metrics/AbcSize

    def create_configure_and_start_task
        task = create_task
        syskit_configure_and_start(task)
        task
    end
end
