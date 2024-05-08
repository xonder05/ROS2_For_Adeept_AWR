#include <gpiod.h>
#include <atomic>
#include <thread>
#include <chrono>

class PWMGenerator {
public:
    ~PWMGenerator() {
        stop();
    }

    void initialize(struct gpiod_line *input_line) {
        line = input_line;
    }

    void start() {
        stop_flag = false;
        pwm_thread = std::thread(&PWMGenerator::generate_pwm, this);
    }

    void stop() {
        stop_flag = true;
        if (pwm_thread.joinable()) {
            pwm_thread.join();
        }
    }

    void set_duty_cycle(int duty_cycle_percent) {
        duty_cycle = static_cast<int>(period * (static_cast<float>(duty_cycle_percent) / 100.0));
    }

private:
    int period = 1000; //micro seconds
    int duty_cycle = 0;

    struct gpiod_line *line;
    std::atomic<bool> stop_flag{false};
    std::thread pwm_thread;

    void generate_pwm() {
        while (!stop_flag) {
            if (duty_cycle != 0) {
                gpiod_line_set_value(line, 1);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(duty_cycle));

            gpiod_line_set_value(line, 0);
            std::this_thread::sleep_for(std::chrono::microseconds(period - duty_cycle));
        }
    }
};
