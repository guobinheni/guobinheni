#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <unistd.h> // For close, read, write, etc.
#include <fcntl.h>  // For open
#include <termios.h> // For termios struct and functions
#include <thread>
#include <chrono>

class LEDScreenController {
public:
    LEDScreenController(const std::string& port, unsigned int baud_rate)
        : serial_port(open(port.c_str(), O_RDWR | O_NOCTTY)) {
        if (serial_port == -1) {
            throw std::runtime_error("Error occurred while opening the serial port: " + port);
        }

        set_serial_port_options(baud_rate);
    }

    ~LEDScreenController() {
        ::close(serial_port);
    }

    void async_send_display_message() {
        auto now = std::chrono::system_clock::now();
        if (last_person_detected_time != nullptr && now - *last_person_detected_time < std::chrono::seconds(3)) {
            return; // Send message only if person was detected less than 3 seconds ago
        }

        std::thread t([this](){
            static const uint8_t display_data[] = {/* 显示指令数据 */};
            write_to_serial_port(display_data);
            last_person_detected_time = now;
        });
        t.detach();
    }

    void async_send_clear_message() {
        auto now = std::chrono::system_clock::now();
        if (last_person_detected_time == nullptr || now - *last_person_detected_time >= std::chrono::seconds(3)) {
            std::thread t([this](){
                static const uint8_t clear_data[] = {/* 清屏指令数据 */};
                write_to_serial_port(clear_data);
                last_person_detected_time = now;
            });
            t.detach();
        }
    }

private:
    int serial_port;
    std::chrono::time_point<std::chrono::system_clock> last_person_detected_time{nullptr};

    void set_serial_port_options(unsigned int baud_rate) {
        struct termios options;
        tcgetattr(serial_port, &options); // Get current options

        cfsetispeed(&options, baud_rate); // Set input baud rate
        cfsetospeed(&options, baud_rate); // Set output baud rate

        options.c_cflag &= ~PARENB; // No parity bit
        options.c_cflag &= ~CSTOPB; // One stop bit
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8; // Eight data bits

        options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

        options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non-canonical mode

        tcsetattr(serial_port, TCSANOW, &options); // Apply new settings immediately
    }

    void write_to_serial_port(const uint8_t* data, size_t length) {
        ssize_t bytes_written = write(serial_port, data, length);
        if (bytes_written != static_cast<ssize_t>(length)) {
            std::cerr << "Failed to write all data to the serial port. Wrote " << bytes_written << " out of " << length << " bytes." << std::endl;
        }
    }
};

int main(int argc, char* argv[]) {
    if (argc > 1) {
        LEDScreenController controller("/dev/ttyUSB0", 9600);
        controller.async_send_display_message();
    } else {
        std::cerr << "No message provided. Defaulting to 'Person detected!'.\n";
        LEDScreenController controller("/dev/ttyUSB0", 9600);
        controller.async_send_display_message();
    }

    return 0;
}