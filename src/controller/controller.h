#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <thread>
#include <atomic>

using namespace godot;

class Controller : public RefCounted {
    GDCLASS(Controller, RefCounted)

private:
#ifdef _WIN32
    void *serial_handle;
#else
    int serial_fd;
#endif

    std::thread read_thread;
    std::atomic<bool> running;

    std::atomic<int> axis_x;
    std::atomic<int> axis_y;
    std::atomic<int> buttons;

    void read_loop();


protected:
    static void _bind_methods();

public:
    Controller();
    ~Controller();

    bool start(const String &port = "/dev/ttyACM0");
    void stop();

    int get_axis_x() const { return axis_x.load(); }
    int get_axis_y() const { return axis_y.load(); }
    int get_buttons() const { return buttons.load(); }
};

#endif