#ifndef DISPLAY_H
#define DISPLAY_H

#include <godot_cpp/classes/ref_counted.hpp>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

using namespace godot;

class Display : public RefCounted {
    GDCLASS(Display, RefCounted)

private:
#ifdef _WIN32
    void *serial_handle;
#else
    int serial_fd;
#endif

    bool connected;
    
    void send_packet(uint8_t command, const uint8_t* data, uint8_t length);

protected:
    static void _bind_methods();

public:
    Display();
    ~Display();

    bool connect_display(const String &port);
    void disconnect_display();
    
    void show_text(const String &text);
    void clear();
    void set_brightness(int level);
};

#endif