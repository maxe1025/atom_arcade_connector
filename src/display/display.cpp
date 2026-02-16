#include "display.h"
#include <godot_cpp/classes/engine.hpp>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

const uint8_t CMD_HEADER = 0xBB;
const uint8_t CMD_SHOW_TEXT = 0x01;
const uint8_t CMD_CLEAR = 0x02;
const uint8_t CMD_SET_BRIGHTNESS = 0x03;

void Display::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("connect_display", "port"), &Display::connect_display);
    ClassDB::bind_method(D_METHOD("disconnect_display"), &Display::disconnect_display);
    ClassDB::bind_method(D_METHOD("show_text", "text"), &Display::show_text);
    ClassDB::bind_method(D_METHOD("clear"), &Display::clear);
    ClassDB::bind_method(D_METHOD("set_brightness", "level"), &Display::set_brightness);
}

Display::Display()
{
    connected = false;
#ifdef _WIN32
    serial_handle = INVALID_HANDLE_VALUE;
#else
    serial_fd = -1;
#endif
}

Display::~Display()
{
    disconnect_display();
}

bool Display::connect_display(const String &port)
{
    if (connected)
        return true;

#ifdef _WIN32
    serial_handle = CreateFileA(
        port.utf8().get_data(),
        GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL);

    if (serial_handle == INVALID_HANDLE_VALUE)
    {
        ERR_PRINT("Failed to open COM port for display");
        return false;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(serial_handle, &dcbSerialParams);

    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.StopBits = ONESTOPBIT;

    SetCommState(serial_handle, &dcbSerialParams);

#else
    serial_fd = open(port.utf8().get_data(), O_WRONLY | O_NOCTTY);
    if (serial_fd < 0)
    {
        ERR_PRINT("Failed to open serial port for display");
        return false;
    }

    struct termios tty{};
    tcgetattr(serial_fd, &tty);

    cfsetospeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tcsetattr(serial_fd, TCSANOW, &tty);
#endif

    connected = true;
    return true;
}

void Display::disconnect_display()
{
    connected = false;

#ifdef _WIN32
    if (serial_handle != INVALID_HANDLE_VALUE)
    {
        CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;
    }
#else
    if (serial_fd >= 0)
    {
        close(serial_fd);
        serial_fd = -1;
    }
#endif
}

void Display::send_packet(uint8_t command, const uint8_t* data, uint8_t length)
{
    if (!connected)
        return;

    // Build packet: [HEADER][COMMAND][LENGTH][DATA...][CHECKSUM]
    uint8_t packet[258];
    packet[0] = CMD_HEADER;
    packet[1] = command;
    packet[2] = length;
    
    if (length > 0 && data != nullptr)
    {
        memcpy(&packet[3], data, length);
    }
    
    uint8_t checksum = 0;
    for (int i = 1; i < (3 + length); i++)
    {
        checksum += packet[i];
    }
    packet[3 + length] = checksum & 0xFF;
    
    int total_length = 4 + length;

#ifdef _WIN32
    DWORD written;
    WriteFile(serial_handle, packet, total_length, &written, NULL);
#else
    write(serial_fd, packet, total_length);
#endif
}

void Display::show_text(const String &text)
{
    CharString utf8 = text.utf8();
    send_packet(CMD_SHOW_TEXT, (const uint8_t*)utf8.get_data(), utf8.length());
}

void Display::clear()
{
    send_packet(CMD_CLEAR, nullptr, 0);
}

void Display::set_brightness(int level)
{
    if (level < 0) level = 0;
    if (level > 15) level = 15;
    
    uint8_t brightness = (uint8_t)level;
    send_packet(CMD_SET_BRIGHTNESS, &brightness, 1);
}