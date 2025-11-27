#include "controller.h"
#include <godot_cpp/classes/engine.hpp>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

/*
 * Register Godot-exposed methods
 */
void Controller::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("start", "port"), &Controller::start);
    ClassDB::bind_method(D_METHOD("stop"), &Controller::stop);

    ClassDB::bind_method(D_METHOD("get_axis_x"), &Controller::get_axis_x);
    ClassDB::bind_method(D_METHOD("get_axis_y"), &Controller::get_axis_y);
    ClassDB::bind_method(D_METHOD("get_buttons"), &Controller::get_buttons);
}

Controller::Controller()
{
    running = false;

    axis_x = 512;
    axis_y = 512;
    buttons = 0;

#ifdef _WIN32
    serial_handle = INVALID_HANDLE_VALUE;
#else
    serial_fd = -1;
#endif
}

Controller::~Controller()
{
    stop();
}

/*
 * Opens the serial port and starts the reading thread.
 */
bool Controller::start(const String &port)
{
    if (running)
        return true;

#ifdef _WIN32
    serial_handle = CreateFileA(
        port.utf8().get_data(),
        GENERIC_READ,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL);

    if (serial_handle == INVALID_HANDLE_VALUE)
    {
        ERR_PRINT("Failed to open COM port");
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

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 1;
    SetCommTimeouts(serial_handle, &timeouts);

#else
    serial_fd = open(port.utf8().get_data(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0)
    {
        ERR_PRINT("Failed to open serial port");
        return false;
    }

    struct termios tty{};
    tcgetattr(serial_fd, &tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;

    tcsetattr(serial_fd, TCSANOW, &tty);
#endif

    running = true;
    read_thread = std::thread(&Controller::read_loop, this);
    read_thread.detach();

    return true;
}

/*
 * Stops the serial thread and closes the port.
 */
void Controller::stop()
{
    running = false;

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

/*
 * Background thread: reads 6-byte binary packets.
 * Packet format:
 * [0] X low byte
 * [1] X high byte
 * [2] Y low byte
 * [3] Y high byte
 * [4] Button bitmask
 * [5] Unused/reserved
 */
void Controller::read_loop()
{
    uint8_t header = 0;
    uint8_t data[5];

    while (running)
    {

        // --------------------------------------------------
        // 1. Warten auf das Startbyte 0xAA
        // --------------------------------------------------
        bool got_header = false;
        while (!got_header && running)
        {
#ifdef _WIN32
            DWORD br = 0;
            ReadFile(serial_handle, &header, 1, &br, NULL);
            if (br == 1 && header == 0xAA)
            {
                got_header = true;
            }
            else
            {
                Sleep(1);
            }
#else
            int r = read(serial_fd, &header, 1);
            if (r == 1 && header == 0xAA)
            {
                got_header = true;
            }
            else
            {
                usleep(1000);
            }
#endif
        }

        if (!running)
            break;

        // --------------------------------------------------
        // 2. Die nächsten 5 Bytes einlesen (XL, XH, YL, YH, BTN)
        // --------------------------------------------------
        int received = 0;
        while (received < 5 && running)
        {
#ifdef _WIN32
            DWORD br = 0;
            ReadFile(serial_handle, data + received, 5 - received, &br, NULL);
            if (br > 0)
            {
                received += br;
            }
            else
            {
                Sleep(1);
            }
#else
            int r = read(serial_fd, data + received, 5 - received);
            if (r > 0)
            {
                received += r;
            }
            else
            {
                usleep(1000);
            }
#endif
        }

        if (!running)
            break;

        uint8_t xl = data[0];
        uint8_t xh = data[1];
        uint8_t yl = data[2];
        uint8_t yh = data[3];
        uint8_t btn = data[4];

        axis_x = xl | (xh << 8);
        axis_y = yl | (yh << 8);
        buttons = btn;
    }
}
