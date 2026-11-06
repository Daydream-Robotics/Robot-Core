// win_serial_io.hpp - SerialIO for a Windows COM port (PC testing only)
#pragma once
#include "rplidar_lite.hpp"
#include <windows.h>
#include <string>

class WinSerialIO : public SerialIO {
public:
    ~WinSerialIO() { close(); }

    // port: "COM15" or "\\\\.\\COM15"
    bool open(const std::string& port, uint32_t baud) {
        std::string path = port;
        if (path.rfind("\\\\.\\", 0) != 0) path = "\\\\.\\" + path;

        h_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                         OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE) return false;

        SetupComm(h_, 64 * 1024, 4096);

        DCB dcb = {};
        dcb.DCBlength = sizeof(dcb);
        GetCommState(h_, &dcb);
        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fParity = FALSE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        if (!SetCommState(h_, &dcb)) { close(); return false; }

        // Reads return immediately with whatever is already received.
        COMMTIMEOUTS t = {};
        t.ReadIntervalTimeout = MAXDWORD;
        t.WriteTotalTimeoutConstant = 200;
        SetCommTimeouts(h_, &t);

        PurgeComm(h_, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
        EscapeCommFunction(h_, CLRDTR);   // same as the Slamtec SDK does after opening
        return true;
    }

    void close() {
        if (h_ != INVALID_HANDLE_VALUE) { CloseHandle(h_); h_ = INVALID_HANDLE_VALUE; }
    }

    int write(const uint8_t* data, int len) override {
        DWORD written = 0;
        if (!WriteFile(h_, data, (DWORD)len, &written, nullptr)) return 0;
        return (int)written;
    }

    int read(uint8_t* buffer, int maxLen) override {
        DWORD got = 0;
        if (!ReadFile(h_, buffer, (DWORD)maxLen, &got, nullptr)) return 0;
        return (int)got;
    }

    uint32_t millis() override { return (uint32_t)GetTickCount64(); }
    void sleepMs(uint32_t ms) override { Sleep(ms); }
    void setDTR(bool asserted) override { EscapeCommFunction(h_, asserted ? SETDTR : CLRDTR); }

private:
    HANDLE h_ = INVALID_HANDLE_VALUE;
};
