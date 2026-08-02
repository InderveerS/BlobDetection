#pragma once
#include <Arduino.h>

// ================================================================
// COPY of the Communicator class from the main robot project. Kept
// byte-compatible on purpose so both ends of the link behave identically -
// if you change one, change the other.
//
// Basic UART link to a serial peer. Messages are plain text terminated by
// '\n'. Receiving is fully non-blocking: poll() drains whatever bytes have
// arrived and reassembles complete lines, so it never stalls the control loop.
//
// Threading: drive this from ONE task only - HardwareSerial is not safe to
// touch concurrently from multiple tasks. On this board that task is the
// core-0 link task in vision_link.cpp; nothing else may call it.
// ================================================================
class Communicator {
    public:
        // uart        : which hardware UART to use, e.g. &Serial1 (NOT Serial,
        //               that's the USB debug port).
        // rxPin/txPin : this board's GPIOs. Wire this RX to the peer's TX and
        //               this TX to the peer's RX (crossed), with common GND.
        // baud        : must match the peer's UART baud.
        Communicator(HardwareSerial& uart, uint8_t rxPin, uint8_t txPin, uint32_t baud = 115200);

        void begin(); // call once in setup

        // Send one message; a trailing '\n' is added. Near-instant unless the
        // TX buffer fills, so keep messages short.
        void send(const char* msg);
        void send(const String& msg) { send(msg.c_str()); }

        // printf-style convenience:  sendf("X %d Y %d", x, y);
        //
        // Returns false and sends NOTHING if the formatted message would not
        // fit. That matters: a truncated line still gets a '\n' appended, so
        // the peer would see a malformed message that looks complete. Better to
        // drop it and have the caller notice.
        bool sendf(const char* fmt, ...);

        // Non-blocking: reads all available bytes into the line buffer. Returns
        // true when a COMPLETE new message just finished this call. Call often
        // (e.g. every control cycle). If several lines arrive between polls,
        // only the newest is kept - poll fast enough if you can't miss any.
        bool poll();

        // Valid after poll() returns true: the most recent complete message,
        // null-terminated, without the newline.
        const char* lastMessage() const { return mMessage; }

        // MUST match the main robot project's Communicator. It governs the
        // receive path as well as send: poll() drops any inbound line longer
        // than BUF_SIZE-1, so a peer with a bigger buffer can send messages
        // this end silently discards.
        static constexpr size_t BUF_SIZE = 256;

    private:
        HardwareSerial& mUart;
        uint8_t mRxPin;
        uint8_t mTxPin;
        uint32_t mBaud;

        char mBuffer[BUF_SIZE]; // line being assembled
        size_t mLen = 0;
        char mMessage[BUF_SIZE] = {0}; // last complete line
};
