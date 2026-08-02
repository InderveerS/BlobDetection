#include "communicator.hpp"
#include <stdarg.h>

Communicator::Communicator(HardwareSerial& uart, uint8_t rxPin, uint8_t txPin, uint32_t baud)
    : mUart(uart), mRxPin(rxPin), mTxPin(txPin), mBaud(baud) {}

void Communicator::begin() {
    // Bigger RX buffer than the 256-byte default so a burst from the peer
    // isn't dropped if poll() is briefly late. Must be set before begin().
    mUart.setRxBufferSize(512);
    mUart.begin(mBaud, SERIAL_8N1, mRxPin, mTxPin);
}

void Communicator::send(const char* msg) {
    mUart.print(msg);
    mUart.print('\n');
}

bool Communicator::sendf(const char* fmt, ...) {
    char buf[BUF_SIZE];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // n < 0 is an encoding error; n >= sizeof(buf) means it was truncated.
    // Sending a truncated line would look valid to the peer, so refuse.
    if (n < 0 || (size_t)n >= sizeof(buf)) return false;

    send(buf);
    return true;
}

bool Communicator::poll() {
    bool completed = false;
    while (mUart.available()) {
        char c = (char)mUart.read();

        if (c == '\n' || c == '\r') {
            if (mLen > 0) {                       // ignore empty lines / bare \r\n
                mBuffer[mLen] = '\0';
                memcpy(mMessage, mBuffer, mLen + 1);
                mLen = 0;
                completed = true;                 // keep draining; newest line wins
            }
        } else if (mLen < BUF_SIZE - 1) {
            mBuffer[mLen++] = c;
        } else {
            mLen = 0;                             // overrun: drop the too-long line
        }
    }
    return completed;
}
