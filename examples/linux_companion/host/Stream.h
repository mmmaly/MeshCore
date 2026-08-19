#pragma once
#include <cstdarg>
#include <cstdio>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Mock Stream class for native testing
// Provides minimal interface needed by Utils.h

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print
{
public:
    virtual size_t write(uint8_t b) { return 1; }
    size_t write(const char *str)
    {
        if(str == NULL) {
            return 0;
        }
        return write((const uint8_t *) str, strlen(str));
    }
    virtual size_t write(const uint8_t *buffer, size_t size) { 
        size_t t = 0;
        for (int i = 0; i < size; i++) { t += write(buffer[i]); }
        return t;
    }
    size_t write(const char *buffer, size_t size)
    {
        return write((const uint8_t *) buffer, size);
    }

    virtual size_t print(unsigned char b, int r = DEC) { return 0; }
    virtual size_t print(int v, int r = DEC) { return 0; }
    virtual size_t print(unsigned int v, int r = DEC) { return 0; }
    virtual size_t print(long v, int r = DEC) { return 0; }
    virtual size_t print(unsigned long v, int r = DEC) { return 0; }
    virtual size_t print(long long v, int r = DEC) { return 0; }
    virtual size_t print(unsigned long long v, int r = DEC) { return 0; }
    virtual size_t print(double v, int p = 2) { return 0; }

    size_t print(char c) { return write(c); }
    size_t print(const char* str) { return write(str); }

    // The unit-test mock leaves these out; firmware sources such as
    // Identity::printTo() call them, so a host build needs them.
    size_t println() { return write('\n'); }
    size_t println(const char* str) { size_t n = print(str); return n + write('\n'); }
    size_t println(char c) { size_t n = print(c); return n + write('\n'); }
    template <typename T> size_t println(T v, int r = DEC) { size_t n = print(v, r); return n + write('\n'); }
    
    // Arduino's Print::printf (ESP/nRF cores provide it); firmware uses it.
    virtual int printf(const char* fmt, ...) {
        va_list ap; va_start(ap, fmt);
        char buf[512];
        int n = vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        if (n > 0) write((const uint8_t*)buf, (size_t)(n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1));
        return n;
    }

    virtual void flush() { /* Empty implementation for backward compatibility */ }    
};

class Stream: public Print
{
public:
    virtual ~Stream() = default;
    virtual int available() { return 0; }
    virtual int availableForWrite() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return 0; }

    virtual size_t readBytes(char *buffer, size_t length) { 
        size_t i = 0;
        while (i < length && available()) {
            buffer[i++] = read();
        }
        return i;
    }
    virtual size_t readBytes(uint8_t *buffer, size_t length)
    {
        return readBytes((char *) buffer, length);
    }
};