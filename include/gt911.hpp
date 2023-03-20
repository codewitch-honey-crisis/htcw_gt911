/* by honey the codewitch

Portions derived from GT911 by alex-code. Original license follows
MIT License

Copyright (c) 2021 alex-code

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#pragma once
#include <Arduino.h>
#include <Wire.h>
namespace arduino {

template <int16_t PinRst = -1, int16_t PinInt = -1, uint8_t Address = 0x5D>
class gt911 {
   public:
    using type = gt911;
    constexpr static const int16_t pin_rst = PinRst;
    constexpr static const int16_t pin_int = PinInt;
    constexpr static const uint8_t address = Address;
    struct point {
        uint8_t id;
        uint16_t x;
        uint16_t y;
        uint16_t area;
    };

   private:
    static volatile bool in_irq;
#if defined(ESP8266)
    static void ICACHE_RAM_ATTR irq_handler() {
        noInterrupts();
        in_irq = true;
        interrupts();
    }
#elif defined(ESP32)
    static void IRAM_ATTR irq_handler() {
        noInterrupts();
        in_irq = true;
        interrupts();
    }
#else
    static void irq_handler() {
        noInterrupts();
        in_irq = true;
        interrupts();
    }
#endif

    TwoWire& m_i2c;
    uint16_t m_fw_id;
    uint16_t m_width;
    uint16_t m_height;
    uint8_t m_vendor_id;
    uint8_t m_rotation;
    size_t m_locations_size;
    point m_locations[5];

    void i2c_start(uint16_t reg) {
        m_i2c.beginTransmission(address);
        m_i2c.write(reg >> 8);
        m_i2c.write(reg & 0xFF);
    }

    bool write(uint16_t reg, uint8_t data) {
        i2c_start(reg);
        m_i2c.write(data);
        return m_i2c.endTransmission() == 0;
    }

    uint8_t read(uint16_t reg) {
        i2c_start(reg);
        m_i2c.endTransmission();
        m_i2c.requestFrom(address, (uint8_t)1);
        while (m_i2c.available()) {
            return m_i2c.read();
        }
        return 0;
    }

    bool writeBytes(uint16_t reg, uint8_t* data, uint16_t size) {
        i2c_start(reg);
        for (uint16_t i = 0; i < size; i++) {
            m_i2c.write(data[i]);
        }
        return m_i2c.endTransmission() == 0;
    }

    bool readBytes(uint16_t reg, uint8_t* data, uint16_t size) {
        i2c_start(reg);
        m_i2c.endTransmission();

        uint16_t index = 0;
        while (index < size) {
            uint8_t req = _min(size - index, I2C_BUFFER_LENGTH);
            m_i2c.requestFrom(address, req);
            while (m_i2c.available()) {
                data[index++] = m_i2c.read();
            }
            index++;
        }

        return size == index - 1;
    }

    uint8_t calcChecksum(uint8_t* buf, uint8_t len) {
        uint8_t ccsum = 0;
        for (uint8_t i = 0; i < len; i++) {
            ccsum += buf[i];
        }

        return (~ccsum) + 1;
    }

    uint8_t readChecksum() {
        return read(0x80FF);
    }

    int8_t readTouches() {
        uint32_t timeout = millis() + 20;
        do {
            uint8_t flag = read(0x814E);
            if ((flag & 0x80) && ((flag & 0x0F) < sizeof(m_locations))) {
                write(0x814E, 0);
                return flag & 0x0F;
            }
            delay(1);
        } while (millis() < timeout);

        return 0;
    }

    bool readTouchPoints() {
        uint8_t buffer[8 * sizeof(m_locations)];
        bool result = readBytes(0x814E + 1, buffer, sizeof(buffer));
        if (result) {
            for (uint8_t i = 0; i < (sizeof(m_locations)/sizeof(point)); ++i) {
                point& p = m_locations[i];
                const uint8_t* pb = buffer + (i * 8);
                p.id = *pb;
                memcpy(&p.area, pb + 5, 2);
                // TODO: test rotations
                switch (m_rotation & 2) {
                    case 1:
                        memcpy(&p.y, pb + 1, 2);
                        memcpy(&p.x, pb + 3, 2);
                        p.x = m_height - p.x;
                        break;
                        break;
                    case 2:
                        memcpy(&p.x, pb + 1, 2);
                        memcpy(&p.y, pb + 3, 2);
                        p.x = m_width - p.x;
                        p.y = m_height - p.y;
                        break;
                        break;
                    case 3:
                        memcpy(&p.y, pb + 1, 2);
                        memcpy(&p.x, pb + 3, 2);
                        p.y = m_width - p.y;
                        break;
                    default:  // 0
                        memcpy(&p.x, pb + 1, 2);
                        memcpy(&p.y, pb + 3, 2);
                        break;
                }
            }
        }

        return result;
    }

    gt911(const gt911& rhs) = delete;
    gt911& operator=(const gt911& rhs) = delete;
    void do_move(gt911& rhs) {
        m_i2c = rhs.m_i2c;
        m_fw_id = rhs.m_fw_id;
        m_width = rhs.m_width;
        m_height = rhs.m_height;
        m_vendor_id = rhs.m_vendor_id;
        m_rotation = rhs.m_rotation;
        memcpy(m_locations, rhs.m_locations, m_locations_size * sizeof(point));
    }

   public:
    gt911(TwoWire& i2c = Wire) : m_i2c(i2c), m_fw_id(0), m_width(0), m_height(0), m_vendor_id(0), m_rotation(0), m_locations_size(0) {
    }
    gt911(gt911&& rhs) {
        do_move(rhs);
    }
    gt911& operator=(gt911&& rhs) {
        do_move(rhs);
        return *this;
    }
    bool initialized() const { return m_width != 0; }
    bool initialize() {
        if (m_width == 0) {
            m_locations_size = 0;
            if (pin_rst > -1) {
                delay(300);
                reset();
                delay(200);
            }
            if (pin_int > -1) {
                pinMode(pin_int, INPUT);
                attachInterrupt(pin_int, irq_handler, FALLING);
            }
            m_i2c.begin();
            m_i2c.beginTransmission(address);
            if (m_i2c.endTransmission() == 0) {
                // Need to get resolution to use rotation
                uint8_t buffer[11];
                readBytes(0x8140, buffer, sizeof(buffer));
                memcpy(&m_fw_id, &buffer[4], 2);
                memcpy(&m_width, &buffer[6], 2);
                memcpy(&m_height, &buffer[8], 2);
                m_vendor_id = buffer[10];
            } else {
                return false;
            }
        }
        return m_width != 0;
    }
    uint8_t rotation() const { return m_rotation; }
    void rotation(uint8_t value) {
        m_rotation = value & 2;
    }
    size_t locations_size() const {
        return m_locations_size;
    }
    void locations(point* out_locations,size_t* in_out_locations_size) const {
        if(out_locations==nullptr || in_out_locations_size==nullptr) {
            return;
        }
        int count = *in_out_locations_size;
        if(m_locations_size<count) {
            *in_out_locations_size = m_locations_size;
        }
        memcpy(out_locations,m_locations,sizeof(point)**in_out_locations_size);
    }
    void reset() {
        if (!initialized()) {
            return;
        }
        delay(1);
        if (pin_int > -1) {
            pinMode(pin_int, OUTPUT);
            digitalWrite(pin_int, LOW);
        }
        if (pin_rst > -1) {
            pinMode(pin_rst, OUTPUT);
            digitalWrite(pin_rst, LOW);
        }

        delay(11);
        if (pin_int > -1) {
            digitalWrite(pin_int, address == 0x14);
        }

        delayMicroseconds(110);
        if (pin_rst > -1) {
            pinMode(pin_rst, INPUT);
        }

        delay(6);
        if (pin_int > -1) {
            digitalWrite(pin_int, LOW);
        }
        delay(51);
        m_width = 0;
    }
    void update() {
        if (!initialized()) {
            return;
        }
        bool irq = false;
        if (pin_int > -1) {
            noInterrupts();
            irq = in_irq;
            in_irq = false;
            interrupts();
        } else {
            irq = true;
        }
        uint8_t contacts = 0;
        if (irq) {
            contacts = readTouches();
            if (contacts > 0) {
                readTouchPoints();
            }
        }
        m_locations_size = contacts;
    }
};

template <int16_t PinRst, int16_t PinInt, uint8_t Address>
volatile bool gt911<PinRst, PinInt, Address>::in_irq = false;
}  // namespace arduino