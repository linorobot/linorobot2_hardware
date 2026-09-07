// Copyright (c) 2026 Linorobot contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FAKE_LD19_H
#define FAKE_LD19_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <math.h>
#include <stdint.h>

#ifdef USE_LIDAR_UDP
#include <WiFiUdp.h>
#endif

#ifndef FAKE_MAP_WIDTH
#define FAKE_MAP_WIDTH 10.0f    // Simulated room width (meters, -5.0 to +5.0)
#endif

#ifndef FAKE_MAP_HEIGHT
#define FAKE_MAP_HEIGHT 6.0f    // Simulated room height (meters, -3.0 to +3.0)
#endif

// TX ring buffer for the emitted stream, in bytes (ESP32 only -- RP2040 and
// RP2350 have no TX ring). One revolution is PACKS_PER_REV * 47 = 1786 bytes,
// so 2048 absorbs a whole scan and write() never waits on the wire.
#ifndef FAKE_LD19_TX_BUFFER
#define FAKE_LD19_TX_BUFFER 2048
#endif

// An LD19 sees 12 m. Beyond that it reports no measurement, not a reading at
// its limit. The default room is kept inside that reach: 10 x 6 m has an
// 11.66 m diagonal, so every wall is visible from every corner. A room larger
// than the sensor leaves whole sectors returning nothing, scan matching goes
// under-constrained, and odometry drift stops being corrected -- measured on a
// 16 x 12 room, the map smeared to a 41 x 37 m span with the walls nowhere
// near where they belong. Enlarging it is allowed, but that is the trade.
// Where the LiDAR sits on the robot, forward of base_footprint. The shipped
// linorobot2 URDF puts it at x = 0.12 m, and the emulator has to raycast from
// there rather than from the robot's centre: ROS transforms every scan by
// base_footprint -> laser before using it, so measuring from the centre puts
// each wall 12 cm too far out along whatever heading the robot has. Standing
// still that is a uniform 12 cm error; turning, it sweeps a 24 cm circle and
// the walls smear. Measured effect on a finished map: 0.196 m median distance
// from the true walls, down to a few cm once the origin is right.
#ifndef FAKE_LIDAR_OFFSET_X
#define FAKE_LIDAR_OFFSET_X 0.12f
#endif

#ifndef FAKE_LD19_MAX_RANGE_M
#define FAKE_LD19_MAX_RANGE_M 12.0f
#endif

#ifndef FAKE_SCAN_HZ
#define FAKE_SCAN_HZ 10         // Simulated LiDAR revolutions per second
#endif

#ifndef FAKE_SONAR_CONE_DEG
#define FAKE_SONAR_CONE_DEG 30.0f   // Ultrasonic beam width, full angle
#endif

#ifndef FAKE_ROBOT_RADIUS
#define FAKE_ROBOT_RADIUS 0.20f // Keeps the robot off the wall rather than in it
#endif

#ifndef FAKE_WALL_OBSTACLE
#define FAKE_WALL_OBSTACLE 1    // 1 to include obstacle wall
#endif

#ifndef FAKE_WALL_X1
#define FAKE_WALL_X1 2.0f       // Obstacle wall start X (m)
#endif
#ifndef FAKE_WALL_Y1
#define FAKE_WALL_Y1 -1.5f      // Obstacle wall start Y (m)
#endif
#ifndef FAKE_WALL_X2
#define FAKE_WALL_X2 2.0f       // Obstacle wall end X (m)
#endif
#ifndef FAKE_WALL_Y2
#define FAKE_WALL_Y2 1.5f       // Obstacle wall end Y (m)
#endif

#ifndef LIDAR_BAUDRATE
#define LIDAR_BAUDRATE 230400
#endif

#ifndef LIDAR_SERIAL
#define LIDAR_SERIAL 1
#endif

class FakeLD19
{
private:
    struct Segment {
        float x1, y1, x2, y2;
    };

    static const uint16_t POINTS_PER_PACK = 12;
    static const uint16_t POINTS_PER_REV  = 456;
    static const uint16_t PACKS_PER_REV   = POINTS_PER_REV / POINTS_PER_PACK; // 38
    static constexpr float DEG_PER_POINT  = 360.0f / 456.0f;                  // ~0.78947 deg
    // Scan rate is a bandwidth decision. Each revolution is 38 packets of 47
    // bytes, so 10 Hz needs ~17.9 kB/s of the ~23 kB/s that 230400 baud
    // carries -- 78% of the wire, which the TX ring buffer below absorbs
    // without ever stalling the control loop (measured on ESP32: odometry
    // holds 50.0 Hz at a 10 Hz scan). Going much above this has no headroom
    // left to give.
    static const uint16_t SPEED_DPS       = (uint16_t)(FAKE_SCAN_HZ * 360);
    static const uint32_t PACK_PERIOD_US  = (uint32_t)(1000000UL / (PACKS_PER_REV * FAKE_SCAN_HZ));
    // How many packets one step() may hand the UART. On ESP32 the TX ring
    // buffer absorbs them and write() returns at once, so a burst is free. On
    // RP2040/RP2350 there is no TX ring at all -- SerialUART::write() calls
    // uart_putc_raw() and spins once the 32-byte hardware FIFO fills, and its
    // availableForWrite() reports a bare 0/1 rather than a byte count, so
    // there is nothing to test before writing. One packet per step is the
    // bound there: the loop runs far faster than the ~380 packets/s a 10 Hz
    // scan needs, so pacing costs no scan rate and caps the stall at the 15
    // bytes that do not fit the FIFO (~0.65 ms).
#if defined(ESP32)
    static const uint8_t  MAX_PACKS_PER_STEP = 8;
#else
    static const uint8_t  MAX_PACKS_PER_STEP = 1;
#endif
    // Lag beyond which the schedule is resynced instead of burst through. Tied
    // to the scan, not to the per-step budget -- a platform that emits one
    // packet per step is not thereby behind.
    static const uint32_t RESYNC_LAG_US = PACK_PERIOD_US * 8;

    // Current robot pose in global world frame
    float pose_x_ = 0.0f;
    float pose_y_ = 0.0f;
    float pose_theta_ = 0.0f;

    uint16_t point_idx_ = 0;
    uint32_t next_pack_us_ = 0;
    Stream *out_stream_ = nullptr;
    int tx_pin_ = -1;
    bool enabled_ = false;

#ifdef USE_LIDAR_UDP
    WiFiUDP udp_;
    bool udp_enabled_ = false;
    uint8_t udp_buf_[256];
    uint16_t udp_buf_len_ = 0;
#endif

    // CalCRC8 lookup table (poly 0x4D), LDROBOT LD19 standard
    static uint8_t crc8(const uint8_t *data, int len)
    {
        static const uint8_t table[256] = {
          0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
          0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
          0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
          0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
          0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
          0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
          0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
          0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
          0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
          0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
          0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
          0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
          0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
          0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
          0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
          0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
        };
        uint8_t c = 0;
        while (len--) c = table[(c ^ *data++) & 0xff];
        return c;
    }

    static inline void put16(uint8_t *p, uint16_t v) {
        p[0] = (uint8_t)(v & 0xff);
        p[1] = (uint8_t)(v >> 8);
    }

public:
    FakeLD19() {}

    void setStream(Stream *stream)
    {
        out_stream_ = stream;
        enabled_ = true;
    }

    void begin(int tx_pin = -1, uint32_t baud = LIDAR_BAUDRATE)
    {
        tx_pin_ = tx_pin;
        if (tx_pin >= 0)
        {
#if defined(ESP32)
            HardwareSerial *serial = new HardwareSerial(LIDAR_SERIAL);
            // Give the UART a TX ring buffer big enough for a whole
            // revolution. Without one, write() has only the 128-byte hardware
            // FIFO behind it, and a catch-up burst of MAX_PACKS_PER_STEP
            // packets (376 bytes) blocks the control loop until the wire
            // drains it -- 16 ms at 230400, most of a 20 ms control period.
            // The buffer is drained by the UART interrupt, so write() returns
            // immediately and the emission costs the loop nothing but the
            // memcpy. Must be called before begin() to take effect.
            serial->setTxBufferSize(FAKE_LD19_TX_BUFFER);
            // On ESP32, configure UART with TX pin on LIDAR_RXD
            serial->begin(baud, SERIAL_8N1, -1, tx_pin);
            out_stream_ = serial;
            enabled_ = true;
#elif defined(ARDUINO_ARCH_RP2040)
#if defined(LIDAR_SERIAL) && LIDAR_SERIAL == 2
            Serial2.setTX(tx_pin);
            Serial2.begin(baud);
            out_stream_ = &Serial2;
#else
            Serial1.setTX(tx_pin);
            Serial1.begin(baud);
            out_stream_ = &Serial1;
#endif
            enabled_ = true;
#endif
        }

#ifdef USE_LIDAR_UDP
        udp_enabled_ = true;
        enabled_ = true;
#endif
        next_pack_us_ = micros() + PACK_PERIOD_US;
    }

    // Update the simulator with the robot odometry pose (meters and radians)
    // Hold a pose inside the room. The simulated drivetrain has no notion of
    // collision, so a robot driven at a wall sails straight through it and out
    // of the map -- the scan jumps to max range and SLAM has nothing to build
    // from. Stopping at the wall is also what a real robot does: the wheels
    // keep turning, the robot does not move, and odometry stops advancing.
    // Returns true when the pose had to be moved.
    bool clampToRoom(float &x, float &y) const
    {
        const float lim_x = (float)FAKE_MAP_WIDTH * 0.5f - (float)FAKE_ROBOT_RADIUS;
        const float lim_y = (float)FAKE_MAP_HEIGHT * 0.5f - (float)FAKE_ROBOT_RADIUS;
        const float in_x = x, in_y = y;
        if (x > lim_x) x = lim_x;
        if (x < -lim_x) x = -lim_x;
        if (y > lim_y) y = lim_y;
        if (y < -lim_y) y = -lim_y;

#if FAKE_WALL_OBSTACLE
        // The obstacle wall is solid too. Raycasting it but not colliding with
        // it lets the robot drive straight through the one thing in the room,
        // and the scan then shows that wall *behind* it -- which quietly makes
        // any obstacle-avoidance test meaningless, because nothing stops a plan
        // that goes through it.
        pushOffSegment(x, y,
                       (float)FAKE_WALL_X1, (float)FAKE_WALL_Y1,
                       (float)FAKE_WALL_X2, (float)FAKE_WALL_Y2);
#endif
        return (x != in_x) || (y != in_y);
    }

    // Push a point out to FAKE_ROBOT_RADIUS from a segment, along the shortest
    // way out, if it is inside. Approach direction does not matter: the robot
    // leaves by the side it came in on.
    static void pushOffSegment(float &x, float &y,
                               float x1, float y1, float x2, float y2)
    {
        const float r = (float)FAKE_ROBOT_RADIUS;
        const float sx = x2 - x1, sy = y2 - y1;
        const float len2 = sx * sx + sy * sy;

        // closest point on the segment, with the parameter clamped to its ends
        float t = (len2 > 1e-9f) ? ((x - x1) * sx + (y - y1) * sy) / len2 : 0.0f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        const float cx = x1 + t * sx, cy = y1 + t * sy;

        float nx = x - cx, ny = y - cy;
        float d = sqrtf(nx * nx + ny * ny);
        if (d >= r) return;             // already clear

        if (d < 1e-6f)
        {
            // dead on the segment: no direction to push along, so use its
            // normal and pick a side rather than dividing by zero
            if (len2 > 1e-9f) { nx = -sy; ny = sx; d = sqrtf(len2); }
            else              { nx = 1.0f; ny = 0.0f; d = 1.0f; }
        }
        x = cx + nx / d * r;
        y = cy + ny / d * r;
    }

    // What a forward-facing ultrasonic sensor would read: the nearest return
    // inside a cone, not a single ray. Gives the robot actual feedback that
    // something is ahead, rather than leaving it to be inferred from odometry
    // that has stopped advancing.
    float rangeAheadM(float cone_deg = (float)FAKE_SONAR_CONE_DEG)
    {
        const float half = cone_deg * 0.5f;
        uint16_t nearest = 0xFFFF;
        for (float d = -half; d <= half; d += 2.0f)
        {
            uint16_t r = raycastRangeMm(d < 0.0f ? d + 360.0f : d);
            if (r > 0 && r < nearest) nearest = r;
        }
        // -1 means "no reading", matching what the firmware's
        // rangeAheadOrNegative() expects. Returning 0.0 here would read as an
        // obstacle touching the robot and hold the safety stop on permanently
        // whenever the path ahead is simply open beyond the sensor's reach.
        return (nearest == 0xFFFF) ? -1.0f : (float)nearest / 1000.0f;
    }

    void updatePose(float x, float y, float theta)
    {
        pose_x_ = x;
        pose_y_ = y;
        pose_theta_ = theta;
    }

    // Raycast distance for a given beam angle (in degrees, 0..360)
    uint16_t raycastRangeMm(float beam_deg)
    {
        // Global ray angle in radians: robot yaw MINUS the beam angle, because
        // an LD19's angle field increases the way the head physically turns,
        // which is clockwise seen from above -- the opposite of the
        // right-handed convention ROS uses. ldlidar_stl_ros2 negates it again
        // to produce a proper counter-clockwise LaserScan, so emitting the
        // mathematically positive direction here leaves that flip unmatched and
        // publishes a mirror image of the room.
        //
        // Nothing looks wrong: the scan is the right shape and the right size,
        // and while the robot is still it is even self-consistent. Only when it
        // turns does the mirrored scan rotate against the odometry, so scan
        // matching fights the wheels and the map inflates. Measured against a
        // ground-truth raycast of the room: 1.305 m mean error as emitted,
        // 0.019 m once mirrored.
        float ray_rad = pose_theta_ - (beam_deg * 0.0174532925f);
        float dx = cosf(ray_rad);
        float dy = sinf(ray_rad);

        // the beam leaves the sensor, which is mounted ahead of the robot's centre
        const float ox = pose_x_ + cosf(pose_theta_) * (float)FAKE_LIDAR_OFFSET_X;
        const float oy = pose_y_ + sinf(pose_theta_) * (float)FAKE_LIDAR_OFFSET_X;

        // Define room perimeter boundaries centered at (0,0)
        const float half_w = (float)FAKE_MAP_WIDTH * 0.5f;
        const float half_h = (float)FAKE_MAP_HEIGHT * 0.5f;

        Segment segs[5] = {
            {-half_w, -half_h,  half_w, -half_h}, // South wall
            { half_w, -half_h,  half_w,  half_h}, // East wall
            { half_w,  half_h, -half_w,  half_h}, // North wall
            {-half_w,  half_h, -half_w, -half_h}, // West wall
            {(float)FAKE_WALL_X1, (float)FAKE_WALL_Y1, (float)FAKE_WALL_X2, (float)FAKE_WALL_Y2} // Short obstacle wall
        };

        int count = FAKE_WALL_OBSTACLE ? 5 : 4;
        float min_dist = (float)FAKE_LD19_MAX_RANGE_M; // nothing seen yet

        for (int i = 0; i < count; i++)
        {
            float sx = segs[i].x2 - segs[i].x1;
            float sy = segs[i].y2 - segs[i].y1;
            float denom = dx * sy - dy * sx;
            if (fabsf(denom) < 1e-6f) continue;

            float px = segs[i].x1 - ox;
            float py = segs[i].y1 - oy;

            float t = (px * sy - py * sx) / denom;
            float u = (px * dy - py * dx) / denom;

            if (t > 0.03f && u >= 0.0f && u <= 1.0f)
            {
                if (t < min_dist) min_dist = t;
            }
        }

        // No segment inside the sensor's reach means no measurement. Reporting
        // the range limit instead would put a return there, and a real LD19
        // does not do that -- it reports 0. It matters more than it sounds:
        // beams that overshoot the far wall would otherwise draw a phantom arc
        // at exactly 12 m, SLAM would map it as a wall standing inside the
        // room, and the map comes out larger and sheared. 0 is the protocol's
        // "no measurement", which the driver turns into an invalid range.
        if (min_dist >= (float)FAKE_LD19_MAX_RANGE_M) return 0;

        // Add small simulated white noise (+-5 mm)
        float noise = ((float)(rand() % 11) - 5.0f);
        float dist_mm = (min_dist * 1000.0f) + noise;

        if (dist_mm < 50.0f) dist_mm = 50.0f;
        return (uint16_t)dist_mm;
    }

    // Advance packet generation and stream out if it is time
    // Emit however many packets the clock says are due, not one per call.
    //
    // A 10 Hz LD19 is 380 packets/s, but loop() does not iterate anywhere near
    // that: the micro-ROS executor dominates each cycle, so one packet per call
    // yielded roughly 1.5 rev/s -- enough to look like it works, far too slow
    // for SLAM to build a map from. The budget bounds the catch-up so a long
    // stall cannot monopolise the loop, and the schedule advances by whole
    // periods so it does not drift.
    void step()
    {
        if (!enabled_) return;

        for (uint8_t budget = 0; budget < MAX_PACKS_PER_STEP; budget++)
        {
            uint32_t now = micros();
            if ((int32_t)(now - next_pack_us_) < 0) return;
            // if we fell far behind, resync rather than burst through a backlog
            if ((int32_t)(now - next_pack_us_) > (int32_t)RESYNC_LAG_US)
                next_pack_us_ = now;
            next_pack_us_ += PACK_PERIOD_US;
            emitPack();
        }
    }

private:
    void emitPack()
    {
        uint8_t pkt[47];
        pkt[0] = 0x54;
        pkt[1] = 0x2C;
        put16(&pkt[2], SPEED_DPS);

        float start_deg = point_idx_ * DEG_PER_POINT;
        float end_deg   = (point_idx_ + POINTS_PER_PACK - 1) * DEG_PER_POINT;
        put16(&pkt[4], (uint16_t)(fmodf(start_deg, 360.0f) * 100.0f));

        for (int i = 0; i < POINTS_PER_PACK; i++)
        {
            float deg = (point_idx_ + i) * DEG_PER_POINT;
            uint16_t dist = raycastRangeMm(deg);
            // 0 distance is "no measurement"; a real unit reports no
            // confidence with it, and drivers use that to reject the point.
            uint8_t inten = (dist == 0) ? 0
                          : (dist < 1500) ? 220 : (dist < 4000) ? 180 : 120;
            put16(&pkt[6 + i * 3], dist);
            pkt[6 + i * 3 + 2] = inten;
        }

        put16(&pkt[42], (uint16_t)(fmodf(end_deg, 360.0f) * 100.0f));
        put16(&pkt[44], (uint16_t)(millis() % 30000));
        pkt[46] = crc8(pkt, 46);

        if (out_stream_)
        {
            out_stream_->write(pkt, 47);
        }

#ifdef USE_LIDAR_UDP
        if (udp_enabled_)
        {
            memcpy(udp_buf_ + udp_buf_len_, pkt, 47);
            udp_buf_len_ += 47;
            if (udp_buf_len_ >= 141)
            {
                udp_.beginPacket(LIDAR_SERVER, LIDAR_PORT);
                udp_.write(udp_buf_, udp_buf_len_);
                udp_.endPacket();
                udp_buf_len_ = 0;
            }
        }
#endif

        point_idx_ += POINTS_PER_PACK;
        if (point_idx_ >= POINTS_PER_REV) point_idx_ -= POINTS_PER_REV;
    }
};

#endif
