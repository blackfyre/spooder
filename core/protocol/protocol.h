#ifndef PROTOCOL_H
#define PROTOCOL_H

// The serial link between the gamepad bridge and the robot: frame layout,
// CRC, encoders and an incremental decoder, shared by both firmwares.
// Hardware-free: compiled for both boards and for host tests.
//
// Frame:
//   0      start marker 0xA5
//   1      protocol version
//   2      type (1 = command, 2 = telemetry)
//   3      sequence number (wraps)
//   4      payload length N (fixed per type and version)
//   5      payload, N bytes, little-endian
//   5+N    CRC-16/CCITT-FALSE over bytes 1..4+N, little-endian
#include <stdint.h>

namespace protocol
{

constexpr uint8_t START = 0xA5;
constexpr uint8_t VERSION = 1;

enum class FrameType : uint8_t
{
  Command = 1,
  Telemetry = 2,
};

constexpr int HEADER_SIZE = 5;
constexpr int CRC_SIZE = 2;
constexpr int COMMAND_PAYLOAD_SIZE = 12;
constexpr int TELEMETRY_PAYLOAD_SIZE = 32;
constexpr int COMMAND_FRAME_SIZE = HEADER_SIZE + COMMAND_PAYLOAD_SIZE + CRC_SIZE;
constexpr int TELEMETRY_FRAME_SIZE = HEADER_SIZE + TELEMETRY_PAYLOAD_SIZE + CRC_SIZE;
constexpr int MAX_FRAME_SIZE = TELEMETRY_FRAME_SIZE;

// Command flags.
constexpr uint8_t FLAG_CONTROLLER_CONNECTED = 1u << 0;
constexpr uint8_t FLAG_EMERGENCY_STOP = 1u << 1;

// Bridge to robot, every 20 ms: the complete desired state, never events.
struct Command
{
  uint8_t mode;               // RequestedMode value
  int8_t translateX;          // -100..100
  int8_t translateY;          // -100..100, + = forward
  int8_t rotate;              // -100..100
  uint8_t height;             // 0..100
  uint8_t speed;              // 0..100
  uint8_t gait;               // gait::Gait index
  uint8_t stride;             // StrideMode value
  uint8_t flags;              // FLAG_*
  uint8_t clearCounter;       // incremented once per fault-clear request
  uint8_t modeRequestCounter; // incremented on every explicit mode request
};

// Sensor availability bits.
constexpr uint8_t SENSOR_TOF = 1u << 0;
constexpr uint8_t SENSOR_ATTITUDE = 1u << 1;

// Robot to bridge, every 100 ms.
struct Telemetry
{
  uint8_t bodyMode;  // body::Mode value
  uint8_t gait;
  bool outputsOn;
  uint16_t faults;   // protection::Fault bits
  uint16_t voltageMv;
  uint16_t currentMa;
  uint16_t validFrames;     // command frames accepted (wraps)
  uint16_t discardedFrames; // frames rejected (wraps)
  uint8_t sensorsAvailable; // SENSOR_* bits; fields below are meaningful only when set
  uint16_t tofMm;
  uint8_t tofStatus;
  int16_t pitchCentideg;
  int16_t rollCentideg;
};

/**
 * @brief CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection).
 */
uint16_t crc16(const uint8_t *data, int length);

/**
 * @brief Writes a complete frame into @p out (at least the frame's size).
 * @return the number of bytes written
 */
int encodeCommand(const Command &c, uint8_t seq, uint8_t *out);
int encodeTelemetry(const Telemetry &t, uint8_t seq, uint8_t *out);

struct Frame
{
  FrameType type;
  uint8_t seq;
  Command command;     // valid when type == Command
  Telemetry telemetry; // valid when type == Telemetry
};

// Incremental decoder. Feed every received byte with push(), then call next()
// until it returns false. A rejected candidate (bad version, type, length or
// CRC) is counted, and decoding resumes at the next start marker after its
// first byte, so a valid frame following garbage is never lost. An incomplete
// candidate is kept until its CRC can be checked, unless the line goes quiet:
// call idle() when no byte has arrived for longer than a frame takes to send,
// so a false start cannot hide a complete frame behind it.
class Decoder
{
public:
  void push(uint8_t byte);
  bool next(Frame &out);
  void idle();

  uint32_t validFrames() const { return valid_; }
  uint32_t discardedFrames() const { return discarded_; }

private:
  enum class Candidate
  {
    Invalid,
    Incomplete,
    Valid,
  };
  Candidate check(int at, int *total) const;
  void emit(int total, Frame &out);
  void drop(int count);
  void reject();

  uint8_t buf_[MAX_FRAME_SIZE] = {};
  int len_ = 0;
  uint32_t valid_ = 0;
  uint32_t discarded_ = 0;
};

} // namespace protocol

#endif // PROTOCOL_H
