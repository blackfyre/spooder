#ifndef VIRTUAL_ROBOT_PTY_H
#define VIRTUAL_ROBOT_PTY_H

// A pseudo-terminal standing for one of the robot's serial links. The
// virtual robot holds the master; a front-end opens the slave (or a stable
// symbolic link to it) as if it were the robot's port.
#include <stdint.h>
#include <string>
#include <vector>

class Pty
{
public:
  // Bytes held for a slow reader, at most; more output is dropped, frames
  // whole (the board's equivalent is its 256-byte transmit buffer).
  static constexpr size_t MAX_BACKLOG = 4096;

  ~Pty() { close(); }

  /**
   * @brief Opens a new terminal in raw mode and, if @p linkPath is set, points
   * that symbolic link at it (replacing what was there).
   * @return false with @p error set on failure
   */
  bool open(const std::string &linkPath, std::string &error);

  /**
   * @brief Closes the terminal (its reader sees the port vanish) and removes
   * the link.
   */
  void close();

  bool isOpen() const { return fd_ >= 0; }
  const std::string &path() const { return slave_; }

  /**
   * @brief True while a program has the terminal open: the equivalent of a
   * USB host being connected.
   */
  bool connected();

  /**
   * @brief Reads at most @p max pending bytes without blocking.
   */
  int read(uint8_t *out, int max);

  /**
   * @brief Writes a frame whole, or drops it whole if no one is connected or
   * the backlog is full.
   */
  bool writeFrame(const uint8_t *frame, int length);

  /**
   * @brief Writes text, dropped as frames are.
   */
  void writeText(const char *text);

  // Output held for a slow reader: wait for POLLOUT on fd() and flush(), so
  // it goes out even when nothing new is written (stepped mode).
  bool backlogged() const { return fd_ >= 0 && !backlog_.empty(); }
  int fd() const { return fd_; }
  void flush();

private:
  bool queue(const uint8_t *data, size_t length);

  int fd_ = -1;
  std::string slave_;
  std::string link_;
  std::vector<uint8_t> backlog_;
};

#endif // VIRTUAL_ROBOT_PTY_H
