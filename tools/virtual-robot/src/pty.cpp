#include "pty.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

bool Pty::open(const std::string &linkPath, std::string &error)
{
  close();
  int fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0 || grantpt(fd) != 0 || unlockpt(fd) != 0)
  {
    error = std::string("cannot create a pseudo-terminal: ") + strerror(errno);
    if (fd >= 0)
      ::close(fd);
    return false;
  }
  const char *name = ptsname(fd);
  if (!name)
  {
    error = "ptsname failed";
    ::close(fd);
    return false;
  }
  slave_ = name;
  // Raw mode, set once on the slave: no echo, no line editing, no CR/LF
  // translation. Closing the slave again leaves the master reporting a
  // hang-up until a front-end opens it.
  int s = ::open(name, O_RDWR | O_NOCTTY);
  if (s < 0)
  {
    error = std::string("cannot open ") + name + ": " + strerror(errno);
    ::close(fd);
    return false;
  }
  struct termios t;
  tcgetattr(s, &t);
  cfmakeraw(&t);
  tcsetattr(s, TCSANOW, &t);
  ::close(s);
  fd_ = fd;
  backlog_.clear();

  if (!linkPath.empty())
  {
    // Replace the link atomically, so a front-end never sees it missing.
    std::string tmp = linkPath + ".tmp";
    unlink(tmp.c_str());
    if (symlink(name, tmp.c_str()) != 0 || rename(tmp.c_str(), linkPath.c_str()) != 0)
    {
      error = "cannot link " + linkPath + ": " + strerror(errno);
      close();
      return false;
    }
    link_ = linkPath;
  }
  return true;
}

void Pty::close()
{
  if (!link_.empty())
  {
    unlink(link_.c_str());
    link_.clear();
  }
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  backlog_.clear();
}

bool Pty::connected()
{
  if (fd_ < 0)
    return false;
  struct pollfd p = {fd_, POLLOUT, 0};
  if (poll(&p, 1, 0) < 0)
    return false;
  return (p.revents & POLLHUP) == 0;
}

int Pty::read(uint8_t *out, int max)
{
  if (fd_ < 0 || max <= 0)
    return 0;
  ssize_t n = ::read(fd_, out, (size_t)max);
  return n > 0 ? (int)n : 0; // EAGAIN: nothing pending; EIO: no one connected
}

void Pty::flush()
{
  if (fd_ < 0)
    return;
  if (!connected())
  {
    backlog_.clear(); // the reader has gone: as without a USB host
    return;
  }
  while (!backlog_.empty())
  {
    ssize_t n = ::write(fd_, backlog_.data(), backlog_.size());
    if (n <= 0)
      return; // the reader is behind; try again next time
    backlog_.erase(backlog_.begin(), backlog_.begin() + n);
  }
}

bool Pty::queue(const uint8_t *data, size_t length)
{
  if (!connected())
  {
    backlog_.clear(); // as without a USB host: output is dropped
    return false;
  }
  flush();
  if (backlog_.size() + length > MAX_BACKLOG)
    return false;
  // What the kernel does not take now is held and sent first next time, so
  // nothing is ever cut.
  backlog_.insert(backlog_.end(), data, data + length);
  flush();
  return true;
}

bool Pty::writeFrame(const uint8_t *frame, int length)
{
  return queue(frame, (size_t)length);
}

void Pty::writeText(const char *text)
{
  queue((const uint8_t *)text, strlen(text));
}
