#include "serial_port.hpp"

#include <cerrno>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <iostream>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define termios asmtermios
#include <asm/termios.h>
#undef termios
#include <termios.h>

namespace serial_driver
{
  extern "C" int ioctl(int d, int request, ...);

  Port::Port(std::shared_ptr<SerialConfig> ptr) { config = ptr; }

  bool Port::init()
  {
    struct termios newtio;
    struct termios oldtio;
    bzero(&newtio, sizeof(newtio));
    bzero(&oldtio, sizeof(oldtio));

    if (tcgetattr(fd, &oldtio) != 0)
    {
      perror("tcgetattr");
      return false;
    }
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    /* set data bits */
    switch (config->databits)
    {
    case 5:
      newtio.c_cflag |= CS5;
      break;
    case 6:
      newtio.c_cflag |= CS6;
      break;
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
    default:
      fprintf(stderr, "unsupported data size\n");
      return false;
    }
    /* set parity */
    switch (config->parity)
    {
    case Parity::NONE:
      newtio.c_cflag &= ~PARENB; /* Clear parity enable */
      newtio.c_iflag &= ~INPCK;  /* Disable input parity check */
      break;
    case Parity::ODD:
      newtio.c_cflag |= (PARODD | PARENB); /* Odd parity instead of even */
      newtio.c_iflag |= INPCK;             /* Enable input parity check */
      break;
    case Parity::EVEN:
      newtio.c_cflag |= PARENB;  /* Enable parity */
      newtio.c_cflag &= ~PARODD; /* Even parity instead of odd */
      newtio.c_iflag |= INPCK;   /* Enable input parity check */
      break;
    case Parity::MARK:
      newtio.c_cflag |= PARENB; /* Enable parity */
      newtio.c_cflag |= CMSPAR; /* Stick parity instead */
      newtio.c_cflag |= PARODD; /* Even parity instead of odd */
      newtio.c_iflag |= INPCK;  /* Enable input parity check */
      break;
    case Parity::SPACE:
      newtio.c_cflag |= PARENB;  /* Enable parity */
      newtio.c_cflag |= CMSPAR;  /* Stick parity instead */
      newtio.c_cflag &= ~PARODD; /* Even parity instead of odd */
      newtio.c_iflag |= INPCK;   /* Enable input parity check */
      break;
    default:
      fprintf(stderr, "unsupported parity\n");
      return false;
    }

    /* set stop bits */
    switch (config->stopbit)
    {
    case StopBit::ONE:
      newtio.c_cflag &= ~CSTOPB;
      break;
    case StopBit::TWO:
      newtio.c_cflag |= CSTOPB;
      break;
    default:
      perror("unsupported stop bits");
      return false;
    }

    if (config->flowcontrol)
      newtio.c_cflag |= CRTSCTS;
    else
      newtio.c_cflag &= ~CRTSCTS;

    newtio.c_cc[VTIME] = 10; /* Time-out value (tenths of a second) [!ICANON]. */
    newtio.c_cc[VMIN] = 0;   /* Minimum number of bytes read at once [!ICANON]. */
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) != 0)
    {
      perror("tcsetattr");
      return false;
    }

    struct termios2 tio;

    if (ioctl(fd, TCGETS2, &tio))
    {
      perror("TCGETS2");
      return false;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = config->baudrate;
    tio.c_ospeed = config->baudrate;

    if (ioctl(fd, TCSETS2, &tio))
    {
      perror("TCSETS2");
      return false;
    }

    if (ioctl(fd, TCGETS2, &tio))
    {
      perror("TCGETS2");
      return false;
    }
    isinit = true;
    return true;
  }

  bool Port::setPermission(const std::string & name)
  {
    std::string cmd = "sudo chmod 777 " + name;
    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
    {
      std::cerr << "Failed to run permission command for " << name << std::endl;
      return false;
    }
    const int rc = pclose(pipe);
    if (rc != 0)
    {
      std::cerr << "Permission command failed for " << name
                << " with code " << rc << std::endl;
      return false;
    }
    return true;
  }

  int Port::openPort()
  {
    auto try_open = [this](const std::string & device_name) {
        setPermission(device_name);
        fd = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0)
        {
          std::cerr << "open device failed: " << device_name
                    << " error=" << strerror(errno) << std::endl;
          isopen = false;
          return false;
        }

        config->devname = device_name;
        return true;
      };

    if (!try_open(config->devname))
    {
      for (auto device_name : device_names)
      {
        if (device_name == config->devname) {
          continue;
        }
        if (try_open(device_name)) {
          break;
        }
      }
    }

    if (fd < 0)
    {
      isopen = false;
      return fd;
    }

    flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0)
    {
      std::cerr << "fcntl(F_GETFL) failed for " << config->devname
                << " error=" << strerror(errno) << std::endl;
      closePort();
      return -1;
    }
    // Keep non-blocking reads so uplink gimbal packets can be polled.
    flags |= O_NONBLOCK;

    if (fcntl(fd, F_SETFL, flags) < 0)
    {
      std::cerr << "fcntl(F_SETFL) failed for " << config->devname
                << " error=" << strerror(errno) << std::endl;
      closePort();
      return -1;
    }

    if (isatty(fd) == 0)
    {
      std::cerr << config->devname << " is not a tty device" << std::endl;
      closePort();
      return -1;
    }

    if (!init())
    {
      std::cerr << "Serial init failed for " << config->devname << std::endl;
      closePort();
      return -1;
    }

    isopen = true;
    std::cout << "Serial port opened: " << config->devname
              << " fd=" << fd << std::endl;
    return fd;
  }

  int Port::transmit(uint8_t *buff, int writeSize)
  {
    int num = write(fd, buff, writeSize);
    if (num < 0)
    {
      std::cerr << "Serial write failed on " << config->devname
                << " error=" << strerror(errno) << std::endl;
    }
    return num;
  }

  int Port::receive(uint8_t *buffer)
  {
    // do not change the 64 -> size of the usb driver.
    int num = read(fd, buffer, 64);
    if (num < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return 0;
    }
    return num;
  }

  bool Port::closePort()
  {
    if (fd < 0)
    {
      isopen = false;
      isinit = false;
      return true;
    }

    isopen = false;
    isinit = false;
    const int close_rc = close(fd);
    fd = -1;
    return close_rc == 0;
  }

  bool Port::reopen()
  {
    if (isPortOpen())
      closePort();

    if (openPort() >= 0 && isPortOpen())
      return true;

    return false;
  }

  bool Port::isPortInit() { return isinit; }

  bool Port::isPortOpen() { return isopen; }

  Port::~Port() {}
  SerialConfig::~SerialConfig() {}

} // namespace serial_driver
