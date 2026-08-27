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

#include <termios.h>

namespace serial_driver
{
Port::Port(std::shared_ptr<SerialConfig> ptr) {config = ptr;}

bool Port::init()
{
  struct termios newtio;
  bzero(&newtio, sizeof(newtio));

  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  /* set data bits */
  switch (config->databits) {
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
  switch (config->parity) {
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
  switch (config->stopbit) {
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

  if (config->flowcontrol) {
    newtio.c_cflag |= CRTSCTS;
  } else {
    newtio.c_cflag &= ~CRTSCTS;
  }

  // 只用标准 POSIX 波特率：不再走 asm/termios + ioctl(TCGETS2/TCSETS2) 的
  // 非标波特率 hack（旧实现只为支持 961200，而实际配置是 115200，hack 从未
  // 生效过，还让代码依赖内核头文件）。端口是非阻塞读（O_NONBLOCK），
  // VMIN/VTIME 不生效，无需设置。
  speed_t speed;
  switch (config->baudrate) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 500000: speed = B500000; break;
    case 921600: speed = B921600; break;
    default:
      fprintf(
        stderr,
        "unsupported baud rate %d: use a standard POSIX rate "
        "(9600/19200/38400/57600/115200/230400/460800/500000/921600)\n",
        config->baudrate);
      return false;
  }
  cfsetispeed(&newtio, speed);
  cfsetospeed(&newtio, speed);

  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
    perror("tcsetattr");
    return false;
  }

  isinit = true;
  return true;
}

int Port::openPort()
{
  const std::string requested_device = config->devname;
  bool bound_fallback = false;

  auto try_open = [this, &bound_fallback](const std::string & device_name) {
      fd = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
      if (fd < 0) {
        std::cerr << "open device failed: " << device_name
                  << " error=" << strerror(errno) << std::endl;
        if (errno == EACCES || errno == EPERM) {
          // 不再 sudo chmod：权限归 udev/dialout 组管。
          std::cerr << "Permission denied on " << device_name
                    << " — add the user to the 'dialout' group or install a udev rule "
                    << "(e.g. KERNEL==\"ttyACM*\", MODE=\"0666\")." << std::endl;
        }
        isopen = false;
        return false;
      }

      if (device_name != config->devname) {
        bound_fallback = true;
      }
      config->devname = device_name;
      return true;
    };

  if (!try_open(config->devname)) {
    // 静默回退是「平时不报错、战时坑人」的典型：多插一个 CDC 设备就会
    // 把指令发给错误的对象。现在每次回退都大声告警，并支持配置关闭。
    if (config->allow_fallback) {
      std::cerr << "[FALLBACK] requested device " << requested_device
                << " unavailable; scanning ttyACM candidates. Set allow_fallback=false "
                << "to disable (recommended when multiple CDC devices are attached)."
                << std::endl;
      for (auto device_name : device_names) {
        if (device_name == config->devname) {
          continue;
        }
        if (try_open(device_name)) {
          break;
        }
      }
    }
  }

  if (bound_fallback) {
    std::cerr << "[FALLBACK] BOUND TO DIFFERENT DEVICE: requested=" << requested_device
              << " actual=" << config->devname
              << " — verify this is the MCU, not another USB-serial dongle!"
              << std::endl;
  }

  if (fd < 0) {
    isopen = false;
    return fd;
  }

  flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    std::cerr << "fcntl(F_GETFL) failed for " << config->devname
              << " error=" << strerror(errno) << std::endl;
    closePort();
    return -1;
  }
  // Keep non-blocking reads so uplink gimbal packets can be polled.
  flags |= O_NONBLOCK;

  if (fcntl(fd, F_SETFL, flags) < 0) {
    std::cerr << "fcntl(F_SETFL) failed for " << config->devname
              << " error=" << strerror(errno) << std::endl;
    closePort();
    return -1;
  }

  if (isatty(fd) == 0) {
    std::cerr << config->devname << " is not a tty device" << std::endl;
    closePort();
    return -1;
  }

  if (!init()) {
    std::cerr << "Serial init failed for " << config->devname << std::endl;
    closePort();
    return -1;
  }

  isopen = true;
  std::cout << "Serial port opened: " << config->devname
            << " fd=" << fd << std::endl;
  return fd;
}

int Port::transmit(uint8_t * buff, int writeSize)
{
  last_errno_ = 0;
  errno = 0;
  int num = write(fd, buff, writeSize);
  if (num < 0) {
    last_errno_ = errno;
    std::cerr << "Serial write failed on " << config->devname
              << " error=" << strerror(errno) << std::endl;
  }
  return num;
}

int Port::receive(uint8_t * buffer)
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
  if (fd < 0) {
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
  if (isPortOpen()) {
    closePort();
  }

  if (openPort() >= 0 && isPortOpen()) {
    return true;
  }

  return false;
}

bool Port::isPortInit() {return isinit;}

bool Port::isPortOpen() {return isopen;}

Port::~Port()
{
  // 节点/容器卸载时关掉还开着的 fd：旧实现析构为空，端口只靠 closePort()
  // 显式关闭，每次重启组件都会泄漏一个 fd。
  if (fd >= 0) {
    closePort();
  }
}
SerialConfig::~SerialConfig() {}

} // namespace serial_driver
