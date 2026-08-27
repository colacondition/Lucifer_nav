#pragma once
// （补上的 include guard：此前整文件无 guard，二次包含直接编译错误。）

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace serial_driver
{

enum PkgState : uint8_t
{
  COMPLETE = 0,
  HEADER_INCOMPLETE,
  PAYLOAD_INCOMPLETE,
  CRC_HEADER_ERRROR,
  CRC_PKG_ERROR,
  OTHER
};

enum StopBit : uint8_t { ONE = 0, ONE_POINT_FIVE, TWO };

enum Parity : uint8_t { NONE = 0, ODD, EVEN, MARK, SPACE };

class SerialConfig
{
public:
  SerialConfig() = delete;
  SerialConfig(
    int bps, int databit, bool flow, StopBit stopbits,
    Parity paritys, std::string name)
  : baudrate(bps), databits(databit), flowcontrol(flow), stopbit(stopbits),
    parity(paritys), devname(name) {}
  ~SerialConfig();


  int baudrate = 115200;
  int databits = 8;
  bool flowcontrol = 0;
  StopBit stopbit = StopBit::ONE;
  Parity parity = Parity::NONE;
  std::string devname = "/dev/ttyACM0";
  // 指定设备打不开时是否允许自动扫描 /dev/ttyACM{0..2} 兜底。默认保持
  // 历史行为（true）；实车多插一个 CDC 设备时务必关掉——静默绑错串口
  // 就是把指令发给空气甚至别的设备。
  bool allow_fallback = true;
};

class Port
{
public:
  Port(std::shared_ptr<SerialConfig> ptr);
  ~Port();

  int openPort();
  bool closePort();
  bool init();
  bool reopen();
  bool isPortInit();
  bool isPortOpen();

  int transmit(uint8_t * buff, int writeSize);
  int receive(uint8_t * buffer);
  // 最近一次 transmit/write 失败的 errno（成功时清零）。供调用方区分
  // 「非阻塞满缓冲（EAGAIN，可下一帧重试）」和「真故障（需要 reopen）」。
  int lastError() const {return last_errno_;}
  int fd = -1;

private:
  std::shared_ptr<SerialConfig> config;
  std::vector<std::string> device_names = {"/dev/ttyACM0", "/dev/ttyACM1",
    "/dev/ttyACM2"};
  int flags = 0;
  int last_errno_ = 0;
  bool isinit = false;
  bool isopen = false;
};
} // namespace serial_driver
