#pragma once

#ifdef __linux__
#include <termios.h>
#else
// for the termios struct
typedef unsigned char cc_t;
typedef unsigned int speed_t;
typedef unsigned int tcflag_t;
#define NCCS 32
struct termios {
  tcflag_t c_iflag; /* input mode flags */
  tcflag_t c_oflag; /* output mode flags */
  tcflag_t c_cflag; /* control mode flags */
  tcflag_t c_lflag; /* local mode flags */
  cc_t c_line;      /* line discipline */
  cc_t c_cc[NCCS];  /* control characters */
  speed_t c_ispeed; /* input speed */
  speed_t c_ospeed; /* output speed */
#define _HAVE_STRUCT_TERMIOS_C_ISPEED 1
#define _HAVE_STRUCT_TERMIOS_C_OSPEED 1
};
// baudrate masks
#define B300 0000007
#define B600 0000010
#define B1200 0000011
#define B1800 0000012
#define B2400 0000013
#define B4800 0000014
#define B9600 0000015
#define B19200 0000016
#define B38400 0000017
#define B57600 0010001
#define B115200 0010002
#define B230400 0010003
#define B460800 0010004
#define B500000 0010005
#define B576000 0010006
#define B921600 0010007
#define B1000000 0010010
#define B1152000 0010011
#define B1500000 0010012
#define B2000000 0010013
#define B2500000 0010014
#define B3000000 0010015
#define B3500000 0010016
// C_cflag bits
#define CS5 0000000
#define CS6 0000020
#define CS7 0000040
#define CS8 0000060
#define CREAD 0000200
#define CLOCAL 0004000
#endif

#include <strings.h>

#include <cstdio>

#include <core/logger.hpp>
#include <core/types.hpp>

// Uart3 not exposed in BBB headers
enum class UartBus { kUart0 = 0, kUart1 = 1, kUart2 = 2, kUart4 = 4, kUart5 = 5 };

namespace hyped::io {

class Uart {
 public:
  static std::optional<Uart> create(
    core::ILogger &logger,
    const UartBus bus,
    const std::uint32_t baud_rate,  // TODOLater: Figure out a default for this by testing
    const std::uint8_t bits_per_bye = 8);
  ~Uart();

  core::Result send(char *tx, std::uint8_t length);
  core::Result read(unsigned char *rx, std::uint8_t length);

 private:
  Uart(core::ILogger &logger,
       const int file_descriptor,
       const std::uint32_t baud_mask,
       const std::uint8_t bits_per_byte_mask);
  // BBB can work with baudrates from 300 to 3686400 (AM335x and AMIC110 Sitara™ manual)
  static std::optional<std::uint32_t> getBaudRateMask(const std::uint32_t baud_rate);
  // Acceptable values are 5,6,7 or 8
  static std::optional<std::uint8_t> getBitsPerByteMask(const std::uint8_t bits_per_byte);
  core::ILogger &logger_;
  const std::uint32_t baud_mask_;
  const int file_descriptor_;
  struct termios tty_;
};

}  // namespace hyped::io