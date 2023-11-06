#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include <iostream>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <core/logger.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <core/wall_clock.hpp>
#include <io/can.hpp>
#include <io/hardware_can.hpp>
#include <io/hardware_gpio.hpp>
#include <io/hardware_i2c.hpp>
#include <io/pwm.hpp>
#include <motors/constant_frequency_calculator.hpp>
#include <motors/controller.hpp>
#include <sensors/accelerometer.hpp>
#include <sensors/i2c_mux.hpp>
#include <sensors/keyence.hpp>
#include <utils/manual_time.hpp>

// global
bool should_brake = false;
bool should_go    = false;

// threading to keep a background process updating the keyence data
void keep_updating_keyence(std::shared_ptr<hyped::sensors::Keyence> keyence,
                           int sockfd,
                           int num_poles)
{
  int current_highest_stripe_count = 0;
  while (true) {
    keyence->updateStripeCount();
    const std::uint8_t stripe_count = keyence->getStripeCount();
    if (stripe_count >= num_poles) {
      std::cout << "begin brake procedure, we have read" << stripe_count << " stripes already"
                << "\n";
      should_brake = true;
    }
    if (stripe_count < current_highest_stripe_count) {
      std::cout << "keyence data decreased, ignoring"
                << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    std::cout << "cur(r fk u kshitij)ent stripe count: " << stripe_count << "\n";
    const float distance = stripe_count * 6;
    // get the stripe count in json format
    std::string json = "{\"keyence_1\": " + std::to_string(stripe_count)
                       + ", \"keyence_2\": " + std::to_string(stripe_count)
                       + ", \"displacement\": " + std::to_string(distance) + "}";
    send(sockfd, json.c_str(), json.length(), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (should_brake) { break; }
  }
}

void log_accelerometer(std::shared_ptr<hyped ::sensors::Accelerometer> accelerometer, int file_fd)
{
  while (true) {
    if (accelerometer->isValueReady() == hyped::core::Result::kFailure) { continue; }
    const auto optional_data = accelerometer->read();
    if (!optional_data) { continue; }
    const auto data    = *optional_data;
    std::string record = std::to_string(data.measured_at.time_since_epoch().count()) + ", "
                         + std::to_string(data.x) + ", " + std::to_string(data.y) + ", "
                         + std::to_string(data.z) + "\n";
    send(file_fd, record.c_str(), record.length(), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void keep_receiving_commands(int sockfd2)
{
  char buffer[1024] = {0};
  while (true) {
    const int valread = read(sockfd2, buffer, 1024);
    if (valread < 0) {
      std::cout << "Failed to read from socket"
                << "\n";
      std::cout << "Error: " << strerror(errno) << "\n";
      continue;
    }
    // convert received message to string
    std::string received(buffer);
    if (received == "go") {
      std::cout << "Received go command"
                << "\n";
      should_go = true;
    } else if (received == "stop") {
      std::cout << "Received stop command"
                << "\n";
      should_brake = true;
    } else {
      std::cout << "Received invalid command: " << received << "\n";
    }
  }
}

std::optional<std::shared_ptr<hyped::io::ICan>> getCan(hyped::core::ILogger &logger,
                                                       const std::string &bus)
{
  const auto new_can = hyped::io::HardwareCan::create(logger, bus);
  if (!new_can) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to create CAN1 instance");
    return std::nullopt;
  }
  return *new_can;
}

int main(int argc, char **argv)
{
  // read number of poles from args
  if (argc != 2) {
    std::cout << "Usage: ./{name_of_binary} <number of poles before we brake>"
              << "\n";
    return -1;
  }
  const int num_poles = std::stoi(argv[1]);
  hyped::core::WallClock time;
  hyped::core::Timer timer(time);
  hyped::core::Logger logger("Pod", hyped::core::LogLevel::kWarn, time);
  // Connect to receiver TCP server
  int sockfd2 = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd2 < 0) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to create socket");
    return -1;
  }

  struct sockaddr_in serv_addr2;
  serv_addr2.sin_family      = AF_INET;
  serv_addr2.sin_port        = htons(65432);
  serv_addr2.sin_addr.s_addr = inet_addr("192.168.6.1");

  // Set socket to non-blocking
  int flags = fcntl(sockfd2, F_GETFL, 0);
  if (flags == -1) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to get socket flags");
    return -1;
  }

  if (fcntl(sockfd2, F_SETFL, flags | O_NONBLOCK) == -1) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to set socket to non-blocking");
    return -1;
  }

  // Connect to TCP server
  if (connect(sockfd2, (struct sockaddr *)&serv_addr2, sizeof(serv_addr2)) < 0
      && errno != EINPROGRESS) {
    // Print error message and errno
    logger.log(hyped::core::LogLevel::kFatal, "Failed to connect to socket");
    logger.log(hyped::core::LogLevel::kFatal, "Error: %s", strerror(errno));
    return -1;
  }
  // set up a thread to keep receiving commands from the server
  std::thread t2(keep_receiving_commands, sockfd2);
  t2.detach();

  if (!should_go) {
    std::cout << "Waiting for go command"
              << "\n";
    while (!should_go) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  /* big man can */
  const auto optional_can = getCan(logger, "can1");
  if (!optional_can) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to create CAN instance");
    return -1;
  }
  const auto can = std::move(*optional_can);
  /* getting controllerrrrr */
  const auto frequency_calculator
    = std::make_shared<hyped::motors::ConstantFrequencyCalculator>(logger);
  frequency_calculator->setFrequency(10);
  const auto optional_controller
    = hyped::motors::Controller::create(logger, "messages.json", can, frequency_calculator);
  if (!optional_controller) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to create motor controller instance");
    return -1;
  }
  auto controller = std::move(*optional_controller);
  // running pre-op on controller - IDLE
  hyped::io::CanFrame preop_frame;
  // #  can1 000 [2] 80 00
  preop_frame.can_id               = 0x000;
  preop_frame.can_dlc              = 0x02;
  preop_frame.data[0]              = 0x80;
  preop_frame.data[1]              = 0x00;
  hyped::core::Result preop_result = can->send(preop_frame);
  if (preop_result == hyped::core::Result::kFailure) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to send pre-operational command");
    return -1;
  }
  // running operational on controller - READY
  hyped::io::CanFrame op_frame;
  // #  can1 000 [2] 01 00
  op_frame.can_id               = 0x000;
  op_frame.can_dlc              = 0x02;
  op_frame.data[0]              = 0x01;
  op_frame.data[1]              = 0x00;
  hyped::core::Result op_result = can->send(op_frame);
  if (op_result == hyped::core::Result::kFailure) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to send operational command");
    return -1;
  }
  // ACCELERATING
  /* gee-pee-I-oh */
  const auto gpio             = std::make_shared<hyped::io::HardwareGpio>(logger);
  const auto optional_keyence = hyped::sensors::Keyence::create(
    logger, gpio, 66);  // 66 is the pin number for keyence u dumbo
  if (!optional_keyence) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to create keyence instance");
    return -1;
  }
  const auto keyence = *optional_keyence;
  const auto optional_brake_pin
    = gpio->getWriter(1, hyped::io::Edge::kRising);  // TODO change this to the correct pin
  if (!optional_brake_pin) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to get brake pin");
    return -1;
  }
  const auto brake_pin             = *optional_brake_pin;
  const auto retract_brakes_result = brake_pin->write(hyped::core::DigitalSignal::kLow);
  if (retract_brakes_result == hyped::core::Result::kFailure) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to retract brakes");
    return -1;
  }
  // set up a thread to update keyence at every 100ms
  // std::thread t(keep_updating_keyence, keyence, sockfd, num_poles);
  // t.detach();
  // set up a thread to log accelerometer data
  const auto optional_i2c
    = hyped::io::HardwareI2c::create(logger, 1);  // TODO change this to the correct bus
  if (!optional_i2c) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to create I2C instance");
    return -1;
  }
  const auto i2c                    = *optional_i2c;
  const auto optional_accelerometer = hyped::sensors::Accelerometer::create(
    logger, i2c, 1, 0x19);  // TODO change this to the correct address
  if (!optional_accelerometer) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to create accelerometer instance");
    return -1;
  }
  const auto accelerometer = *optional_accelerometer;
  int file_fd              = open("accelerometer_data.csv", O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (file_fd < 0) {
    logger.log(hyped::core::LogLevel::kFatal, "Failed to open file");
    return -1;
  }
  std::thread t3(log_accelerometer, accelerometer, file_fd);
  t3.detach();
  controller->setCurrent(0);
  // setting up the motor to vrooom vrooom
  for (int i = 200; i <= 1000; i += 200) {
    controller->setCurrent(i);
    const auto accelerate_result = controller->run(hyped::motors::FauxState::kAccelerate);
    if (accelerate_result == hyped::core::Result::kFailure) {
      logger.log(hyped::core::LogLevel::kFatal, "Failed to accelerate");
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  // STOPPING
  for (int i = 1000; i >= 0; i -= 200) {
    const auto accelerate_result = controller->run(hyped::motors::FauxState::kStop);
    if (accelerate_result == hyped::core::Result::kFailure) {
      logger.log(hyped::core::LogLevel::kFatal, "Failed to accelerate");
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  brake_pin->write(hyped::core::DigitalSignal::kHigh);
  // SHUTTING DOWN
  std::cout << "your mum lol, bye...."
            << "\n";
  return 0;
}