#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "robo_cleaner_controller/external_api/CleanerGuiExternalBridge.h"

static void runApp(
    const std::shared_ptr<CleanerGuiExternalBridge> &node) {
  node->run();
}

int32_t main(int32_t argc, char *argv[]) {
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = false;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CleanerGuiExternalBridge>();
  const int32_t errCode = node->init();
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "CleanerGuiExternalBridge::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  std::thread spinThread([&node]() {
    rclcpp::spin(node);
    node->shutdown();
  });

  runApp(node);

  rclcpp::shutdown();
  spinThread.join();

  return EXIT_SUCCESS;
}
