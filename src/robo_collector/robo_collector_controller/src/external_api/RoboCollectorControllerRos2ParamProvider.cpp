//Corresponding header
#include "robo_collector_controller/external_api/RoboCollectorControllerRos2ParamProvider.h"

//System headers
#include <sstream>

//Other libraries headers
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto NODE_NAME = "RoboCollectorControllerRos2ParamProvider";

constexpr auto GUI_WINDOW_X_PARAM_NAME = "gui_window_x";
constexpr auto GUI_WINDOW_Y_PARAM_NAME = "gui_window_y";
constexpr auto GUI_WINDOW_WIDTH_PARAM_NAME = "gui_window_width";
constexpr auto GUI_WINDOW_HEIGHT_PARAM_NAME = "gui_window_height";
constexpr auto USE_LOCAL_CONTROLLER_MODE_PARAM_NAME =
    "use_local_controller_mode";

//screen
constexpr auto DEFAULT_WINDOW_X = 1272;
constexpr auto DEFAULT_WINDOW_Y = 527;
constexpr auto DEFAULT_WINDOW_WIDTH = 648;
constexpr auto DEFAULT_WINDOW_HEIGHT = 553;

//misc
constexpr auto DEFAULT_USE_LOCAL_CONTROLLER_MODE = true;

template<typename T>
void handleParamError(const char* paramName, T& value, const T& defaultValue) {
  std::ostringstream ostr;
  ostr << "Param: [" << paramName << "] has invalid value: [" << value
       << "]. Overriding with default value: [" << defaultValue << "]";
  LOGR("%s", ostr.str().c_str());

  value = defaultValue;
}
}

void RoboCollectorControllerRos2Params::print() const {
  std::ostringstream ostr;
  ostr << "==================================================================\n"
      << "Printing node(" << NODE_NAME << ") params:\n"
       << GUI_WINDOW_X_PARAM_NAME << ": " << guiWindow.x << '\n'
       << GUI_WINDOW_Y_PARAM_NAME << ": " << guiWindow.y << '\n'
       << GUI_WINDOW_WIDTH_PARAM_NAME << ": " << guiWindow.w << '\n'
       << GUI_WINDOW_HEIGHT_PARAM_NAME << ": " << guiWindow.h << '\n'
       << USE_LOCAL_CONTROLLER_MODE_PARAM_NAME << ": "
           << ((LocalControllerMode::ENABLED == localControrllerMode) ?
               "true" : "false") << '\n'
       << "=================================================================\n";

  LOG("%s", ostr.str().c_str());
}

void RoboCollectorControllerRos2Params::validate() {
  if (0 >= guiWindow.w) {
    handleParamError(GUI_WINDOW_WIDTH_PARAM_NAME, guiWindow.w,
        DEFAULT_WINDOW_WIDTH);
  }
  if (0 >= guiWindow.h) {
    handleParamError(GUI_WINDOW_HEIGHT_PARAM_NAME, guiWindow.h,
        DEFAULT_WINDOW_HEIGHT);
  }
}

RoboCollectorControllerRos2ParamProvider::RoboCollectorControllerRos2ParamProvider()
    : rclcpp::Node(NODE_NAME) {
  declare_parameter<int32_t>(GUI_WINDOW_X_PARAM_NAME, DEFAULT_WINDOW_X);
  declare_parameter<int32_t>(GUI_WINDOW_Y_PARAM_NAME, DEFAULT_WINDOW_Y);
  declare_parameter<int32_t>(GUI_WINDOW_WIDTH_PARAM_NAME, DEFAULT_WINDOW_WIDTH);
  declare_parameter<int32_t>(GUI_WINDOW_HEIGHT_PARAM_NAME,
      DEFAULT_WINDOW_HEIGHT);
  declare_parameter<bool>(USE_LOCAL_CONTROLLER_MODE_PARAM_NAME,
      DEFAULT_USE_LOCAL_CONTROLLER_MODE);
}

RoboCollectorControllerRos2Params RoboCollectorControllerRos2ParamProvider::getParams() {
  get_parameter(GUI_WINDOW_X_PARAM_NAME, _params.guiWindow.x);
  get_parameter(GUI_WINDOW_Y_PARAM_NAME, _params.guiWindow.y);
  get_parameter(GUI_WINDOW_WIDTH_PARAM_NAME, _params.guiWindow.w);
  get_parameter(GUI_WINDOW_HEIGHT_PARAM_NAME, _params.guiWindow.h);

  bool useLocalControllerMode{};
  get_parameter(USE_LOCAL_CONTROLLER_MODE_PARAM_NAME, useLocalControllerMode);
  _params.localControrllerMode = useLocalControllerMode ?
      LocalControllerMode::ENABLED : LocalControllerMode::DISABLED;

  _params.validate();

  return _params;
}
