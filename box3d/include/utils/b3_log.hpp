#ifndef BOX3D_B3_LOG_HPP
#define BOX3D_B3_LOG_HPP


#include <spdlog/spdlog.h>

#include <Eigen/Core>

/// @brief Format an eigen MatrixXd
std::string b3_matrix_str(const Eigen::MatrixXd& x, int precision = 16);

const std::vector<std::pair<std::string, spdlog::level::level_enum>> SPDLOG_LEVEL_NAMES_TO_LEVELS =
        { { "trace",   spdlog::level::trace    },
          { "debug",   spdlog::level::debug    },
          { "info",    spdlog::level::info     },
          { "warning", spdlog::level::warn     },
          { "error",   spdlog::level::err      },
          { "critical",spdlog::level::critical },
          { "off",     spdlog::level::off      } };

void set_logger_level(spdlog::level::level_enum log_level);


#endif //BOX3D_B3_LOG_HPP
