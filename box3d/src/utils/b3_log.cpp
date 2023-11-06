
#include "utils/b3_log.hpp"

#include <iomanip>

#include <Eigen/Core>


void set_logger_level(spdlog::level::level_enum log_level) {
    spdlog::set_level(log_level);
}

static const Eigen::IOFormat RowVectorFmt(Eigen::FullPrecision,
                                          Eigen::DontAlignCols,
                                          ", ",
                                          ",\n",
                                          "[",
                                          "]",
                                          "",
                                          "");

static const Eigen::IOFormat MatrixFmt(Eigen::FullPrecision,
                                       Eigen::DontAlignCols,
                                       ", ",
                                       ",\n",
                                       "[",
                                       "]",
                                       "[",
                                       "]");

std::string b3_matrix_str(const Eigen::MatrixXd& x, const int precision)
{
    std::stringstream ssx;
    Eigen::MatrixXd m = x;
    if (m.cols() == 1) {
        m.transposeInPlace();
    }
    ssx << std::setprecision(precision);
    if (m.rows() == 1) {
        ssx << m.format(RowVectorFmt);
    } else {
        ssx << m.format(MatrixFmt);
    }
    return ssx.str();
}