#pragma once

#include <iostream>
#include <Eigen/Dense>

#define RESET "\033[0m"
#define RED "\033[31m"  /* Red */
#define BLUE "\033[34m" /* Blue */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define CYAN "\033[36m" /* Cyan */
#define MAGENTA "\033[35m" /* Magenta */

#define FUNC_TAG "[" << __func__ << "] "
const Eigen::IOFormat fmt(Eigen::StreamPrecision, 0, ", ", "\n", "[ ", " ]");
const Eigen::IOFormat full_fmt(Eigen::FullPrecision, 0, ", ", "\n", "[ ", " ]");