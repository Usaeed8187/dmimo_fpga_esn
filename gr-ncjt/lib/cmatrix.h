//
// Created by donald on 2/11/25.
//

#ifndef GR_NCJT_LIB_CMATRIX_H
#define GR_NCJT_LIB_CMATRIX_H

#include <Eigen/Core>
#include <Eigen/Dense>

typedef Eigen::Matrix<std::complex<float>, -1, -1, Eigen::RowMajor> CMatrixX;

typedef Eigen::Matrix<std::complex<float>, 2, 2, Eigen::RowMajor> CMatrix2;
typedef Eigen::Matrix<std::complex<float>, 4, 4, Eigen::RowMajor> CMatrix4;

#endif //GR_NCJT_LIB_CMATRIX_H
