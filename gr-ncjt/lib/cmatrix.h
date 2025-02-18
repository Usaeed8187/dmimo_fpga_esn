//
// Created by donald on 2/11/25.
//

#ifndef GR_NCJT_LIB_CMATRIX_H
#define GR_NCJT_LIB_CMATRIX_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

typedef Eigen::Matrix<std::complex<float>, -1, -1> CMatrixX;
typedef Eigen::Matrix<std::complex<float>, 2, 2> CMatrix2;
typedef Eigen::Matrix<std::complex<float>, 4, 4> CMatrix4;

typedef Eigen::Tensor<gr_complex, 1> CTensor1D;
typedef Eigen::Tensor<gr_complex, 2> CTensor2D;
typedef Eigen::Tensor<gr_complex, 3> CTensor3D;
typedef Eigen::Tensor<gr_complex, 4> CTensor4D;
typedef Eigen::Tensor<gr_complex, 5> CTensor5D;

typedef Eigen::Tensor<float, 0> TensorScalar;
typedef Eigen::Tensor<float, 1> Tensor1D;
typedef Eigen::Tensor<float, 2> Tensor2D;
typedef Eigen::Tensor<float, 3> Tensor3D;
typedef Eigen::Tensor<float, 4> Tensor4D;

#endif //GR_NCJT_LIB_CMATRIX_H
