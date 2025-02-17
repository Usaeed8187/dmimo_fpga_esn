//
// Created by donald on 2/11/25.
//

#ifndef GR_NCJT_LIB_CMATRIX_H
#define GR_NCJT_LIB_CMATRIX_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

typedef Eigen::Matrix<std::complex<float>, -1, -1, Eigen::RowMajor> CMatrixX;
typedef Eigen::Matrix<std::complex<float>, 2, 2, Eigen::RowMajor> CMatrix2;
typedef Eigen::Matrix<std::complex<float>, 4, 4, Eigen::RowMajor> CMatrix4;

typedef Eigen::Tensor<gr_complex, 2, Eigen::RowMajor> CTensor2D;
typedef Eigen::Tensor<gr_complex, 3, Eigen::RowMajor> CTensor3D;
typedef Eigen::Tensor<gr_complex, 4, Eigen::RowMajor> CTensor4D;

typedef Eigen::Tensor<float, 0, Eigen::RowMajor> TensorScalar;
typedef Eigen::Tensor<float, 1, Eigen::RowMajor> Tensor1D;
typedef Eigen::Tensor<float, 2, Eigen::RowMajor> Tensor2D;
typedef Eigen::Tensor<float, 3, Eigen::RowMajor> Tensor3D;
typedef Eigen::Tensor<float, 4, Eigen::RowMajor> Tensor4D;

#endif //GR_NCJT_LIB_CMATRIX_H
