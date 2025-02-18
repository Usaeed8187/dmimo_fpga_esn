/* -*- c++ -*- */
/*
 * Copyright 2025 Wireless @ Virginia Tech.
 *
 */

#include "stbc_sic_decode.h"

/**
 * @brief Generalized Batch Matrix Multiplication with Broadcasting.
 *
 * Performs batch matrix multiplication for tensors in column-major order
 * without reshaping the inputs to 3D. This function supports broadcasting for
 * the batch dimensions.
 *
 * @param A Tensor with shape (M, K, A_batch1, A_batch2).
 * @param B Tensor with shape (K, N, B_batch1, B_batch2).
 * @return Output tensor AB with shape (M, N, max(A_batch1,B_batch1), max(A_batch2,B_batch2)).
 */
CTensor4D matmul_4d(
    const CTensor4D& A,
    const CTensor4D& B) 
{
    // Extract dimensions for A and B
    int M = A.dimension(0); // Rows of A
    int K = A.dimension(1); // Shared dimension
    int N = B.dimension(1); // Columns of B

    // Ensure the shared dimension matches
    assert(K == B.dimension(0) && "The shared dimension K of A and B must match.");

    // Batch dimensions
    int batch_dim_A1 = A.dimension(2); // First batch dimension of A
    int batch_dim_A2 = A.dimension(3); // Second batch dimension of A
    int batch_dim_B1 = B.dimension(2); // First batch dimension of B
    int batch_dim_B2 = B.dimension(3); // Second batch dimension of B

    // Compute broadcasted batch dimensions
    int output_batch1 = std::max(batch_dim_A1, batch_dim_B1);
    int output_batch2 = std::max(batch_dim_A2, batch_dim_B2);

    // Ensure broadcasting compatibility
    assert((batch_dim_A1 == batch_dim_B1 || batch_dim_A1 == 1 || batch_dim_B1 == 1) &&
            "Batch dimensions must either match or be 1 for broadcasting.");
    assert((batch_dim_A2 == batch_dim_B2 || batch_dim_A2 == 1 || batch_dim_B2 == 1) &&
            "Batch dimensions must either match or be 1 for broadcasting.");

    // Output tensor shape: (M, N, output_batch1, output_batch2)
    Eigen::array<int, 4> output_shape = {M, N, output_batch1, output_batch2};

    // Dynamically create the output tensor
    CTensor4D output(M, N, output_batch1, output_batch2);
    // output.resize(output_shape);
    output.setZero(); // Initialize with zeros

    // Broadcast A and B to ensure shapes align
    Eigen::array<int, 4> A_broadcast_shape = {1, 1, output_batch1 / batch_dim_A1, output_batch2 / batch_dim_A2};
    Eigen::array<int, 4> B_broadcast_shape = {1, 1, output_batch1 / batch_dim_B1, output_batch2 / batch_dim_B2};

    CTensor4D A_broadcasted = A.broadcast(A_broadcast_shape);
    CTensor4D B_broadcasted = B.broadcast(B_broadcast_shape);

    // Perform batch matrix multiplication
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            // Extract A.row(i) and B.col(j)
            CTensor3D A_row = A_broadcasted.chip(i, 0); // Shape (K, batch1, batch2)
            CTensor3D B_col = B_broadcasted.chip(j, 1); // Shape (K, batch1, batch2)

            // Multiply and sum over the shared dimension (K)
            CTensor3D product = A_row * B_col; // Element-wise multiplication
            CTensor2D sum_result = product.sum(Eigen::array<int, 1>{0}); // Sum over axis 0 (K)

            // Assign to the output tensor
            output.chip(i, 0).chip(j, 0) = sum_result;
        }
    }

    return output;
}




/**
 * @brief Perform Alamouti encoding on input symbols.
 * 
 * This function implements the Alamouti space-time block coding scheme for a given
 * set of input symbols. The input symbols are reshaped and encoded for transmission
 * across two antennas, ensuring orthogonality between the transmitted symbols.
 *
 * @param input Input tensor with dimensions (num_syms, subcarriers), where:
 *              - num_syms: Number of OFDM symbols (must be even).
 *              - subcarriers: Number of subcarriers.
 * @return A 3D tensor with dimensions (2, num_syms, subcarriers), where:
 *         - 2: Number of transmit antennas.
 *         - num_syms: Total number of OFDM symbols.    
 *         - subcarriers: Number of subcarriers.
 *
 * @note The number of symbols (num_syms) must be even.
 * @throws std::logic_error if num_syms is not even.
 * 
 * Example usage:
 * @code
 * CTensor2D input(num_syms, subcarriers);
 * input.setRandom();  // Populate with random values
 * CTensor3D encoded = alamouti_encode(input);
 * @endcode
 */
CTensor3D alamouti_encode(CTensor2D& input) {

    // Dimensions
    const int num_syms = input.dimension(0);     // Number of OFDM symbols
    const int subcarriers = input.dimension(1);  // Number of subcarriers

    // Check that the total number of symbols is even
    assert(num_syms % 2 == 0 && "Total number of symbols must be even");

    // Calculate new shape
    int num_syms_half = num_syms / 2;

    // Reshape input tensor: [..., num_syms/2, 1, 2]
    std::array<int, 4> reshape_dims = {2, 1, num_syms_half, subcarriers};
    auto reshaped = Eigen::TensorMap<CTensor4D>(input.data() , reshape_dims);

    // Extract first and second symbols
    CTensor3D x_first = reshaped.chip(0,0);
    CTensor3D x_second = reshaped.chip(1,0);

    // Compute conjugates and combine into Alamouti code
    CTensor4D x_t2(2, 1, num_syms_half, subcarriers);
    x_t2.chip(0, 0) = -x_second.conjugate();  // -conj(x_second)
    x_t2.chip(1, 0) = x_first.conjugate();  // conj(x_first)

    // Concatenate along the second-to-last dimension (axis -2, i.e., the `1` in [...,num_syms/2,2,2])
    CTensor4D x_alamouti_temp(2, 2, num_syms_half, subcarriers);
    x_alamouti_temp.chip(0,1) = reshaped.chip(0,1);
    x_alamouti_temp.chip(1,1) = x_t2.chip(0,1);

    // Reshape to final output shape: [..., num_syms, 2]
    Eigen::array<int, 3> final_shape = {2, num_syms, subcarriers};
    CTensor3D x_alamouti = Eigen::TensorMap<CTensor3D>(x_alamouti_temp.data(),final_shape);

    return x_alamouti;
}


/**
 * @brief Alamouti decoder.
 *
 * Decodes symbols received from M_r receive antennas using Alamouti decoding.
 *
 * @param r Received symbols of shape [M_r, 2, num_syms_half, num_subcarriers], where:
 * 
 *          - M_r: Number of receive antennas.
 * 
 *          - 2: First or second received symbol.
 * 
 *          - num_syms_half: Half the number of transmitted symbols.
 * 
 *          - num_subcarriers: Number of subcarriers.
 * 
 * @param h Channel estimation of shape [2, M_r, num_syms_half, num_subcarriers], where:
 * 
 *          - 2: Two transmit antennas.
 * 
 *          - M_r: Number of receive antennas.
 * 
 *          - num_syms_half: Half the number of transmitted symbols.
 * 
 *          - num_subcarriers: Number of subcarriers.
 * 
 * @return A tuple of two tensors:
 * 
 *         - z: Estimated symbols of shape [num_syms, num_subcarriers].
 * 
 *         - h_eq: Effective channel gain of shape [num_syms, num_subcarriers].
 */
std::tuple<CTensor2D, Tensor2D>
alamouti_decode(const CTensor4D& r,
                const CTensor4D& h) {
    // Dimensions
    const int M_r = r.dimension(0);                  // Number of receive antennas
    const int num_syms_half = r.dimension(2);        // num_syms / 2
    const int num_subcarriers = r.dimension(3);      // Number of subcarriers

    // Split r into r1 and r2 (equivalent to TensorFlow's slicing)
    CTensor3D r1 = r.chip(0, 1); // Slice [M_r, num_syms/2, num_subcarriers]
    CTensor3D r2 = r.chip(1, 1);

    // Split h into h1 and h2
    CTensor3D h1 = h.chip(0, 0); // Slice [M_r, num_syms/2, num_subcarriers]
    CTensor3D h2 = h.chip(1, 0);

    // z1 = h1^* * r1 + h2 * r2^*
    CTensor3D z1 = h1.conjugate() * r1 + h2 * r2.conjugate();

    // z2 = h2^* * r1 - h1 * r2^*
    CTensor3D z2 = h2.conjugate() * r1 - h1 * r2.conjugate();

    // Step 1: Reshape z1 and z2 to [M_r, 1, num_syms_half, num_subcarriers]
    Eigen::array<int, 4> z_reshaped_dims = {M_r, 1, num_syms_half, num_subcarriers};
    CTensor4D z1_reshaped = Eigen::TensorMap<CTensor4D>(z1.data(),z_reshaped_dims);
    CTensor4D z2_reshaped = Eigen::TensorMap<CTensor4D>(z2.data(),z_reshaped_dims);

    // Step 2: Concatenate along axis 1 to produce [M_r, 2, num_syms_half, num_subcarriers]
    CTensor4D z_combined = z1_reshaped.concatenate(z2_reshaped, 1);

    // Step 3: Reshape concatenated tensor to [M_r, 2 * num_syms_half, num_subcarriers]
    Eigen::array<int, 3> z_combined_dims = {M_r, 2 * num_syms_half, num_subcarriers};
    CTensor3D z_combined_reshaped = Eigen::TensorMap<CTensor3D>(z_combined.data(),z_combined_dims);

    // Step 4: Sum over receive antennas (axis 0) to get [2 * num_syms_half, num_subcarriers]
    CTensor2D z_summed = z_combined_reshaped.sum(Eigen::array<int, 1>{0});

    // Effective channel gain
    Tensor3D h_eq = h1.abs().pow(2) + h2.abs().pow(2); // [M_r, num_syms/2, num_subcarriers]
    Tensor2D h_eq_summed = h_eq.sum(Eigen::array<int, 1>{0}); // [num_syms/2, num_subcarriers]

    // Step 1: Reshape h_eq_summed to [1, num_syms/2, num_subcarriers]
    Eigen::array<int, 3> h_eq_intermediate_dims = {1, num_syms_half, num_subcarriers};
    Tensor3D h_eq_intermediate = Eigen::TensorMap<Tensor3D>(h_eq_summed.data(),h_eq_intermediate_dims);

    // Step 2: Concatenate with itself to form [2, num_syms/2, num_subcarriers]
    Tensor3D h_eq_duplicated = h_eq_intermediate.concatenate(h_eq_intermediate, 0);

    // Step 3: Reshape to final dimensions [num_syms, num_subcarriers]
    Eigen::array<int, 2> h_eq_final_dims = {num_syms_half * 2, num_subcarriers};
    Tensor2D h_eq_reshaped = Eigen::TensorMap<Tensor2D>(h_eq_duplicated.data(),h_eq_final_dims);


    return {z_summed, h_eq_reshaped};
}

// Function to compute the Hermitian transpose (conjugate transpose)
CTensor4D _hermitian(const CTensor4D& matrix) {
    Eigen::array<int, 4> shuffle_order = {1, 0, 2 , 3}; // Swap first two axes for Hermitian
    return matrix.conjugate().shuffle(shuffle_order);
}

// Function to compute the inverse of a Hermitian matrix (Zero Forcing)
CTensor4D _inv(const CTensor4D& omega_matrix) {
    CTensor4D omega_hermitian = _hermitian(omega_matrix);
    int dim0 = omega_matrix.dimension(0);
    int dim1 = omega_matrix.dimension(1);
    assert(dim0 == 2 && dim1 == 2 && "Need a square matrix of size 2x2 to do the inversion");
    int dim2 = omega_matrix.dimension(2);
    int dim3 = omega_matrix.dimension(3);

    // Compute (ΩᴴΩ) and extract the first element for normalization
    CTensor2D sigma_matrix = matmul_4d(omega_matrix,omega_hermitian).chip(0,0).chip(0,0);
    CTensor4D sigma = Eigen::TensorMap<CTensor4D>(
        sigma_matrix.data(), Eigen::array<int, 4>{1, 1, dim2, dim3}
    ).broadcast(Eigen::array<int, 4> {dim0, dim0, 1, 1});
    return omega_hermitian / sigma; // Making use of broadcasting
}



// Alamouti Zero-Forcing Decoder for Double Cluster

/**
 * @brief Alamouti Zero-Forcing Decoder for Double Cluster MIMO.
 *
 * This function implements the Alamouti decoding with Zero-Forcing (ZF) 
 * to mitigate interference and equalize the received signals in a 
 * MIMO-OFDM system with double clusters.
 *
 * The function takes in the received signal `r` and the estimated channel `h`, 
 * and applies zero-forcing equalization to recover the transmitted symbols.
 *
 * ## Input Shape Constraints:
 * 
 * - **r**: `(M_r, 2, num_syms_half, num_subcarriers)`
 *   
 *   - `M_r`: Number of receive antennas (must be even).
 * 
 *   - `2`: Represents consecutive received symbols.
 * 
 *   - `num_syms_half`: Half the total number of OFDM symbols.
 * 
 *   - `num_subcarriers`: Number of OFDM subcarriers.
 *
 * - ** h **: `(4, M_r, num_syms_half, num_subcarriers)` 
 * 
 *   - `4`: Represents the two transmit antennas from each cluster (2+2).
 * 
 *   - `M_r`: Number of receive antennas.
 * 
 *   - `num_syms_half`: Half the total number of OFDM symbols.
 * 
 *   - `num_subcarriers`: Number of OFDM subcarriers.
 *
 * ## Output:
 * 
 * - **equalized_symbols**: (2, num_syms, num_subcarriers)  
 * 
 *   - Estimated symbols after zero-forcing equalization.
 * 
 *   - The last dimension represents the stream index.
 *
 * - **noise_enhancement**: (2, num_syms, num_subcarriers)  
 * 
 *   - Represents the signal-to-noise ratio (SNR) gain of each symbol.
 * 
 *   - Used to normalize the received symbols.
 *
 * @param r Received symbols with shape (M_r, 2, num_syms_half, num_subcarriers).
 * @param h Channel estimation matrix with shape (4, M_r, num_syms_half, num_subcarriers).
 * @return A pair of tensors:
 *         - `equalized_symbols`: The estimated symbols after decoding. Shape=(2, num_syms_half * 2, num_subcarriers)
 *         - `noise_enhancement`: The SNR gain per symbol. Shape=(2, num_syms_half * 2, num_subcarriers)
 */
std::pair<CTensor3D, Tensor3D > 
alamouti_decode_zf_double(const CTensor4D& r, 
                          const CTensor4D& h) {
    
    int M_r = r.dimension(0);
    int num_syms_half = r.dimension(2);
    int num_subcarriers = r.dimension(3);
    // the input r should be of shape (M_r, 2, num_syms_half, num_subcarriers) where 2 represents consecutive symbols
    // the input h should be of shape (4, M_r, num_syms_half, num_subcarriers) where 4 represents N_t

    // Assertions
    assert(M_r % 2 == 0 && "M_r must be even.");
    assert(r.dimension(1) == 2 && "r must have 2 slots.");
    assert(h.dimension(0) == 4 && "h must have 4 transmit antennas.");
    assert(r.dimension(0) == h.dimension(1) && "Mismatch in r and h shapes for the number of receive antennas.");

    
    Eigen::array<Eigen::IndexPair<int>, 1> product_dims = { Eigen::IndexPair<int>(1, 0) };
    // Step 1: Shuffle r to swap M_r and 2
    CTensor4D r_transposed = r.shuffle(Eigen::array<int, 4>{1, 0, 2, 3}); // Swap first two axes
    // r_transposed is of shape (2, M_r, num_syms_half, num_subcarriers)

    // Step 2: Construct r_r_conj by concatenating along last axis
    CTensor4D r_r_conj_temp(2, M_r, num_syms_half, num_subcarriers);
    r_r_conj_temp.chip(0, 0) = r_transposed.chip(0, 0);
    r_r_conj_temp.chip(1, 0) = r_transposed.chip(1, 0).conjugate();
    CTensor4D r_r_conj = Eigen::TensorMap<CTensor4D>(r_r_conj_temp.data(),Eigen::array<int, 4>{2 * M_r, 1, num_syms_half, num_subcarriers});
    // r_r_conj is [r11, r21*, ... , r14, r24*] where the second index represents the receive antenna

    // Step 3: Shuffle h to swap M_r and 4 (transposing it)
    CTensor4D h_trans = h.shuffle(Eigen::array<int, 4>{1, 0, 2, 3}); // new shape (M_r, 4, num_syms_half, num_subcarriers)
    CTensor4D h_trans_conj = h_trans.conjugate();

    // Step 4: Modify h_trans_conj for Alamouti Structure
    CTensor4D h_modified(M_r , 4 , num_syms_half , num_subcarriers);
    h_modified.chip(0, 1) = h_trans_conj.chip(1, 1);
    h_modified.chip(1, 1) = -h_trans_conj.chip(0, 1);
    h_modified.chip(2, 1) = h_trans_conj.chip(3, 1);
    h_modified.chip(3, 1) = -h_trans_conj.chip(2, 1);

    // Step 5: Construct Omega matrix
    CTensor4D Omega(2 * M_r , 4 , num_syms_half , num_subcarriers);
    for (int i = 0; i < M_r; i++) {
        Omega.chip(2 * i, 0) = h_trans.chip(i, 0);
        Omega.chip(2 * i + 1, 0) = h_modified.chip(i, 0);
    }

    // Step 6: Construct Zero-Forcing Matrix
    CTensor4D zero_forcer(4, 2 * M_r, num_syms_half, num_subcarriers);
    
    // Identity Matrix (2x2) expanded for all symbols & subcarriers
    CTensor4D Eye(2, 2, num_syms_half, num_subcarriers);
    Eye.setZero();
    // Identity diagonal elements
    Eye.chip(0,0).chip(0,0).setConstant(gr_complex(1.0, 0.0)); // First chip -> axis 0 of Eye, second chip -> axis 1 of Eye
    Eye.chip(1,0).chip(1,0).setConstant(gr_complex(1.0, 0.0));
    
    for (int i = 0; i < M_r / 2; i++) {

        // Extract inverse submatrices
        Eigen::array<Eigen::Index, 4> offsets;
        Eigen::array<Eigen::Index, 4> extents;
        
        offsets = {4*i, 0, 0, 0};
        extents = {2, 2, Omega.dimension(2),Omega.dimension(3)};
        CTensor4D omega11 = Omega.slice(offsets, extents);
        CTensor4D inv_omega11 = _inv(omega11);
        
        offsets = {4*i+2, 0, 0, 0};
        CTensor4D omega12 = Omega.slice(offsets, extents);
        
        offsets = {4*i, 2, 0, 0};
        CTensor4D omega21 = Omega.slice(offsets, extents);
        
        offsets = {4*i+2, 2, 0, 0};
        CTensor4D omega22 = Omega.slice(offsets, extents);
        CTensor4D inv_omega22 = _inv(omega22);

        // Compute -inv_omega1 * Omega(2*i + 1, :)
        // auto neg_inv_omega1_Omega = -omega12.contract(inv_omega11, product_dims);
        CTensor4D neg_inv_omega1_Omega = -matmul_4d(omega12, inv_omega11);
        
        // Compute -inv_omega2 * Omega(2*i, :)
        // CTensor4D neg_inv_omega2_Omega = -omega21.contract(inv_omega22, product_dims);
        CTensor4D neg_inv_omega2_Omega = -matmul_4d(omega21, inv_omega22);

        // Assign computed blocks into zero_forcer
        offsets = {0, 4*i, 0, 0};
        extents = {2, 2, Omega.dimension(2),Omega.dimension(3)};
        zero_forcer.slice(offsets, extents) = Eye;
        
        offsets = {2, 4*i, 0, 0};
        zero_forcer.slice(offsets, extents) = neg_inv_omega1_Omega;
        
        offsets = {0, 4*i+2, 0, 0};
        zero_forcer.slice(offsets, extents) = neg_inv_omega2_Omega;

        offsets = {2, 4*i+2, 0, 0};
        zero_forcer.slice(offsets, extents) = Eye;
    }

    // Step 7: Remove interference
    // CTensor4D r_without_interference = zero_forcer.contract(r_r_conj, product_dims); // (4 , 1 , num_syms_half, num_subcarriers)
    CTensor4D r_without_interference = matmul_4d(zero_forcer, r_r_conj);
    
    // Step 8: Extract r1 and r2 using slicing
    Eigen::array<Eigen::Index, 4> offsets;
    Eigen::array<Eigen::Index, 4> extents;

    // Extract r1 (first two rows: indices 0 and 1)
    offsets = {0, 0, 0, 0};   // Start from row 0
    extents = {2, r_without_interference.dimension(1), r_without_interference.dimension(2), r_without_interference.dimension(3)};
    CTensor4D r1 = r_without_interference.slice(offsets, extents); //(2 , 1 , num_syms_half, num_subcarriers)

    // Extract r2 (last two rows: indices 2 and 3)
    offsets = {2, 0, 0, 0};   // Start from row 2
    CTensor4D r2 = r_without_interference.slice(offsets, extents); //(2 , 1 , num_syms_half, num_subcarriers)

    // Step 9: Compute new_equivalent_omega = Omega * zero_forcer
    // CTensor4D new_equivalent_omega = zero_forcer.contract(Omega, product_dims); 
    CTensor4D new_equivalent_omega = matmul_4d(zero_forcer,Omega);
    // (4, 2 * M_r , num_syms_half , num_subcarriers) by (2 * M_r , 4 , num_syms_half , num_subcarriers) becomes (4, 4, num_syms_half , num_subcarriers)

    // Step 10: Extract omega1 and omega2 using slicing
    offsets = {0, 0, 0, 0};   // First 2x2 block from new_equivalent_omega
    extents = {2, 2, new_equivalent_omega.dimension(2), new_equivalent_omega.dimension(3)};
    CTensor4D omega1 = new_equivalent_omega.slice(offsets, extents);
    // (2, 2, num_syms_half , num_subcarriers)

    offsets = {2, 2, 0, 0};   // Last 2x2 block
    CTensor4D omega2 = new_equivalent_omega.slice(offsets, extents);
    // (2, 2, num_syms_half , num_subcarriers)

    // Step 11: Compute first and second cluster symbols
    CTensor4D omega1_inv =  _inv(omega1);
    CTensor4D omega2_inv =  _inv(omega2);
    // CTensor4D first_cluster_symbol = omega1_inv.contract(r1, product_dims);  // Shape: (2,1, num_syms_half , num_subcarriers))
    CTensor4D first_cluster_symbol = matmul_4d(omega1_inv, r1);
    // CTensor4D second_cluster_symbol = omega2_inv.contract(r2, product_dims);  // Shape: (2,1, num_syms_half , num_subcarriers)
    CTensor4D second_cluster_symbol = matmul_4d(omega2_inv, r2);
    // first_cluster_symbol is s1 and s2 while second_cluster_symbol is s3 and s4

    // Step 12: Reshape and stack results into equalized_symbols
    CTensor3D equalized_symbols(2, num_syms_half * 2, num_subcarriers);
    equalized_symbols.chip(0, 0) = Eigen::TensorMap<CTensor2D>(first_cluster_symbol.data(),Eigen::array<int, 2>{num_syms_half * 2, num_subcarriers});
    equalized_symbols.chip(1, 0) = Eigen::TensorMap<CTensor2D>(second_cluster_symbol.data(),Eigen::array<int, 2>{num_syms_half * 2, num_subcarriers});
    // Step 13: Compute noise enhancement
    // CTensor4D noise_enhancer_cov = (zero_forcer).contract(_hermitian(zero_forcer), product_dims); 
    CTensor4D noise_enhancer_cov = matmul_4d(zero_forcer, _hermitian(zero_forcer));
    //                                    (4, 4, num_syms_half , num_subcarriers))

    offsets = {0, 0, 0, 0};   // First 2x2 block from new_equivalent_omega
    extents = {2, 2, noise_enhancer_cov.dimension(2), noise_enhancer_cov.dimension(3)};
    CTensor4D cluster1_noise_enhancer_cov = matmul_4d(omega1_inv,
        matmul_4d(noise_enhancer_cov.slice(offsets, extents), _hermitian(omega1_inv))) ;
    

    offsets = {2, 2, 0, 0};
    CTensor4D cluster2_noise_enhancer_cov = matmul_4d(omega2_inv,
        matmul_4d(noise_enhancer_cov.slice(offsets, extents), _hermitian(omega2_inv))) ;

    CTensor3D cluster1_noise_power_temp(2, num_syms_half, num_subcarriers);
    cluster1_noise_power_temp.chip(0,0) = cluster1_noise_enhancer_cov.chip(0,0).chip(0,0); // cluster1_noise_enhancer_cov[0,0,...]
    cluster1_noise_power_temp.chip(1,0) = cluster1_noise_enhancer_cov.chip(1,0).chip(1,0); // cluster1_noise_enhancer_cov[1,1,...]
    CTensor2D cluster1_noise_power = Eigen::TensorMap<CTensor2D>(cluster1_noise_power_temp.data(),Eigen::array<int, 2>{num_syms_half * 2, num_subcarriers});

    CTensor3D cluster2_noise_power_temp(2, num_syms_half, num_subcarriers);
    cluster2_noise_power_temp.chip(0,0) = cluster2_noise_enhancer_cov.chip(0,0).chip(0,0); // cluster2_noise_enhancer_cov[0,0,...]
    cluster2_noise_power_temp.chip(1,0) = cluster2_noise_enhancer_cov.chip(1,0).chip(1,0); // cluster2_noise_enhancer_cov[1,1,...]
    CTensor2D cluster2_noise_power = Eigen::TensorMap<CTensor2D>(cluster2_noise_power_temp.data(),Eigen::array<int, 2>{num_syms_half * 2, num_subcarriers});

    Tensor3D both_noise_power(2, num_syms_half * 2, num_subcarriers);
    both_noise_power.chip(0,0) = cluster1_noise_power.abs();
    both_noise_power.chip(1,0) = cluster2_noise_power.abs();
    
    return {equalized_symbols, (1.0/both_noise_power)};
}

// ZF + SIC function
/**
 * @brief Alamouti Zero-Forcing Decoder for Double Cluster MIMO.
 *
 * This function implements the Alamouti decoding with Zero-Forcing (ZF) 
 * to mitigate interference and equalize the received signals in a 
 * MIMO-OFDM system with double clusters.
 *
 * The function takes in the received signal `r` and the estimated channel `h`, 
 * and applies zero-forcing equalization to recover the transmitted symbols.
 *
 * ## Input Shape Constraints:
 * 
 * - **r**: `(M_r, 2, num_syms_half, num_subcarriers)`
 *   
 *   - `M_r`: Number of receive antennas (must be even).
 *  
 *   - `num_syms`: The total number of OFDM symbols.
 * 
 *   - `num_subcarriers`: Number of OFDM subcarriers.
 *
 * - ** h **: `(M_r, N_t , num_syms_half, num_subcarriers)` 
 * 
 *   - `M_r`: Number of receive antennas.
 * 
 *   - `N_t = 4`: Represents the 4 transmit antennas, two for each cluster (2+2).
 * 
 *   - `num_syms_half`: Half the total number of OFDM symbols.
 * 
 *   - `num_subcarriers`: Number of OFDM subcarriers.
 *
 * ## Output:
 * 
 * - **equalized_symbols**: (2, num_syms, num_subcarriers)  
 * 
 *   - Estimated symbols after zero-forcing equalization.
 * 
 *   - The last dimension represents the stream index.
 *
 * - **noise_enhancement**: (2, num_syms, num_subcarriers)  
 * 
 *   - Represents the signal-to-noise ratio (SNR) gain of each symbol.
 * 
 *   - Used to normalize the received symbols.
 *
 * @param r Received symbols with shape (M_r, num_syms, num_subcarriers).
 * @param h Channel estimation matrix with shape (M_r , N_t, num_syms, num_subcarriers) with N_t=4.
 * @return A pair of tensors:
 *         - `equalized_symbols`: The estimated symbols after decoding. Shape=(2, num_syms_half * 2, num_subcarriers)
 *         - `gains`: The SNR gain per symbol. Shape=(2, num_syms_half * 2, num_subcarriers)
 */
std::pair<CTensor3D, Tensor3D> alamouti_decode_zf_sic_double(
    CTensor3D& r, 
    CTensor4D& h,
    const int num_bits_per_symbol
)
{
    int M_r = r.dimension(0);
    int num_syms = r.dimension(1);
    int num_subcarriers = r.dimension(2);
    int num_syms_half = num_syms/2;
    assert(M_r % 2 == 0 && "M_r must be even.");
    assert(h.dimension(0) == 4 && "N_t (fist dimension of h) must be 4.");
    assert(h.dimension(1) == M_r && "Mismatch between M_r of the received signal and channel.");
    assert(h.dimension(2) == num_syms && "Mismatch between num_syms of the received signal and channel.");
    assert(h.dimension(3) == num_subcarriers && "Mismatch between num_subcarriers of the received signal and channel.");

    CTensor4D r_reshaped = Eigen::TensorMap<CTensor4D>(r.data(), Eigen::array<int,4>{M_r, 2, num_syms_half, num_subcarriers});

    Eigen::array<int, 4> transpose_dims = {1, 0, 2, 3}; // Transpose to [N_t, M_r, num_syms, num_subcarriers]
    CTensor4D h_transposed = h.shuffle(transpose_dims);
    Eigen::array<int, 5> reshape_dims = {4, M_r, 2, num_syms_half, num_subcarriers}; // Reshape to [N_t, M_r, 2, num_syms/2, num_subcarriers]
    CTensor5D h_reshaped = Eigen::TensorMap<CTensor5D>(h_transposed.data(),reshape_dims);
    CTensor4D h_avg = (h_reshaped.chip(0, 2) + h_reshaped.chip(1, 2)) / gr_complex(2.0f, 0.0f); //[N_t, M_r, num_syms/2, num_subcarriers]
    
    // the input r should be of shape (M_r, num_syms, num_subcarriers) where 2 represents consecutive symbols
    // the input h should be of shape (4, M_r, num_syms_half, num_subcarriers) where 4 represents N_t
    auto [y, gains] = alamouti_decode_zf_double(r_reshaped, h_avg); // y: [2, num_syms, num_subcarriers], gains: [2, num_syms, num_subcarriers]
    
    CTensor2D comparison = (gains.chip(0,0) >= gains.chip(1,0)).cast<gr_complex>();
    CTensor4D cluster0isbetter = Eigen::TensorMap<CTensor4D>(comparison.data(),Eigen::array<int, 4> {1,1,num_syms, num_subcarriers});
    comparison = (gains.chip(0,0) < gains.chip(1,0)).cast<gr_complex>() ;
    CTensor4D cluster1isbetter = Eigen::TensorMap<CTensor4D>(comparison.data(),Eigen::array<int, 4> {1,1,num_syms, num_subcarriers});
    // shape = (num_syms_half * 2, num_subcarriers)
    
    QAMModulator modulator(num_bits_per_symbol);
    
    CTensor2D y_chipped = y.chip(0,0);
    CTensor4D x0 = modulator.remap_4d(
        Eigen::TensorMap<CTensor4D>(y_chipped.data(),Eigen::array<int, 4> {1,1,num_syms, num_subcarriers})
    );
    
    y_chipped = y.chip(1,0);
    CTensor4D x1 = modulator.remap_4d(
        Eigen::TensorMap<CTensor4D>(y_chipped.data(),Eigen::array<int, 4> {1,1,num_syms, num_subcarriers})
    ); // TODO: Use modulator2 here

    CTensor4D better_x = cluster0isbetter * x0 + cluster1isbetter * x1;
    Eigen::array<Eigen::Index, 4> offsets = {0, 0, 0, 0};
    Eigen::array<Eigen::Index, 4> extents = {h.dimension(0), 2, h.dimension(2),h.dimension(3)};
    CTensor4D channel0 = h.slice(offsets, extents);
    offsets = {0, 2, 0, 0};
    CTensor4D channel1 = h.slice(offsets, extents);

    CTensor2D better_x_reshaped = Eigen::TensorMap<CTensor2D>(better_x.data(),Eigen::array<int,2> {num_syms, num_subcarriers});
    CTensor4D better_x_alamouti = Eigen::TensorMap<CTensor4D>(
        alamouti_encode(better_x_reshaped).data(),Eigen::array<int,4> {2, 1 , num_syms , num_subcarriers} ); // {2, 1 , num_syms , num_subcarriers}
    
    CTensor4D channel_better_cluster = (
        channel0 * cluster0isbetter.broadcast(Eigen::array<int, 4> {M_r, 2, 1, 1})+
        channel1 * cluster1isbetter.broadcast(Eigen::array<int, 4> {M_r, 2, 1, 1})
    ); // {M_r, 2, num_syms, num_subcarriers}
    CTensor4D channel_worse_cluster = (
        channel0 * cluster1isbetter.broadcast(Eigen::array<int, 4> {M_r, 2, 1, 1})+
        channel1 * cluster0isbetter.broadcast(Eigen::array<int, 4> {M_r, 2, 1, 1})
    ); // {M_r, 2, num_syms, num_subcarriers}
    
    CTensor4D better_cluster_effect = Eigen::TensorMap<CTensor4D>(
        matmul_4d(channel_better_cluster, better_x_alamouti).data(),
        Eigen::array<int,4> {M_r, 2, num_syms_half, num_subcarriers}
    ); // {M_r, 2, num_syms_half, num_subcarriers}

    // Remove the effect form the received signal
    CTensor4D ry_all_new = r_reshaped - better_cluster_effect; // {M_r, 2, num_syms_half, num_subcarriers}
    Eigen::array<int, 4> stride_array = {1, 1 , 2, 1}; // Choose every second element in the num_syms dimension
    offsets = {0,0,1,0};
    extents = {M_r, 2, num_syms-1, num_subcarriers};
    CTensor4D channel_worse_cluster_reshaped = (channel_worse_cluster.stride(stride_array) +
     channel_worse_cluster.slice(offsets, extents).stride(stride_array))/gr_complex(2, 0); // {M_r, 2, num_syms/2, num_subcarriers}
    
    auto [new_y , new_SNR] = alamouti_decode(ry_all_new, channel_worse_cluster_reshaped.shuffle(Eigen::array<int,4> {1,0,2,3})) ;
    new_y = new_y / (new_SNR.cast<gr_complex>());
    CTensor4D new_y_reshaped = Eigen::TensorMap<CTensor4D>(new_y.data(),Eigen::array<int,4> {1, 1, num_syms, num_subcarriers}) ;

    CTensor4D new_x =  new_y_reshaped;
    // TODO: Make this modulator1 and modulator2 if the modulation order is different across clusters
    x0 = cluster0isbetter * better_x + cluster1isbetter * new_x ;
    x1 = cluster1isbetter * better_x + cluster0isbetter * new_x ;

    CTensor3D equalized_x(2, num_syms, num_subcarriers);
    equalized_x.chip(0,0) = x0.chip(0,0).chip(0,0);
    equalized_x.chip(1,0) = x1.chip(0,0).chip(0,0);

    return {equalized_x, gains};
    // Demodulate
    
}
