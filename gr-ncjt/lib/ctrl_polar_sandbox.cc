#include "ctrl.h"
#include <iostream>
#include <vector>
#include <iomanip>   // for std::hex etc.

using namespace gr::ncjt;

int main()
{
    // ------------------------------------------------------------------
    // Example 1: QPSK demonstration
    //  - Only 64 bits (control word) are encoded at rate 1/2 => 128 bits => 64 QPSK symbols.
    //  - The extended 64 bits are ignored in QPSK mode.
    // ------------------------------------------------------------------
    {
        // Create a CTRL object (debug=true to show debug prints)
        CTRL ctrl_tx_qpsk(false);

        // Set some fields
        ctrl_tx_qpsk.set_seq_number(1234);
        ctrl_tx_qpsk.set_reserved(0xF);
        ctrl_tx_qpsk.set_nstrm_phase1(1);
        ctrl_tx_qpsk.set_mod_type_phase1(2);
        ctrl_tx_qpsk.set_coding_rate_phase1(5);
        ctrl_tx_qpsk.set_nstrm_phase2(1);
        ctrl_tx_qpsk.set_mod_type_phase2(3);
        ctrl_tx_qpsk.set_coding_rate_phase2(4);
        ctrl_tx_qpsk.set_nstrm_phase3(1);
        ctrl_tx_qpsk.set_mod_type_phase3(1);
        ctrl_tx_qpsk.set_coding_rate_phase3(7);

        // For demonstration, set data_checksum and extended
        ctrl_tx_qpsk.set_data_checksum(0xABCD);
        ctrl_tx_qpsk.set_extended(0x9999999999999999ULL); // will be ignored in QPSK path

        // Pack + modulate => 64 QPSK symbols
        std::vector<gr_complex> syms_qpsk = ctrl_tx_qpsk.pack_and_modulate_qpsk();
        std::cout << "[QPSK] Generated " << syms_qpsk.size() << " symbols." << std::endl;

        // Simulate receive side
        CTRL ctrl_rx_qpsk(false /*debug*/);
        std::vector<gr_complex> csi_qpsk(syms_qpsk.size(), gr_complex(1.0, 0.0)); // CSI = 1.0 + j*0.0
        bool ok_qpsk = ctrl_rx_qpsk.demodulate_and_unpack_qpsk(syms_qpsk, csi_qpsk);
        std::cout << "[QPSK] Demod/Unpack success? " << (ok_qpsk ? "YES" : "NO") << std::endl;

        if(ok_qpsk) {
            // Print out final fields
            std::cout << " seq_number:         " << ctrl_rx_qpsk.get_seq_number() << std::endl;
            std::cout << " reserved(4 bits):   0x" << std::hex << (int)ctrl_rx_qpsk.get_reserved() << std::dec << std::endl;
            std::cout << " nstrm_phase1:       " << (int)ctrl_rx_qpsk.get_nstrm_phase1() << std::endl;
            std::cout << " mod_type_phase1:    " << (int)ctrl_rx_qpsk.get_mod_type_phase1() << std::endl;
            std::cout << " coding_rate_phase1: " << (int)ctrl_rx_qpsk.get_coding_rate_phase1() << std::endl;
            std::cout << " data_checksum:      0x" << std::hex << ctrl_rx_qpsk.get_data_checksum() << std::dec << std::endl;
            std::cout << " ctrl_checksum(10b): 0x" << std::hex << ctrl_rx_qpsk.get_ctrl_checksum() << std::dec << std::endl;
            std::cout << " extended (ignored in QPSK decode): 0x"
                      << std::hex << ctrl_rx_qpsk.get_extended() << std::dec << std::endl;
        }
    }

    // ------------------------------------------------------------------
    // Example 2: 16QAM demonstration
    //  - 128 bits total => includes extended => 256 coded bits => 64 16QAM symbols.
    //  - The 10-bit CRC is computed over the entire 128 bits.
    // ------------------------------------------------------------------
    {
        CTRL ctrl_tx_16qam(false /*debug*/);

        ctrl_tx_16qam.set_seq_number(9999);
        ctrl_tx_16qam.set_reserved(0x2);
        ctrl_tx_16qam.set_nstrm_phase1(0);
        ctrl_tx_16qam.set_mod_type_phase1(1);
        ctrl_tx_16qam.set_coding_rate_phase1(3);
        ctrl_tx_16qam.set_nstrm_phase2(1);
        ctrl_tx_16qam.set_mod_type_phase2(2);
        ctrl_tx_16qam.set_coding_rate_phase2(4);
        ctrl_tx_16qam.set_nstrm_phase3(0);
        ctrl_tx_16qam.set_mod_type_phase3(3);
        ctrl_tx_16qam.set_coding_rate_phase3(1);

        // Possibly set checksums and extended
        ctrl_tx_16qam.set_data_checksum(0xBEEF);
        ctrl_tx_16qam.set_extended(0xAEADDEADCAFECAFEULL);

        // Pack + modulate => 64 16QAM symbols
        std::vector<gr_complex> syms_16qam = ctrl_tx_16qam.pack_and_modulate_16qam();
        std::cout << "[16QAM] Generated " << syms_16qam.size() << " symbols." << std::endl;

        // Receive side
        CTRL ctrl_rx_16qam(false /*debug*/);
        std::vector<gr_complex> csi_16qam(syms_16qam.size(), gr_complex(1.0, 0.0));
        bool ok_16qam = ctrl_rx_16qam.demodulate_and_unpack_16qam(syms_16qam, csi_16qam);
        std::cout << "[16QAM] Demod/Unpack success? " << (ok_16qam ? "YES" : "NO") << std::endl;

        if(ok_16qam) {
            std::cout << " seq_number:         " << ctrl_rx_16qam.get_seq_number() << std::endl;
            std::cout << " reserved(4 bits):   0x" << std::hex << (int)ctrl_rx_16qam.get_reserved() << std::dec << std::endl;
            std::cout << " nstrm_phase1:       " << (int)ctrl_rx_16qam.get_nstrm_phase1() << std::endl;
            std::cout << " mod_type_phase1:    " << (int)ctrl_rx_16qam.get_mod_type_phase1() << std::endl;
            std::cout << " coding_rate_phase1: " << (int)ctrl_rx_16qam.get_coding_rate_phase1() << std::endl;
            std::cout << " data_checksum:      0x" << std::hex << ctrl_rx_16qam.get_data_checksum() << std::dec << std::endl;
            std::cout << " ctrl_checksum(10b): 0x" << std::hex << ctrl_rx_16qam.get_ctrl_checksum() << std::dec << std::endl;
            std::cout << " extended:           0x" << std::hex << ctrl_rx_16qam.get_extended() << std::dec << std::endl;
        }
    }

    // Example 3: Testing set_raw & get_raw for 16QAM
    {
        std::cout << "[Example 3] Testing get_raw/set_raw (16QAM)" << std::endl;
        CTRL ctrl_tx_16qam_test(false);
        ctrl_tx_16qam_test.set_seq_number(123);
        ctrl_tx_16qam_test.set_reserved(0xE);
        ctrl_tx_16qam_test.set_nstrm_phase1(1);
        ctrl_tx_16qam_test.set_mod_type_phase1(2);
        ctrl_tx_16qam_test.set_coding_rate_phase1(4);
        ctrl_tx_16qam_test.set_extended(0xDEADBEEFCAFEBABEULL);

        unsigned __int128 raw_val = ctrl_tx_16qam_test.get_raw();
        CTRL ctrl_rx_16qam_test(false);
        ctrl_rx_16qam_test.set_raw(raw_val);

        std::cout << "  TX seq_number:  " << ctrl_tx_16qam_test.get_seq_number() << std::endl;
        std::cout << "  RX seq_number:  " << ctrl_rx_16qam_test.get_seq_number() << std::endl;
        std::cout << "  TX extended:    0x" << std::hex << ctrl_tx_16qam_test.get_extended() << std::dec << std::endl;
        std::cout << "  RX extended:    0x" << std::hex << ctrl_rx_16qam_test.get_extended() << std::dec << std::endl;
    }

    return 0;
}

