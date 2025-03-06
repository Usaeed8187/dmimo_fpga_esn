#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <iostream>

void
print_tree(const uhd::fs_path &path, uhd::property_tree::sptr tree)
{
    std::cout << path << std::endl;
    BOOST_FOREACH(const std::string &name, tree->list(path))
                {
                    print_tree(path / name, tree);
                }
}

int UHD_SAFE_MAIN(int argc, char *argv[])
{
    uhd::set_thread_priority_safe();

    std::string device_args_1("addr=192.168.60.2");
    std::cout << std::endl;
    std::cout << boost::format("Configuring the USRP device with: %s...") % device_args_1 << std::endl;
    uhd::usrp::multi_usrp::sptr n321 = uhd::usrp::multi_usrp::make(device_args_1);

    // print_tree("/", n321->get_device()->get_tree());

    n321->set_tx_lo_source("external", "lo1", 0); 	// channel 0 txlo <- lo1
    n321->set_tx_lo_source("external", "lo1", 1);	// channel 1 txlo <- lo1
    n321->set_rx_lo_source("external", "lo1", 0);	// channel 0 rxlo <- lo1
    n321->set_rx_lo_source("external", "lo1", 1);	// channel 1 rxlo <- lo1
    n321->set_tx_lo_export_enabled(true, "lo1", 0);
    n321->set_rx_lo_export_enabled(true, "lo1", 0);
    
    // Enable Tx/Rx LO output ports (OUT0 ~ OUT3)
    n321->get_device()->get_tree()->access<bool>(
        "/blocks/0/Radio#0/dboard/tx_frontends/0/los/lo1/lo_distribution/LO_OUT_0/export").set(false);
    n321->get_device()->get_tree()->access<bool>(
        "/blocks/0/Radio#0/dboard/tx_frontends/0/los/lo1/lo_distribution/LO_OUT_1/export").set(true);
    n321->get_device()->get_tree()->access<bool>(
        "/blocks/0/Radio#0/dboard/tx_frontends/0/los/lo1/lo_distribution/LO_OUT_2/export").set(true);
    n321->get_device()->get_tree()->access<bool>(
        "/blocks/0/Radio#0/dboard/tx_frontends/0/los/lo1/lo_distribution/LO_OUT_3/export").set(false);
    n321->get_device()->get_tree()->access<bool>(
        "/blocks/0/Radio#0/dboard/rx_frontends/0/los/lo1/lo_distribution/LO_OUT_0/export").set(true);
    n321->get_device()->get_tree()->access<bool>(
        "/blocks/0/Radio#0/dboard/rx_frontends/0/los/lo1/lo_distribution/LO_OUT_1/export").set(false);
    n321->get_device()->get_tree()->access<bool>(
        "/blocks/0/Radio#0/dboard/rx_frontends/0/los/lo1/lo_distribution/LO_OUT_2/export").set(true);
    n321->get_device()->get_tree()->access<bool>(
        "/blocks/0/Radio#0/dboard/rx_frontends/0/los/lo1/lo_distribution/LO_OUT_3/export").set(false);

    return EXIT_SUCCESS;
}


