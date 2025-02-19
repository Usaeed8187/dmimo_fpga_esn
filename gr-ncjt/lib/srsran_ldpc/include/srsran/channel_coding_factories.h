/*
 *
 * Copyright 2021-2025 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#pragma once

#include "srsran/crc_calculator.h"
#include "srsran/ldpc/ldpc_decoder.h"
#include "srsran/ldpc/ldpc_encoder.h"
#include "srsran/ldpc/ldpc_rate_dematcher.h"
#include "srsran/ldpc/ldpc_rate_matcher.h"
#include "srsran/ldpc/ldpc_segmenter_rx.h"
#include "srsran/ldpc/ldpc_segmenter_tx.h"
#include <memory>

namespace srsran {

class crc_calculator_factory
{
public:
  virtual ~crc_calculator_factory()                                       = default;
  virtual std::unique_ptr<crc_calculator> create(crc_generator_poly poly) = 0;
};

std::shared_ptr<crc_calculator_factory> create_crc_calculator_factory_sw(const std::string& type);

class ldpc_decoder_factory
{
public:
  virtual ~ldpc_decoder_factory()                = default;
  virtual std::unique_ptr<ldpc_decoder> create() = 0;
};

std::shared_ptr<ldpc_decoder_factory> create_ldpc_decoder_factory_sw(const std::string& dec_type);

class ldpc_encoder_factory
{
public:
  virtual ~ldpc_encoder_factory()                = default;
  virtual std::unique_ptr<ldpc_encoder> create() = 0;
};

std::shared_ptr<ldpc_encoder_factory> create_ldpc_encoder_factory_sw(const std::string& enc_type);

class ldpc_rate_dematcher_factory
{
public:
  virtual ~ldpc_rate_dematcher_factory()                = default;
  virtual std::unique_ptr<ldpc_rate_dematcher> create() = 0;
};

std::shared_ptr<ldpc_rate_dematcher_factory> create_ldpc_rate_dematcher_factory_sw(const std::string& dematcher_type);

class ldpc_rate_matcher_factory
{
public:
  virtual ~ldpc_rate_matcher_factory()                = default;
  virtual std::unique_ptr<ldpc_rate_matcher> create() = 0;
};

std::shared_ptr<ldpc_rate_matcher_factory> create_ldpc_rate_matcher_factory_sw();

class ldpc_segmenter_rx_factory
{
public:
  virtual ~ldpc_segmenter_rx_factory()                = default;
  virtual std::unique_ptr<ldpc_segmenter_rx> create() = 0;
};

std::shared_ptr<ldpc_segmenter_rx_factory> create_ldpc_segmenter_rx_factory_sw();

class ldpc_segmenter_tx_factory
{
public:
  virtual ~ldpc_segmenter_tx_factory()                = default;
  virtual std::unique_ptr<ldpc_segmenter_tx> create() = 0;
};

std::shared_ptr<ldpc_segmenter_tx_factory>
create_ldpc_segmenter_tx_factory_sw(std::shared_ptr<crc_calculator_factory> crc_factory);




} // namespace srsran
