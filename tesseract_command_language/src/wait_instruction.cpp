/**
 * @file wait_instruction.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date November 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/wait_instruction.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
WaitInstruction::WaitInstruction(double time) : wait_time_(time) {}
WaitInstruction::WaitInstruction(WaitInstructionType type, int io) : wait_type_(type), wait_io_(io)
{
  if (wait_type_ == WaitInstructionType::TIME)
    throw std::runtime_error("WaitInstruction: Invalid type 'WaitInstructionType::TIME' for constructor");
}

const boost::uuids::uuid& WaitInstruction::getUUID() const { return uuid_; }
void WaitInstruction::setUUID(const boost::uuids::uuid& uuid)
{
  if (uuid.is_nil())
    throw std::runtime_error("WaitInstruction, tried to set uuid to null!");

  uuid_ = uuid;
}
void WaitInstruction::regenerateUUID() { uuid_ = boost::uuids::random_generator()(); }

const boost::uuids::uuid& WaitInstruction::getParentUUID() const { return parent_uuid_; }
void WaitInstruction::setParentUUID(const boost::uuids::uuid& uuid) { parent_uuid_ = uuid; }

const std::string& WaitInstruction::getDescription() const { return description_; }

void WaitInstruction::setDescription(const std::string& description) { description_ = description; }

void WaitInstruction::print(const std::string& prefix) const  // NOLINT
{
  std::cout << prefix + "Wait Instruction, Wait Type: " << static_cast<int>(wait_type_);
  std::cout << ", Description: " << getDescription() << "\n";
}

std::unique_ptr<InstructionInterface> WaitInstruction::clone() const
{
  return std::make_unique<WaitInstruction>(*this);
}

WaitInstructionType WaitInstruction::getWaitType() const { return wait_type_; }

void WaitInstruction::setWaitType(WaitInstructionType type) { wait_type_ = type; }

double WaitInstruction::getWaitTime() const { return wait_time_; }

void WaitInstruction::setWaitTime(double time) { wait_time_ = time; }

int WaitInstruction::getWaitIO() const { return wait_io_; }

void WaitInstruction::setWaitIO(int io) { wait_io_ = io; }

bool WaitInstruction::equals(const InstructionInterface& other) const
{
  const auto* rhs = dynamic_cast<const WaitInstruction*>(&other);
  if (rhs == nullptr)
    return false;

  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(wait_time_, rhs->wait_time_, max_diff);
  equal &= (wait_type_ == rhs->wait_type_);
  equal &= (wait_io_ == rhs->wait_io_);

  return equal;
}

template <class Archive>
void WaitInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(InstructionInterface);
  ar& boost::serialization::make_nvp("uuid", uuid_);
  ar& boost::serialization::make_nvp("parent_uuid", parent_uuid_);
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("wait_type", wait_type_);
  ar& boost::serialization::make_nvp("wait_time", wait_time_);
  ar& boost::serialization::make_nvp("wait_io", wait_io_);
}
}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::WaitInstruction)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::WaitInstruction)
