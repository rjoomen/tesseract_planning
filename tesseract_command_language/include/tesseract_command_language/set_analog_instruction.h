/**
 * @file set_analog_instruction.h
 * @brief Set Analog Instruction
 *
 * @author Levi Armstrong
 * @date March 23, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COMMAND_LANGUAGE_SET_ANALOG_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_SET_ANALOG_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>

namespace tesseract_planning
{
class SetAnalogInstruction final : public InstructionInterface
{
public:
  SetAnalogInstruction() = default;  // Required for boost serialization do not use
  SetAnalogInstruction(std::string key, int index, double value);

  // Instruction
  const boost::uuids::uuid& getUUID() const override final;
  void setUUID(const boost::uuids::uuid& uuid) override final;
  void regenerateUUID() override final;

  const boost::uuids::uuid& getParentUUID() const override final;
  void setParentUUID(const boost::uuids::uuid& uuid) override final;

  const std::string& getDescription() const override final;

  void setDescription(const std::string& description) override final;

  void print(const std::string& prefix = "") const override final;  // NOLINT

  std::unique_ptr<InstructionInterface> clone() const override final;

  // SetAnalogInstruction

  /** @brief Get the analog key */
  std::string getKey() const;

  /** @brief Get the analog index */
  int getIndex() const;

  /** @brief Get the analgo value */
  double getValue() const;

private:
  /** @brief The instructions UUID */
  boost::uuids::uuid uuid_{};
  /** @brief The parent UUID if created from createChild */
  boost::uuids::uuid parent_uuid_{};
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Set Analog Instruction" };
  /** @brief The key is used to identify which type of analog to set */
  std::string key_;
  /** @brief The analog index to set */
  int index_{ 0 };
  /** @brief The analog value */
  double value_{ 0 };

  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  bool equals(const InstructionInterface& other) const override final;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::SetAnalogInstruction)
BOOST_CLASS_TRACKING(tesseract_planning::SetAnalogInstruction, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_SET_ANALOG_INSTRUCTION_H
