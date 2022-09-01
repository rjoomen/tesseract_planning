/**
 * @file raster_only_global_motion_task.h
 * @brief Plans raster paths
 *
 * @author Matthew Powelson
 * @date July 15, 2020
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_ONLY_GLOBAL_PIPELINE_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_ONLY_GLOBAL_PIPELINE_TASK_H

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_common/any.h>

namespace tesseract_planning
{
/**
 * @brief The RasterGlobalPipelineTask class
 * @details The required format is below.
 *
 * Composite
 * {
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 * }
 */
class RasterOnlyGlobalPipelineTask : public TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<RasterOnlyGlobalPipelineTask>;
  using ConstPtr = std::shared_ptr<const RasterOnlyGlobalPipelineTask>;
  using UPtr = std::unique_ptr<RasterOnlyGlobalPipelineTask>;
  using ConstUPtr = std::unique_ptr<const RasterOnlyGlobalPipelineTask>;

  RasterOnlyGlobalPipelineTask() = default;  // Required for serialization
  RasterOnlyGlobalPipelineTask(std::string input_key,
                               std::string output_key,
                               bool cartesian_transition = false,
                               std::string name = "RasterOnlyGlobalPipelineTask");
  ~RasterOnlyGlobalPipelineTask() override = default;
  RasterOnlyGlobalPipelineTask(const RasterOnlyGlobalPipelineTask&) = delete;
  RasterOnlyGlobalPipelineTask& operator=(const RasterOnlyGlobalPipelineTask&) = delete;
  RasterOnlyGlobalPipelineTask(RasterOnlyGlobalPipelineTask&&) = delete;
  RasterOnlyGlobalPipelineTask& operator=(RasterOnlyGlobalPipelineTask&&) = delete;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const RasterOnlyGlobalPipelineTask& rhs) const;
  bool operator!=(const RasterOnlyGlobalPipelineTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  bool cartesian_transition_{ false };

  static void checkTaskInput(const tesseract_common::Any& input);
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterOnlyGlobalPipelineTask, "RasterOnlyGlobalPipelineTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_ONLY_GLOBAL_PIPELINE_TASK_H
