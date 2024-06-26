/**
 * @file types.h
 * @brief Tesseract descartes types
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_TYPES_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <descartes_light/core/edge_evaluator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_vertex_evaluator.h>
#include <tesseract_motion_planners/descartes/descartes_problem.h>

namespace tesseract_planning
{
/**
 * @brief This is used for tesseract descartes samplers that filters out invalid solutions.
 *
 * Example: This would be used to filter out solution outside of custom joint limits.
 *
 */
template <typename FloatType>
using DescartesVertexEvaluatorAllocatorFn =
    std::function<DescartesVertexEvaluator::Ptr(const DescartesProblem<FloatType>&)>;

/**
 * @brief This is used to create edge evaluator within tesseract, to allow thread safe creation of descartes edge
 * evaluators
 */
template <typename FloatType>
using DescartesEdgeEvaluatorAllocatorFn =
    std::function<typename descartes_light::EdgeEvaluator<FloatType>::Ptr(const DescartesProblem<FloatType>&)>;

/**
 * @brief Creates a state evaluator
 */
template <typename FloatType>
using DescartesStateEvaluatorAllocatorFn =
    std::function<typename descartes_light::StateEvaluator<FloatType>::Ptr(const DescartesProblem<FloatType>&)>;

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_TYPES_H
