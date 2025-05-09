/**
 * @file task_composer_server.h
 * @brief A task server
 *
 * @author Levi Armstrong
 * @date August 27, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_SERVER_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_SERVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <filesystem>

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
class TaskComposerExecutor;
class TaskComposerNode;
class TaskComposerFuture;
class TaskComposerDataStorage;
class TaskComposerPluginFactory;
struct TaskComposerProblem;

class TaskComposerServer
{
public:
  using Ptr = std::shared_ptr<TaskComposerServer>;
  using ConstPtr = std::shared_ptr<const TaskComposerServer>;
  using UPtr = std::unique_ptr<TaskComposerServer>;
  using ConstUPtr = std::unique_ptr<const TaskComposerServer>;

  TaskComposerServer();

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  void loadConfig(const YAML::Node& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  void loadConfig(const std::filesystem::path& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  void loadConfig(const std::string& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Add a executors (thread pool)
   * @param executor The executor to add
   */
  void addExecutor(const std::shared_ptr<TaskComposerExecutor>& executor);

  /**
   * @brief Get an executor by name
   * @param name The name of the executor
   * @return The exector, if not found nullptr is returned
   */
  std::shared_ptr<TaskComposerExecutor> getExecutor(const std::string& name);

  /**
   * @brief Check if executors (thread pool) exists with the provided name
   * @param name The name to search
   * @return True if it exists, otherwise false
   */
  bool hasExecutor(const std::string& name) const;

  /**
   * @brief Get the available executors (thread pool) names
   * @return A vector of names
   */
  std::vector<std::string> getAvailableExecutors() const;

  /**
   * @brief Add a task
   * @param task The task to add
   */
  void addTask(std::unique_ptr<TaskComposerNode> task);

  /**
   * @brief Add a task
   * @warning This is only available for python buindings and should not be used in c++
   * @param task The task to add
   */
  void addTaskPython(std::shared_ptr<TaskComposerNode> task);

  /**
   * @brief Get a task
   * @param name The the name of task to retrieve
   */
  const TaskComposerNode& getTask(const std::string& name);

  /**
   * @brief Check if task exists with the provided name
   * @param name The name to search
   * @return True if it exists, otherwise false
   */
  bool hasTask(const std::string& name) const;

  /**
   * @brief Get the available task names
   * @return A vector of names
   */
  std::vector<std::string> getAvailableTasks() const;

  /**
   * @brief Execute the provided task graph
   * @param task_name The task name to run
   * @param data_storage The data storage
   * @param dotgraph Indicate if dotgraph should be generated
   * @param excutor_name The name of the executor to use
   * @return The future associated with execution
   */
  std::unique_ptr<TaskComposerFuture> run(const std::string& task_name,
                                          std::shared_ptr<TaskComposerDataStorage> data_storage,
                                          bool dotgraph,
                                          const std::string& executor_name);

  /**
   * @brief Execute the provided node
   * @details It will call one of the methods below based on the node type
   * @param node The node to execute
   * @param data_storage The data storage
   * @param dotgraph Indicate if dotgraph should be generated
   * @param excutor_name The name of the executor to use
   * @return The future associated with execution
   */
  std::unique_ptr<TaskComposerFuture> run(const TaskComposerNode& node,
                                          std::shared_ptr<TaskComposerDataStorage> data_storage,
                                          bool dotgraph,
                                          const std::string& executor_name);

  /** @brief Queries the number of workers (example: number of threads) */
  long getWorkerCount(const std::string& name) const;

  /** @brief Queries the number of running tasks at the time of this call */
  long getTaskCount(const std::string& name) const;

protected:
  std::shared_ptr<TaskComposerPluginFactory> plugin_factory_;
  std::unordered_map<std::string, std::shared_ptr<TaskComposerExecutor>> executors_;
  std::unordered_map<std::string, std::shared_ptr<TaskComposerNode>> tasks_;

  void loadPlugins();
};
}  // namespace tesseract_planning

#endif  // TASK_COMPOSER_SERVER_H
