^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_task_composer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Remove renaming and indexing (`#674 <https://github.com/tesseract-robotics/tesseract_planning/issues/674>`_)
* Add solver settings to fix state collision task (`#672 <https://github.com/tesseract-robotics/tesseract_planning/issues/672>`_)
* Update based on changes in boost plugin loaders latest release
* Add and leverage nested data storage to avoid key reuse issues (`#673 <https://github.com/tesseract-robotics/tesseract_planning/issues/673>`_)
  * Add and leverage nested data storage to avoid key reuse issues
  * Remove task name from keys in raster tasks
  * Abort when exception is thrown
* Add For Each Task (`#670 <https://github.com/tesseract-robotics/tesseract_planning/issues/670>`_)
* Contributors: Levi Armstrong, Tyler Marr

0.32.0 (2025-09-10)
-------------------
* Print a collision summary to status of failed contact check (`#543 <https://github.com/tesseract-robotics/tesseract_planning/issues/543>`_)
* Add trajopt collision coefficients to fix state collision profile (`#655 <https://github.com/tesseract-robotics/tesseract_planning/issues/655>`_)
* Improve memory allocations
* Added Profiles constructors for YAML conversions (`#652 <https://github.com/tesseract-robotics/tesseract_planning/issues/652>`_)
* Add trajopt joint configs for fix state collision task (`#637 <https://github.com/tesseract-robotics/tesseract_planning/issues/637>`_)
* Fix missing export in kinematic_limits_check_task.h
* Contributors: John Wason, Levi Armstrong, Samantha Smith, Tyler Marr

0.31.0 (2025-07-06)
-------------------
* Fix yaml extensions file name spelling
* Improve processYamlIncludeDirective peformance
* Clang formatting
* Correctly update input key rather than output key for raster task
* Add check to for missing velocity and acceleration data in kinematic limits check task
* Add missing contact_manager_config in FixStateCollisionProfile setialization
* Add constant tcp speed parameterization task
* Update time parameterization interface to leverage profiles to support cartesian time parameterization
* Port KinematicLimitsCheckTask from SNP Workshop
* Implement the option to format result as output for SimplePlanner (`#622 <https://github.com/tesseract-robotics/tesseract_planning/issues/622>`_)
  * Implement the option to format result as output for SimplePlanner
  * Fix format_result_as_input default for generateInterpolatedProgram
  * Fix format_result_as_input default for MinLengthTask
* Remove PluginLoader and ClassLoader from tesseract_common fwd.h
* Make sure serialized objects have friend struct tesseract_common::Serialization
* Contributors: Levi Armstrong, Roelof Oomen, Tyler Marr

0.30.0 (2025-04-23)
-------------------
* Move profile dictionary to tesseract_common
* Rename PlanProfile to MoveProfile
* Remove use of use_weighted_sum in trajopt config
* Update to leverage boost_plugin_loader
* Added class type to task composer dotgraph description
* Fix codecov CI
* Fix issues with how Collision Check Config was being used
* Update to changes with legacy trajopt leveraging trajopt_common::TrajOptCollisionConfig
* Contributors: Levi Armstrong, Michael Ripperger

0.29.1 (2025-03-21)
-------------------

0.29.0 (2025-03-20)
-------------------
* Simplify the use of poly types
* Add constructors to WaypointPoly to support State, Joint and Cartesian Interface class
* Update because of changes with AnyPoly
* Leverage inheritance over type erasure for instructions
* Remove no longer needed methods from the move instruction
* Leverage inheritance over type erasure for waypoints
* Leverage tesseract_collision CollisionEvaluatorType
* Update to leverage std::filesystem
* Update to clang-tidy-17 (`#591 <https://github.com/tesseract-robotics/tesseract_planning/issues/591>`_)
* Remove unimplemented method TaskComposerContext abort that takes a node
* Add python specific method to task composer graph and server
* Remove use of TaskComposerNodeInfo::UPtr and just use TaskComposerNodeInfo instead
* Fix missing include for use of shared_ptr
* Contributors: Joseph Schornak, Levi Armstrong

0.28.4 (2025-01-22)
-------------------
* Update task composer examples to leverage resource url
* Contributors: Levi Armstrong

0.28.3 (2025-01-21)
-------------------

0.28.2 (2025-01-21)
-------------------
* Fix cpack and add ruckig cpack generation and upload (`#585 <https://github.com/tesseract-robotics/tesseract_planning/issues/585>`_)
* Contributors: Levi Armstrong

0.28.1 (2025-01-16)
-------------------

0.28.0 (2025-01-16)
-------------------
* Fix cpack build and enable during CI pipeline
* Remove manip_info as option task input
* Fix motion planner task to support error branching
* Fix cpack name when components contain underscore
* Include task composer in .run-cpack, fix typos in CMakeLists (`#577 <https://github.com/tesseract-robotics/tesseract_planning/issues/577>`_)
* Fix MSVC compiler error in TaskComposerPluginFactory
* Leverage tesseract_common loadYamlFile and loadYamlString
* Modify composite instruction constructor
* Fix composite and move instruction equal operator and serialization (`#561 <https://github.com/tesseract-robotics/tesseract_planning/issues/561>`_)
* Changes required now that environment return const shared pointers for getKinematicGroup and getJointGroup
* Add Ruckig composite profile serialization
* Remove env_state from planning request
* Address some windows build issues (`#541 <https://github.com/tesseract-robotics/tesseract_planning/issues/541>`_)
* Modify the concept of profile overrides in Composite and Move Instruction
* Add boost serialization to profiles
* Add profile base class and update profile dictionary to not leverage template methods
* Contributors: John Wason, Levi Armstrong, Max DeSantis, Roelof Oomen

0.27.0 (2024-12-01)
-------------------
* Add FormatPlanningInputTask
* Contributors: Levi Armstrong

0.26.1 (2024-10-29)
-------------------
* Fix format as input task to handle both joint and state waypoints
* Contributors: Levi Armstrong

0.26.0 (2024-10-27)
-------------------
* Store dotgraph in TaskComposerLog
* Remove TesseractSupportResourceLocator
* Contributors: Levi Armstrong

0.25.1 (2024-09-29)
-------------------

0.25.0 (2024-09-29)
-------------------
* Rename Timer to Stopwatch
* Contributors: Levi Armstrong

0.24.2 (2024-08-19)
-------------------
* Add optional output to task which produce contact results
* Contributors: Levi Armstrong

0.24.1 (2024-08-16)
-------------------
* Add getRootNode to TaskComposerGraph and update TaskComposerNodeInfo to store root_uuid
* Contributors: Levi Armstrong

0.24.0 (2024-08-14)
-------------------
* Update TaskComposerNodeInfo to allow searching graph
* Fix RemapTask support for global remapping
* Add data storage to task composer node info
* Contributors: Levi Armstrong

0.23.6 (2024-08-06)
-------------------
* Make aborted nodes grey in dotgraph
* Fix task composer context serialization
* Fix Mac OS clang compiler warnings (`#496 <https://github.com/tesseract-robotics/tesseract_planning/issues/496>`_)
* Contributors: John Wason, Levi Armstrong

0.23.5 (2024-08-01)
-------------------
* Update so pipelines can have child graph tasks
* Fix Windows and Mac OS Github Actions (`#489 <https://github.com/tesseract-robotics/tesseract_planning/issues/489>`_)
* Contributors: John Wason, Levi Armstrong

0.23.4 (2024-07-29)
-------------------
* Fix task composer graph config loading
* Contributors: Levi Armstrong

0.23.3 (2024-07-28)
-------------------
* Cleanup boost serialization (`#490 <https://github.com/tesseract-robotics/tesseract_planning/issues/490>`_)
* Contributors: Levi Armstrong

0.23.2 (2024-07-25)
-------------------
* Make dotgraph label nojustify
* Fix task composer data storage constructors
* Contributors: Levi Armstrong

0.23.1 (2024-07-24)
-------------------

0.23.0 (2024-07-24)
-------------------
* Add name to data storage to hold pipeline name
* Improve dotgraph
* Fix discrete and continuous task info constructors
* Remove task_composer_problem.h
* Add check for expected keys to TaskComposerGraph
* Update task composer readme
* Do not export plugin libraries (`#474 <https://github.com/tesseract-robotics/tesseract_planning/issues/474>`_)
* Make disabled task dotgraph color yellow
* Environment should be stored as const in data storage
* fix applyCorrectionWorkflow definition
* Remove TaskComposerProblem and leverage TaskComposerDataStorage instead (`#469 <https://github.com/tesseract-robotics/tesseract_planning/issues/469>`_)
* Fixes for building on Ubuntu Noble
* Contributors: Levi Armstrong, Roelof Oomen

0.22.1 (2024-06-12)
-------------------
* Fix: Add cost to cost_infos instead of cnt_infos
* Contributors: Roelof Oomen

0.22.0 (2024-06-10)
-------------------
* Improve FixStateCollisionTask by also add a cost (distance) between new and original joint state
* Add task composer graph validation (`#463 <https://github.com/tesseract-robotics/tesseract_planning/issues/463>`_)
* Add HasDataStorageEntryTask and FormatAsResultTask with unit tests (`#462 <https://github.com/tesseract-robotics/tesseract_planning/issues/462>`_)
* Add include for Ubuntu Noble
* Add status_code to TaskComposerNodeInfo
* Add time parameterization interface (`#455 <https://github.com/tesseract-robotics/tesseract_planning/issues/455>`_)
* Add utility methods to task composer node info container
* Update to use forward declarations (`#449 <https://github.com/tesseract-robotics/tesseract_planning/issues/449>`_)
* Feat/more verbose planning failures (`#440 <https://github.com/tesseract-robotics/tesseract_planning/issues/440>`_)
* Adding Trajopt_Ifopt option to all examples (`#389 <https://github.com/tesseract-robotics/tesseract_planning/issues/389>`_)
* Fix error message for missing output keys
* Contributors: Levi Armstrong, Roelof, Roelof Oomen, Tyler Marr

0.21.7 (2024-02-03)
-------------------
* Add optional namespace field to task nodes (`#433 <https://github.com/tesseract-robotics/tesseract_planning/issues/433>`_)
* Contributors: Tyler Marr

0.21.6 (2023-12-21)
-------------------
* Add Mac OSX support (`#428 <https://github.com/tesseract-robotics/tesseract_planning/issues/428>`_)
* Contributors: John Wason

0.21.5 (2023-12-13)
-------------------
* Fix TaskComposerProblem serialization and equal operator
* Contributors: Levi Armstrong

0.21.4 (2023-11-21)
-------------------

0.21.3 (2023-11-20)
-------------------
* Update README.rst
  Description of Simple Motion Planner task fixed
* Contributors: Roelof

0.21.2 (2023-11-17)
-------------------
* Improve dynamic tasking support
* Contributors: Levi Armstrong

0.21.1 (2023-11-17)
-------------------
* Fix loss of first waypoint in upsample trajectory (`#416 <https://github.com/tesseract-robotics/tesseract_planning/issues/416>`_)
* Use taskflow subflow for graph execution to allow timing of execution
* Contributors: Levi Armstrong, Thomas Hettasch

0.21.0 (2023-11-10)
-------------------
* Fix clang-tidy errors
* Replace input_indexing and output_indexing with indexing
* Replace input_remapping and output_remapping with remapping
* Move TaskComposerProblem input to base class and change type to tesseract_common::AnyPoly
* remove results from TaskComposerNodeInfo
* Unused includes cleanup
* Contributors: Levi Armstrong, Roelof Oomen

0.20.1 (2023-10-02)
-------------------

0.20.0 (2023-09-29)
-------------------
* Remove AbortTask
* Add input instruction to planning problem
* Merge pull request `#370 <https://github.com/tesseract-robotics/tesseract_planning/issues/370>`_ from marip8/update/task-composer-factory-constructor
  Add new task composer plugin factory constructor
* Added unit test for new TaskComposerPluginFactory constructor
* Added constructor to task composer plugin factory to use task composer plugin config struct
* Rename TaskComposerInput to TaskComposerContext and simplify interfaces (`#379 <https://github.com/tesseract-robotics/tesseract_planning/issues/379>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.19.0 (2023-09-05)
-------------------
* Update to leverage cmake components
* Fix Raster and RasterOnly Tasks
* Add elapsed time for pipelines and include in dot graph
* Add conditional to subgraph in dot graph output
* Add input and output keys to dot graph
* Add Remap Task (`#351 <https://github.com/tesseract-robotics/tesseract_planning/issues/351>`_)
* Contributors: Levi Armstrong

0.18.4 (2023-07-07)
-------------------
* Move task composer elapse timing to base classes
* Contributors: Levi Armstrong

0.18.3 (2023-07-04)
-------------------
* Fix MotionPlannerTaskInfo serialization
* Contributors: Levi Armstrong

0.18.2 (2023-07-03)
-------------------
* Add clone method to TaskComposerProblem
* Contributors: Levi Armstrong

0.18.1 (2023-07-03)
-------------------
* Fix TaskComposerServer destruction
* Contributors: Levi Armstrong

0.18.0 (2023-06-30)
-------------------
* Update task_composer_plugins_no_trajopt_ifopt.yaml
* Restruct Raster yaml config to have same look as everything else
* Leverage AbortTask and make ErrorTask not abort
* Remove unused file
* Upgrade to TrajOpt 0.6.0
* Add task composer planning unit tests (`#341 <https://github.com/tesseract-robotics/tesseract_planning/issues/341>`_)
* Fixes for Python wrappers (`#329 <https://github.com/tesseract-robotics/tesseract_planning/issues/329>`_)
* Add TaskComposerServer unit tests
* Add task composer taskflow unit tests (`#339 <https://github.com/tesseract-robotics/tesseract_planning/issues/339>`_)
* Add TaskComposerPipeline and improve task composer code coverage (`#337 <https://github.com/tesseract-robotics/tesseract_planning/issues/337>`_)
* Added trajectory logger printout to trajectory checker (`#338 <https://github.com/tesseract-robotics/tesseract_planning/issues/338>`_)
* Added an extra needed #include for 22.04 builds (`#332 <https://github.com/tesseract-robotics/tesseract_planning/issues/332>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Restructure tesseract_task_composer like other plugin based packages
* Add PlanningTaskComposerProblem
* Added ability to colorize dotgraphs with planning results (`#327 <https://github.com/tesseract-robotics/tesseract_planning/issues/327>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Contributors: John Wason, Levi Armstrong, Tyler Marr

0.17.0 (2023-06-06)
-------------------
* Fix Key Naming Scheme in Raster Motion Task  (`#324 <https://github.com/tesseract-robotics/tesseract_planning/issues/324>`_)
  @marrts Great find and thanks for the fix.
* Fix task composer cmake plugins variable
* Update task nodes to on failure store input in output location to better support error branching
* Fix some typos
* Contributors: Levi Armstrong, Roelof Oomen, Tyler Marr

0.16.3 (2023-05-03)
-------------------
* Fix FormatAsInputTask to store results
* Contributors: Levi Armstrong

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-09)
-------------------
* Add FormatAsInputTask
* Update to leverage CollisionCheckProgramType in collision config
* Fix clang-tidy errors
* Update to support new contact results class (`#297 <https://github.com/tesseract-robotics/tesseract_planning/issues/297>`_)
* Fixing OMPL freespace example and a typo (`#299 <https://github.com/tesseract-robotics/tesseract_planning/issues/299>`_)
  * Fix freespace OMPL example (was hybrid)
  * Fix typo in iterative_spline_parameterization_profile file name
* Add TOTG Node Info class
* Contributors: Levi Armstrong, Roelof

0.15.5 (2023-03-22)
-------------------
* Fix TOTG assignData
* Add fix_state_collision clone method and serialize contact results
* Build fixes for Focal/Foxy and Jammy/Humble
* Contributors: Levi Armstrong, Roelof Oomen

0.15.4 (2023-03-16)
-------------------

0.15.3 (2023-03-15)
-------------------

0.15.2 (2023-03-14)
-------------------
* Clean up task composer serialization
* Contributors: Levi Armstrong

0.15.1 (2023-03-09)
-------------------
* Add method for retrieving task from TaskComposerServer
* Use try catch in TaskComposerTask run because exceptions are not propagated in multi threaded runs.
* Update fix state bounds task to ignore cartesian waypoint types
* Contributors: Levi Armstrong

0.15.0 (2023-03-03)
-------------------
* Update task composer to leverage plugins (`#282 <https://github.com/tesseract-robotics/tesseract_planning/issues/282>`_)
* Use templates for raster task to reduce code duplications (`#279 <https://github.com/tesseract-robotics/tesseract_planning/issues/279>`_)
* Add descartes no post check motion pipeline task
* clean up update end state task
* Fix descartes global motion pipeline task
* Merge pull request `#269 <https://github.com/tesseract-robotics/tesseract_planning/issues/269>`_ from marip8/update/time-param-org
  Added optional builds of time parameterization implementations
* Created separate targets for each time parameterization implementation
* Updated task composer package
* Remove composite start instruction
* Add uuid and parent_uuid to InstructionPoly (`#261 <https://github.com/tesseract-robotics/tesseract_planning/issues/261>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.14.0 (2022-10-23)
-------------------
* Add ompl to default tasks utility function
* Fix trajopt ifopt task name
* Add environment to TaskComposerNodeInfo
* Add method to TaskComposerDataStorage to get copy of all data
* Update TaskComposerNodeInfo contructor to take node type
* Remove tesseract_process_managers package
* Remove references to tesseract process managers package
* Fixes for Ubuntu 22.04 (boost and mutex)
* Add tesseract_task_composer package to replace tesseract_process_managers
* Fix clang tidy errors
* Rename TransitionMuxTask to UpdateStartAndEndStateTask
* Add TaskComposerServer
* Add task composer problem
* Remove clone method from TaskComposerNode
* Finish migrating unit tests
* Break up task to avoid configuration parameters
* Update task to require returning TaskComposerNodeInfo
* Fix raster global tasks
* Cleanup task composer examples
* Add remaining raster tasks
* Fix rebase conflicts
* Fix clang-tidy errors
* Store input and output keys in TaskComposerNode
* Add clone method to TaskComposerNode
* Add TaskComposerPluginFactory
* Cleanup TaskComposerFuture
* Move contents of taskflow_utils.h into taskflow executor
* Add reset capability to TaskComposerInput
* Remove executor from TaskComposerInput
* Add TaskComposerExecutor and TaskComposerFuture
* Add inbound edges to TaskComposerNode
* Fix dot graph generation
* Rename SeedMinLengthTask to MinLengthTask
* Fix task composer seed_min_length_task
* Move the interpolate functions into its own file and add StartTask need for raster task
* Add dump function to create dot graph
* Add raster motion task
* Update TaskComposerGraph to use task uuid as key for nodes
* Add TaskComposerTask class
* Add motion planning pipelines to tesseract_task_composer
* Add format_result_as_input to PlannerRequest
* Fix cmake files
* Add conditional task type
* Add done and error task
* Rename TaskComposerPipeline to TaskComposerGraph
* Add transition mux task
* Add equal operators to task composer tasks
* Remove use of tesseract_common::StatusCode
* Add task composer package
* Contributors: Levi Armstrong, Roelof Oomen
