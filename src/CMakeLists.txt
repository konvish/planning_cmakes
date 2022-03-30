# MRPT
find_package(MRPT REQUIRED
       COMPONENTS bayes random poses system io graphs nav maps gui
       OPTIONAL_COMPONENTS  vision)

# OMPL
link_directories(${OMPL_LIBRARIES})
include_directories(${OMPL_INCLUDE_DIRS})

find_package(ompl REQUIRED
       COMPONENTS base geometric
)

find_package(Boost 1.70 REQUIRED COMPONENTS system filesystem program_options)
find_package(Eigen3 REQUIRED)

# MRPT编译方法
macro(add_mrpt_demo demo_name)
add_executable(${ARGV})
target_link_libraries(${demo_name} ${MRPT_LIBRARIES})
endmacro(add_mrpt_demo)

#OMPL编译方法
macro(add_ompl_demo demo_name)
    add_executable(${ARGV})
    target_link_libraries(${demo_name}
         ompl
         ${Boost_FILESYSTEM_LIBRARY}
         ${Boost_SYSTEM_LIBRARY}
         ${Boost_PROGRAM_OPTIONS_LIBRARY}
    )
endmacro(add_ompl_demo)


add_executable(test test.cpp)

add_mrpt_demo(bayes_rejection_sampling BayesRejectionSampling.cpp)
add_mrpt_demo(bayes_resampling BayesResampling.cpp)
add_mrpt_demo(graphs_astar graphsAstar.cpp)
add_mrpt_demo(nav_rrt_planning NavRRTPlanning.cpp)

add_ompl_demo(cforest_circle_grid_benchmark CForestCircleGridBenchmark.cpp)
add_ompl_demo(diagonal Diagonal.cpp)
add_ompl_demo(geomertic_car_planning GeometricCarPlanning.cpp)
add_ompl_demo(hybrid_system_planning HybridStstemPlanning.cpp)
add_ompl_demo(hybridcube_benchmark HyBridcubeBenchmark.cpp)
add_ompl_demo(kinematic_chain_benchmark KinematicChainBenchmark.cpp)
add_ompl_demo(LTL_with_traingulation LTLWithTraingulation.cpp)
add_ompl_demo(openDE_rigid_body_planning OpenDERigidBodyPlanning.cpp)
add_ompl_demo(optimal_planning OptimalPlanning.cpp)
add_ompl_demo(planner_data PlannerData.cpp)
add_ompl_demo(planner_progress_properties PlannerProgressProperties.cpp)
add_ompl_demo(point2d_planning Point2DPlanning.cpp)
add_ompl_demo(rigid_body_planning RigidBodyPlanning.cpp)
add_ompl_demo(rigid_body_planning_with_controls RigidBodyPlanningWithControls.cpp)
add_ompl_demo(rigid_body_planning_with_ik RigidBodyPlanningWithIK.cpp)
add_ompl_demo(rigid_body_planning_with_integration_and_controls RigidBodyPlanningWithIntegrationAndControls.cpp)
add_ompl_demo(rigid_body_planning_with_odesolver_and_controls RigidBodyPlanningWithODESolverAndControls.cpp)
add_ompl_demo(state_sampling StateSampling.cpp)
add_ompl_demo(thunder_lightning ThunderLightning.cpp)
add_ompl_demo(triangulation_demo TriangulationDemo.cpp)

aux_source_directory(Koules Koules_SRC)
add_ompl_demo(koules ${Koules_SRC})

add_ompl_demo(demo_ConstrainedPlanningSphere constraint/ConstrainedPlanningSphere.cpp)
add_ompl_demo(demo_ConstrainedPlanningTorus constraint/ConstrainedPlanningTorus.cpp)
add_ompl_demo(demo_ConstrainedPlanningImplicitChain constraint/ConstrainedPlanningImplicitChain.cpp)
add_ompl_demo(demo_ConstrainedPlanningImplicitParallel constraint/ConstrainedPlanningImplicitParallel.cpp)
add_ompl_demo(demo_ConstrainedPlanningKinematicChain constraint/ConstrainedPlanningKinematicChain.cpp)

add_ompl_demo(demo_VectorFieldConservative VFRRT/VectorFieldConservative.cpp)
add_ompl_demo(demo_VectorFieldNonconservative VFRRT/VectorFieldNonconservative.cpp)

add_ompl_demo(demo_QuotientSpacePlanningRigidBody2D quotientspace/QuotientSpacePlanningRigidBody2D.cpp)
add_ompl_demo(demo_QuotientSpacePlanningRigidBody3D quotientspace/QuotientSpacePlanningRigidBody3D.cpp)
add_ompl_demo(demo_QuotientSpacePlanningKinematicChain quotientspace/QuotientSpacePlanningKinematicChain.cpp)
add_ompl_demo(demo_QuotientSpacePlanningHyperCube quotientspace/QuotientSpacePlanningHyperCube.cpp)
add_ompl_demo(demo_QuotientSpacePlanningHyperCubeBenchmark quotientspace/QuotientSpacePlanningHyperCubeBenchmark.cpp)

find_package(yaml-cpp)
set_package_properties(yaml-cpp PROPERTIES
    URL "https://github.com/jbeder/yaml-cpp"
    PURPOSE "Used for parsing input data for the PlanarManipulator demo.")
if (yaml-cpp_FOUND)
    add_ompl_demo(demo_PlanarManipulator
        PlanarManipulator/PlanarManipulatorDemo.cpp
        PlanarManipulator/PlanarManipulator.cpp
        PlanarManipulator/PolyWorld.cpp
    )
    target_include_directories(demo_PlanarManipulator PUBLIC ${YAMLCPP_INCLUDE_DIRS})
    target_link_libraries(demo_PlanarManipulator ${YAMLCPP_LIBRARIES})
endif()

if (OMPL_EXTENSION_ODE)
    add_ompl_demo(demo_OpenDERigidBodyPlanning OpenDERigidBodyPlanning.cpp)
endif()

if (OMPL_EXTENSION_TRIANGLE)
    add_ompl_demo(demo_TriangulationDemo TriangulationDemo.cpp)
    add_ompl_demo(demo_LTLWithTriangulation LTLWithTriangulation)
endif()