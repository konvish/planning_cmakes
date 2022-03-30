#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/nav.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::maps;
using namespace std;

string MRPT_EXAMPLES_BASE_DIRECTOTY("/usr");

string mySimpleMap(MRPT_EXAMPLES_BASE_DIRECTOTY + string("/share/mrpt/datasets/malaga-cs-fac-building.simplemap.gz"));
string MyCfgFileName(MRPT_EXAMPLES_BASE_DIRECTOTY + string("/share/mrpt/config_files/navigation-ptgs/ptrrt_config_example1.ini"));

void TestRRT1()
{
    mrpt::random::Randomize();
    CSimpleMap simpleMap;
    ASSERT_FILE_EXISTS_(mySimpleMap);
    cout << "Loading map...";
    {
        mrpt::io::CFileGZInputStream f(mySimpleMap);
        auto arch = mrpt::serialization::archiveFrom(f);
        arch >> simpleMap;
    }
    cout << "Done! Number of sensory frames: " << simpleMap.size() << endl;

    mrpt::nav::PlannerRRT_SE2_TPS planner;
    planner.loadConfig(mrpt::config::CConfigFile(MyCfgFileName));

    planner.params.maxLength = 2.0;
    planner.params.minDistanceBetweenNewNodes = 0.10;
    planner.params.minAngBetweenNewNodes = mrpt::DEG2RAD(20);
    planner.params.goalBias = 0.05;

    planner.params.save_3d_log_freq = 0;

    planner.end_criteria.acceptedDistToTarget = 0.25;
    planner.end_criteria.acceptedAngToTarget = 180.0_deg;
    planner.end_criteria.maxComputationTime = 15.0;
    planner.end_criteria.minComputationTime = 1.0;

    planner.initialize();

    PlannerRRT_SE2_TPS::TPlannerResult planner_result;
    PlannerRRT_SE2_TPS::TPlannerInput planner_input;

    planner_input.start_pose = mrpt::math::TPose2D(0, 0, 0);
    planner_input.goal_pose = mrpt::math::TPose2D(-20, -30, 0);

    planner_input.obstacles_points.loadFromSimpleMap(simpleMap);
    const auto bbox = planner_input.obstacles_points.boundingBox();

    planner_input.world_bbox_min = mrpt::math::TPoint2D(bbox.min.x, bbox.min.y);
    planner_input.world_bbox_max = mrpt::math::TPoint2D(bbox.max.x, bbox.max.y);

#if MRPT_HAS_WXWIDGETS
    mrpt::gui::CDisplayWindow3D win("Result", 1024, 800);
    while (win.isOpen())
#else
    for (size_t i = 0; i < 1; i++)
#endif
    {
        bool refine_solution = false;
        if (!refine_solution)
            planner_result = PlannerRRT_SE2_TPS::TPlannerResult();

        planner.solve(planner_input, planner_result);
        cout << "Found goal_distance: " << planner_result.goal_distance << endl;
        cout << "Found path_cost: " << planner_result.path_cost << endl;
        cout << "Acceptable goal nodes: " << planner_result.acceptable_goal_node_ids.size() << endl;

#if MRPT_HAS_WXWIDGETS
        mrpt::opengl::COpenGLScene::Ptr &scene = win.get3DSceneAndLock();
        scene->clear();
        PlannerRRT_SE2_TPS::TRenderPlannedPathOptions render_opts;
        render_opts.highlight_path_to_node_id = planner_result.best_goal_node_id;

        planner.renderMoveTree(*scene, planner_input, planner_result, render_opts);
        win.unlockAccess3DScene();
        win.repaint();
        win.waitForKey();
#endif
    }
}

void TestRRTstar()
{
    mrpt::random::Randomize();
    CSimpleMap simpleMap;
    ASSERT_FILE_EXISTS_(mySimpleMap);
    cout << "Loading map...";
    {
        mrpt::io::CFileGZInputStream f(mySimpleMap);
        auto arch = mrpt::serialization::archiveFrom(f);
        arch >> simpleMap;
    }
    cout << "Done! Number of sensory frames: " << simpleMap.size() << endl;

    mrpt::nav::PlannerRRT_SE2_TPS planner;
    planner.loadConfig(mrpt::config::CConfigFile(MyCfgFileName));

    planner.params.maxLength = 2.0;
    planner.params.minDistanceBetweenNewNodes = 0.10;
    planner.params.minAngBetweenNewNodes = mrpt::DEG2RAD(20);
    planner.params.goalBias = 0.05;

    planner.params.save_3d_log_freq = 0;

    planner.end_criteria.acceptedDistToTarget = 0.25;
    planner.end_criteria.acceptedAngToTarget = 180.0_deg;
    planner.end_criteria.maxComputationTime = 15.0;
    planner.end_criteria.minComputationTime = 1.0;

    planner.initialize();


    PlannerRRT_SE2_TPS::TPlannerResult planner_result;
    PlannerRRT_SE2_TPS::TPlannerInput planner_input;

    planner_input.start_pose = mrpt::math::TPose2D(0, 0, 0);
    planner_input.goal_pose = mrpt::math::TPose2D(-20, -30, 0);

    planner_input.obstacles_points.loadFromSimpleMap(simpleMap);
    const auto bbox = planner_input.obstacles_points.boundingBox();

    planner_input.world_bbox_min = mrpt::math::TPoint2D(bbox.min.x, bbox.min.y);
    planner_input.world_bbox_max = mrpt::math::TPoint2D(bbox.max.x, bbox.max.y);

#if MRPT_HAS_WXWIDGETS
    mrpt::gui::CDisplayWindow3D win("Result", 1024, 800);
    while (win.isOpen())
#else
    for (size_t i = 0; i < 1; i++)
#endif
    {
        bool refine_solution = false;
        if (!refine_solution)
            planner_result = PlannerRRT_SE2_TPS::TPlannerResult();

        planner.solve(planner_input, planner_result);
        cout << "Found goal_distance: " << planner_result.goal_distance << endl;
        cout << "Found path_cost: " << planner_result.path_cost << endl;
        cout << "Acceptable goal nodes: " << planner_result.acceptable_goal_node_ids.size() << endl;

#if MRPT_HAS_WXWIDGETS
        mrpt::opengl::COpenGLScene::Ptr &scene = win.get3DSceneAndLock();
        scene->clear();
        PlannerRRT_SE2_TPS::TRenderPlannedPathOptions render_opts;
        render_opts.highlight_path_to_node_id = planner_result.best_goal_node_id;

        planner.renderMoveTree(*scene, planner_input, planner_result, render_opts);
        win.unlockAccess3DScene();
        win.repaint();
        win.waitForKey();
#endif
    }
}

int main(int argc, char **argv)
{
    try
    {
        TestRRT1();
        return 0;
    }
    catch (exception &e)
    {
        cout << "MRPT exception caught: " << e.what() << endl;
        return -1;
    }
    catch (...)
    {
        printf("Another exception!!");
        return -1;
    }
}