#include "camera.h"
#include "visualizer.h"
#include "triangulation.h"

#include <chrono>
#include <thread>

int main(int argc, char **argv){
    Camera camera = Camera(std::string(argv[1]));
    camera.GenerateFrames();

    Eigen::Matrix3x4d proj_mat1 = camera.GetFirstFrameProjectMatrix();
    Eigen::Matrix3x4d proj_mat2 = camera.GetSecondFrameProjectMatrix();

    std::vector<Eigen::Vector2d> points1 = camera.GetFirstFramePoints();
    std::vector<Eigen::Vector2d> points2 = camera.GetSecondFramePoints();

    std::vector<Eigen::Vector3d> triangulation_results = TriangulatePoints(proj_mat1, proj_mat2, points1, points2);

    int frame_width = 1920;
    int frame_height = 1080;

    pangolin::CreateWindowAndBind("PoseViewer", frame_width, frame_height);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(frame_width,frame_height, (frame_width+frame_height)/2,
                                            (frame_width+frame_height)/2, frame_width / 2, frame_height / 2,
                                            0.0001, 100000),
                pangolin::ModelViewLookAt(-10, -10, 0, 0, 0, 0, pangolin::AxisZ)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -float(frame_width)/frame_height)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        DrawModel(camera.ModelPoints, camera.ModelLines);
        DrawFrames(camera.Twcs);

        DrawEstimatedModel(triangulation_results);

        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}