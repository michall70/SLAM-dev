#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>

#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct Pose {
    int             frame;
    Eigen::Vector3d t;   // camera center in world
    Eigen::Matrix3d R;   // world->cam rotation
};

static std::vector<Pose> loadTrajectory(const std::string &path) {
    std::vector<Pose> poses;
    std::ifstream     f(path);
    if (!f) {
        std::cerr << "Cannot open " << path << std::endl;
        return poses;
    }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        Pose p;
        p.R.setIdentity();
        ss >> p.frame >> p.t.x() >> p.t.y() >> p.t.z() >> p.R(0, 0) >> p.R(0, 1) >> p.R(0, 2) >> p.R(1, 0) >> p.R(1, 1) >> p.R(1, 2) >> p.R(2, 0) >> p.R(2, 1) >> p.R(2, 2);
        poses.push_back(p);
    }
    return poses;
}

static void drawCameraFrustum(const Eigen::Vector3d &pos, const Eigen::Matrix3d &R_wc, float size) {
    // R_wc = world→cam.  Camera-space → world: R_wc^T
    Eigen::Matrix3d R_cw = R_wc.transpose();

    // Camera frustum: center at origin, looking along +Z
    // Four corners at image plane z = size*2
    float hw = size;       // half-width  at image plane
    float hh = size * 0.75f;  // half-height at image plane
    float fz = size * 2.0f;   // image plane z

    Eigen::Vector3d c[5];
    c[0] = Eigen::Vector3d(0.0, 0.0, 0.0);       // camera center
    c[1] = Eigen::Vector3d(-hw, -hh, fz);         // bottom-left  of image plane
    c[2] = Eigen::Vector3d(hw, -hh, fz);          // bottom-right
    c[3] = Eigen::Vector3d(hw, hh, fz);           // top-right
    c[4] = Eigen::Vector3d(-hw, hh, fz);          // top-left

    // Transform to world
    for (int i = 0; i < 5; i++) {
        c[i] = pos + R_cw * c[i];
    }

    glLineWidth(2);
    glBegin(GL_LINE_LOOP);
    glVertex3d(c[1].x(), c[1].y(), c[1].z());
    glVertex3d(c[2].x(), c[2].y(), c[2].z());
    glVertex3d(c[3].x(), c[3].y(), c[3].z());
    glVertex3d(c[4].x(), c[4].y(), c[4].z());
    glEnd();

    glBegin(GL_LINES);
    for (int i = 1; i <= 4; i++) {
        glVertex3d(c[0].x(), c[0].y(), c[0].z());
        glVertex3d(c[i].x(), c[i].y(), c[i].z());
    }
    glEnd();
    glLineWidth(1);
}

static void drawCameraAxes(const Eigen::Vector3d &pos, const Eigen::Matrix3d &R_wc, float len) {
    Eigen::Matrix3d R_cw = R_wc.transpose();
    Eigen::Vector3d ex = pos + R_cw * Eigen::Vector3d(len, 0, 0);
    Eigen::Vector3d ey = pos + R_cw * Eigen::Vector3d(0, len, 0);
    Eigen::Vector3d ez = pos + R_cw * Eigen::Vector3d(0, 0, len);

    glLineWidth(3);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0); glVertex3d(pos.x(), pos.y(), pos.z()); glVertex3d(ex.x(), ex.y(), ex.z());
    glColor3f(0, 1, 0); glVertex3d(pos.x(), pos.y(), pos.z()); glVertex3d(ey.x(), ey.y(), ey.z());
    glColor3f(0, 0, 1); glVertex3d(pos.x(), pos.y(), pos.z()); glVertex3d(ez.x(), ez.y(), ez.z());
    glEnd();
    glLineWidth(1);
}

int main(int argc, char **argv) {
    std::string dataPath = "./output/trajectory.txt";
    if (argc > 1) dataPath = argv[1];

    auto poses = loadTrajectory(dataPath);
    if (poses.empty()) {
        std::cerr << "No poses loaded." << std::endl;
        return 1;
    }
    std::cout << "Loaded " << poses.size() << " poses." << std::endl;

    // Compute center and extent
    Eigen::Vector3d center(0, 0, 0);
    for (auto &p : poses) center += p.t;
    center /= poses.size();

    double maxDist = 0;
    for (auto &p : poses) {
        double d = (p.t - center).norm();
        if (d > maxDist) maxDist = d;
    }
    double camDist = std::max(maxDist * 3.0, 10.0);

    // ─── Pangolin ───────────────────────────────────────────────────────────
    pangolin::CreateWindowAndBind("3D Trajectory Viewer", 1280, 800);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1280, 800, 900, 900, 640, 400, 0.1, 1000),
        pangolin::ModelViewLookAt(center.x(), center.y() - camDist, center.z() + camDist * 0.3,
                                  center.x(), center.y(), center.z(),
                                  pangolin::AxisZ));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    // UI panel
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(200));

    pangolin::Var<bool>   showFrustums("ui.Show frustums", true);
    pangolin::Var<bool>   showCameraAxes("ui.Show camera axes", true);
    pangolin::Var<bool>   showWorldAxes("ui.Show world axes", true);
    pangolin::Var<bool>   showTrajectory("ui.Show trajectory", true);
    pangolin::Var<float>  frustSz("ui.Frustum size", 0.02, 0.01, 0.1);
    pangolin::Var<int>    frustStep("ui.Frustum step", 5, 1, 20);
    pangolin::Var<float>  axesLen("ui.Axes length", 0.05, 0.01, 0.2);
    pangolin::Var<bool>   resetView("ui.Reset view", false);
    pangolin::Var<bool>   saveImg("ui.Save snapshot", false);

    int saveCount = 0;

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (pangolin::Pushed(resetView)) {
            s_cam = pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(1280, 800, 900, 900, 640, 400, 0.1, 1000),
                pangolin::ModelViewLookAt(center.x(), center.y() - camDist, center.z() + camDist * 0.3,
                                          center.x(), center.y(), center.z(),
                                          pangolin::AxisZ));
        }

        d_cam.Activate(s_cam);

        // ─── World axes ─────────────────────────────────────────────────────
        if (showWorldAxes) {
            float wLen = static_cast<float>(camDist * 0.15);
            glLineWidth(3);
            glBegin(GL_LINES);
            glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(wLen, 0, 0);
            glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, wLen, 0);
            glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, wLen);
            glEnd();
            glLineWidth(1);

            float ofs = wLen * 1.05f;
            glColor3f(1, 0, 0);
            pangolin::default_font().Text("X").Draw(ofs, 0, 0);
            glColor3f(0, 1, 0);
            pangolin::default_font().Text("Y").Draw(0, ofs, 0);
            glColor3f(0, 0, 1);
            pangolin::default_font().Text("Z").Draw(0, 0, ofs);
        }

        // ─── Trajectory path ─────────────────────────────────────────────────
        if (showTrajectory) {
            glColor3f(0.15, 0.4, 0.85);
            glLineWidth(2);
            glBegin(GL_LINE_STRIP);
            for (auto &p : poses) {
                glVertex3d(p.t.x(), p.t.y(), p.t.z());
            }
            glEnd();

            // Start / end dots
            glPointSize(8);
            glBegin(GL_POINTS);
            glColor3f(0, 1, 0);
            glVertex3d(poses.front().t.x(), poses.front().t.y(), poses.front().t.z());
            glColor3f(1, 0, 0);
            glVertex3d(poses.back().t.x(), poses.back().t.y(), poses.back().t.z());
            glEnd();
            glPointSize(1);
        }

        // ─── Camera frustums & axes ──────────────────────────────────────────
        if (showFrustums || showCameraAxes) {
            int step = frustStep;
            for (size_t i = 0; i < poses.size(); i += step) {
                // Color cycles through rainbow
                float t = static_cast<float>(i) / poses.size();
                glColor3f(1 - t, 0.3f, t);

                if (showFrustums) {
                    float sz = frustSz * static_cast<float>(camDist);
                    drawCameraFrustum(poses[i].t, poses[i].R, sz);
                }
                if (showCameraAxes) {
                    float len = axesLen * static_cast<float>(camDist);
                    drawCameraAxes(poses[i].t, poses[i].R, len);
                }
            }
        }

        // HUD overlay
        glColor3f(0.1f, 0.1f, 0.1f);
        std::string hud = std::to_string(poses.size()) + " poses  |  Mouse: rotate / pan / scroll-zoom";
        pangolin::default_font().Text(hud).DrawWindow(10, 10);

        pangolin::FinishFrame();

        if (pangolin::Pushed(saveImg)) {
            char buf[64];
            snprintf(buf, sizeof(buf), "./output/traj_output/snapshot_%04d.png", saveCount++);
            pangolin::SaveWindowOnRender(buf);
        }
    }

    return 0;
}
