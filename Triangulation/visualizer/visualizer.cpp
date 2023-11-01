#include "visualizer.h"

void DrawFrames(std::vector<Eigen::Matrix4d> PoseTwc)
{
    const float w = 0.3;
    const float h = w*0.75;
    const float z = w*0.6;

    for(size_t i=0; i<PoseTwc.size(); i++)
    {
        Eigen::Matrix4f Twc = PoseTwc[i].cast<float>();
        glPushMatrix();
        glMultMatrixf(Twc.data());

        glLineWidth(2);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

void DrawModel(Points ModelPoints, Lines ModelLines) {

    glColor4f(0, 0.9, 0, 0.6);
    glLineWidth(3.0f);
        
    for(int i = 0; i < ModelLines.size(); i++) {
        Line l = ModelLines[i];
        Point p1 = ModelPoints[l.first];
        Point p2 = ModelPoints[l.second];

        glBegin(GL_LINES);
        glVertex3d(p1(0), p1(1), p1(2));
        glVertex3d(p2(0), p2(1), p2(2));
        glEnd();
    }

}

void DrawTwbFrames(std::vector<Eigen::Matrix4d> PoseTwb) {
    const float w = 0.3;
    const float h = w*0.75;
    const float z = w*0.6;

    for(size_t i=0; i<PoseTwb.size(); i++)
    {
        Eigen::Matrix4f Twc = PoseTwb[i].cast<float>();
        glPushMatrix();
        glMultMatrixf(Twc.data());

        glLineWidth(2);
        glColor3f(1.0f,0.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

void DrawEstimatedModel(std::vector<Eigen::Vector3d> Points) {

    glColor4f(0.9, 0.0, 0, 0.6);
    glPointSize(10.0f);

    glBegin(GL_POINTS);
        
    for(int i = 0; i < Points.size(); i++) {
        Eigen::Vector3d p = Points[i];
        
        glVertex3d(p(0), p(1), p(2));
        
    }
    glEnd();

}