#include "displayer.h"

Displayer::Displayer(SLAM *pSlam): mpSlam(pSlam)
{
     mViewpointX = 0;
     mViewpointY = -0.7;
     mViewpointZ = -1.8;
     mViewpointF = 500;
}

void Displayer::Run()
{
    // For current frame
    cv::namedWindow("Test", CV_WINDOW_AUTOSIZE);


    // For map (TODO check all these values have been directly copied from ORBSLAM)
    pangolin::CreateWindowAndBind("Map",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();


    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        // glClearColor(1.0f,1.0f,1.0f,1.0f); // Draw white background


        // TODO Put if(map update) read 3d points
        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3f(1.0,0.0,0.0);
        v3DPointsToDraw = mpSlam->mpMap->GetPoints();
        Eigen::Vector3f t;
        for(std::vector<Point*>::iterator itPt = v3DPointsToDraw.begin(); itPt != v3DPointsToDraw.end(); itPt++)
        {
            t = (*itPt)->GetPosition();
            glVertex3f(t(0),t(1),t(2));
        }
        glEnd();

        // Draw keyframes
        vpKFsToDraw = mpSlam->mpMap->GetKFs();
        Eigen::Matrix<float,4,4> Twc;
        cv::Mat cvTwc;
        float w = 0.2f;
        float z = 0.2f;
        float h = 0.2f;
        for(std::vector<KeyFrame*>::iterator itKF = vpKFsToDraw.begin(); itKF != vpKFsToDraw.end(); itKF++)
        {
            Twc = (*itKF)->GetInvPose();
            // TODO why transpose? Ti works...
            cvTwc = Maths::Eigmat2Cvmat(Twc).t();
            glPushMatrix();
            glMultMatrixf(cvTwc.ptr<GLfloat>(0));
            glLineWidth(5);
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
        glEnd();

        if (mpSlam->IsNewImage())
        {
            mIm = mpSlam->GetImageToDraw();
            vPointsToDraw = mpSlam->GetPointToDraw();
        }

        if (!mIm.empty())
        {
            cv::drawKeypoints(mIm,vPointsToDraw,mIm);
            cv::imshow("Test", mIm);
        }

        // pangolin::glDrawColouredCube();

        pangolin::FinishFrame();
        cv::waitKey(30); // cv::waitKey(30); and no next line
        // std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}
