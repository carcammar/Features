#include "optimization.h"

Optimization::Optimization()
{

}

bool Optimization::visualBA(Map* pMap)
{
    ceres::Problem problem;
    std::vector<double*> vCamParam;
    std::vector<double*> vPointParam;

    unsigned int nKFs = pMap->GetNumberKFs();
    unsigned int nPoints = pMap->GetNumberPoints();

    // 0. Build optimization problem
    vCamParam.resize(nKFs);
    vPointParam.resize(nPoints);

    Eigen::Vector3f phi;
    Eigen::Vector3f r;
    Eigen::Matrix<float,4,4> Tcw;

    std::map<KeyFrame*,Eigen::Vector2f>::iterator itMap;
    Eigen::Vector2f pt;
    bool bGood;

    double *params; //optimization parameters
    std::vector<Point*> vpPoints;
    vpPoints.reserve(nPoints);
    vpPoints = pMap->GetPoints();
    std::vector<KeyFrame*> vpKFs;
    vpKFs.reserve(nKFs);
    vpKFs = pMap->GetKFs();

    // 0.0 KF parameters

    for(unsigned int i = 0; i < vpKFs.size(); i++)
    {
        Tcw = vpKFs[i]->GetPose();
        phi = Maths::LogSO3(Tcw.topLeftCorner(3,3));
        params = new double[6]; // set new address (phi,t)

        params[0] = static_cast <double>(phi(0));
        params[1] = static_cast <double>(phi(1));
        params[2] = static_cast <double>(phi(2));
        params[3] = static_cast <double>(Tcw(0,3));
        params[4] = static_cast <double>(Tcw(1,3));
        params[5] = static_cast <double>(Tcw(2,3));
        vCamParam[i] = params;
    }

    // 0.1 Point parameters
    for(unsigned int i = 0; i < vpPoints.size(); i++)
    {
        Point *point = vpPoints[i];
        params = new double[3]; // (r)
        r = point->GetPosition();
        params[0] = static_cast <double>(r(0));
        params[1] = static_cast <double>(r(1));
        params[2] = static_cast <double>(r(2));
        vPointParam[i] = params;
        // std::cout << "Point Parameters: " << params << "   " << params[0] << ", " << params[1] << ", " << params[2] << std::endl;

        // 0.2 Add observations
        for(unsigned int j = 0; j < vpKFs.size(); j++)
        {
            KeyFrame *kf = vpKFs[j];
            bGood = point->GetObservation(kf, pt);
            if (!bGood)
                continue;
            // std::cout << "Obs KF: " << kf->mId << std::endl;
            ceres::CostFunction* cost_function = cerestypes::ReprojectionError::Create(pt, point, kf);
            // TODO Add Hubber robust kernel
            problem.AddResidualBlock(cost_function, NULL, vCamParam[j], vPointParam[i]);
            // TODO check that point to be added has at least two observations in added frames
        }
    }

    std::cout << "Point blocks: " << vPointParam.size() << std::endl;
    std::cout << "Camera blokcs: " << vCamParam.size() << std::endl;

    // 1. Optimize
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 2. Get optimized results
    // 2.1 KFs
    KeyFrame *pKF;
    for(unsigned int i = 0; i < vCamParam.size(); i++)
    {
        if (!vCamParam[i]) // Check camera has been included into optimization
            continue;
        pKF = vpKFs[i];
        double *pCam = vCamParam[i];
        Eigen::Vector3f phi;
        phi << static_cast<float>(pCam[0]), static_cast<float>(pCam[1]), static_cast<float>(pCam[2]);
        Tcw.topLeftCorner(3,3) = Maths::ExpSO3(phi);
        Tcw(0,3) = static_cast<float>(pCam[3]);
        Tcw(1,3) = static_cast<float>(pCam[4]);
        Tcw(2,3) = static_cast<float>(pCam[5]);
        pKF->SetPose(Tcw);
    }

    // Points
    Point *pP;
    for(unsigned int i = 0; i < vPointParam.size(); i++)
    {
        if (!vPointParam[i]) // Check point has been included into optimization
            continue;
        pP = vpPoints[i];
        double *pPoint = vPointParam[i];
        Eigen::Vector3f pos;
        pos << static_cast<float>(pPoint[0]), static_cast<float>(pPoint[1]), static_cast<float>(pPoint[2]);
        pP->SetPosition(pos);
    }

    return true;
}
