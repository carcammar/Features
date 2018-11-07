#include "optimization.h"

Optimization::Optimization()
{

}

bool Optimization::visualBA(Map* pMap)
{
    ceres::Problem problem;
    std::vector<double*> vCamParam;
    std::vector<double*> vPointParam;

    // 0. Build optimization problem
    vCamParam.resize(pMap->mvpKFs.size());
    vPointParam.resize(pMap->mvpPoints.size());
    Eigen::Vector3f phi;
    Eigen::Vector3f r;
    Eigen::Matrix<float,4,4> Tcw;

    std::map<KeyFrame*,Eigen::Vector2f>::iterator itMap;
    // TODO erase test
    const double stdAng = 0.5;//1.0;
    const double stdTrans = 1.0; //2.0;
    const double stdPoint = 1.0; //2.0;

    double *params;

    // 0.0 KF parameters
    for(unsigned int i = 0; i < pMap->mvpKFs.size(); i++)
    {
        Tcw = pMap->mvpKFs[i]->mTcw;
        phi = Maths::LogSO3(Tcw.topLeftCorner(3,3));
        params = new double[6]; // (phi,t)
        // TODO Remove noise addition used for test
        params[0] = static_cast <double>(phi(0)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdAng;
        params[1] = static_cast <double>(phi(1)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdAng;
        params[2] = static_cast <double>(phi(2)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdAng;
        params[3] = static_cast <double>(Tcw(0,3)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdTrans;
        params[4] = static_cast <double>(Tcw(1,3)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdTrans;
        params[5] = static_cast <double>(Tcw(2,3)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdTrans;
        vCamParam[i] = params;
    }

    // 0.1 Point parameters
    for(unsigned int i = 0; i < pMap->mvpPoints.size(); i++)
    {
        Point *point = pMap->mvpPoints[i];
        params = new double[3]; // (r)
        r = point->mr;
        params[0] = static_cast <double>(r(0)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdPoint;
        params[1] = static_cast <double>(r(1)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdPoint;
        params[2] = static_cast <double>(r(2)) + (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5)*stdPoint;
        vPointParam[i] = params;
        // std::cout << "Point Parameters: " << params << "   " << params[0] << ", " << params[1] << ", " << params[2] << std::endl;

        // 0.2 Add observations
        for(unsigned int j = 0; j < pMap->mvpKFs.size(); j++)
        {
            KeyFrame *kf = pMap->mvpKFs[j];
            itMap = point->mmpKFObs.find(kf);
            if (itMap == point->mmpKFObs.end())
                continue;
            // std::cout << "Obs KF: " << kf->mId << std::endl;
            ceres::CostFunction* cost_function = cerestypes::ReprojectionError::Create(itMap->second, point, itMap->first);
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
    KeyFrame *pKF;
    for(unsigned int i = 0; i < vCamParam.size(); i++)
    {
        if (!vCamParam[i]) // Check camera has been included into optimization
            continue;
        pKF = pMap->mvpKFs[i];
        double *pCam = vCamParam[i];
        Eigen::Vector3f phi;
        phi << static_cast<float>(pCam[0]), static_cast<float>(pCam[1]), static_cast<float>(pCam[2]);
        pKF->mTcw.topLeftCorner(3,3) = Maths::ExpSO3(phi);
        pKF->mTcw(0,3) = static_cast<float>(pCam[3]);
        pKF->mTcw(1,3) = static_cast<float>(pCam[4]);
        pKF->mTcw(2,3) = static_cast<float>(pCam[5]);
    }

    return true;
}
