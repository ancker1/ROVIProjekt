
#include <rw/rw.hpp>

namespace interpolator
{ namespace util {
std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    /******************************************************************************************
     * This function was provided in Robotics, Lab Assignment 5                               *
     ******************************************************************************************/
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==nullptr || frameTcp==nullptr || frameRobotBase==nullptr || frameRobotTcp==nullptr)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==nullptr ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

rw::math::Q getCollisionFreeSolution(rw::models::SerialDevice::Ptr UR6, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector, std::vector<rw::math::Q> solutions)
{
    rw::math::Q configuration;
    for ( unsigned int i = 0; i < solutions.size(); i++ )
    {
        UR6->setQ(solutions[i], state);
        if ( !detector->inCollision(state, NULL, true) )
        {
            configuration = solutions[i];
            break;
        }
    }
    return configuration;
}

rw::math::Q getCollisionFreeSolution(rw::models::SerialDevice::Ptr UR6, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector, std::vector<rw::math::Q> solutions, rw::math::Q prevSolution)
{
    rw::math::Q configuration;
    double dist = std::numeric_limits<double>::max();

    for ( unsigned int i = 0; i < solutions.size(); i++ )
    {
        UR6->setQ(solutions[i], state);
        if ( !detector->inCollision(state, NULL, true) )
        {
            double cdist = rw::math::Q(prevSolution - solutions[i]).norm2();
            if ( cdist < dist )
            {
                configuration = solutions[i];
                dist = cdist;
            }
        }
    }
    return configuration;
}

std::tuple<std::vector<rw::math::Transform3D<>>,std::vector<float>> getPointTimes(rw::math::Transform3D<> pickFrame, rw::math::Transform3D<> homeFrame, rw::math::Transform3D<> placeFrame)
{   // Read/define frames...
    // start point: home
     rw::math::Transform3D<> P0 = homeFrame;
     float t0 = 0.0f;
    // 1st point: above target
    rw::math::Transform3D<> P1 = pickFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t1 = 1.0f;
    // 2nd point: target
    rw::math::Transform3D<> P2 = pickFrame;
    float t2 = 2.0f;
    // 3rd point: above target
    rw::math::Transform3D<> P3 = pickFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t3 = 3.0f;
    // 4th point: home pos
    rw::math::Transform3D<> P4 = homeFrame;
    float t4 = 4.0f;
    // 5th point: above place
    rw::math::Transform3D<> P5 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t5 = 5.0f;
    // 6th point: place
    rw::math::Transform3D<> P6 = placeFrame;
    float t6 = 6.0f;
    // 7th point: above place
    rw::math::Transform3D<> P7 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t7 = 7.0f;
    // 8th point: home
    rw::math::Transform3D<> P8 = homeFrame;
    float t8 = 8.0f;

    std::vector<rw::math::Transform3D<>> points = {P0, P1, P2, P3, P4, P5, P6, P7, P8};
    std::vector<float> times = {t0, t1, t2, t3, t4, t5, t6, t7, t8};

    return std::make_tuple(points, times);
}

std::vector<rw::math::Q> mapCartesianToJoint( std::vector<rw::math::Transform3D<>> CPath, rw::kinematics::MovableFrame *targetFrame, rw::models::SerialDevice::Ptr UR6, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector )
{
    std::vector<rw::math::Q> QPath;
    for ( rw::math::Transform3D<> T : CPath )
    {
        targetFrame->moveTo(T, state);
        std::vector<rw::math::Q> solutions = interpolator::util::getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);
        rw::math::Q configuration = interpolator::util::getCollisionFreeSolution(UR6, state, detector, solutions);
        QPath.push_back(configuration);
    }
    return QPath;
}

}
/***********************************************************************************************************
  *  The interpolation functions is based on the lecture notes written by Aljaz Kramberger & Henrik Gordon *
  **********************************************************************************************************/

  std::vector<rw::math::Transform3D<>> parabolicBlend(std::vector<rw::math::Transform3D<>> P, std::vector<float> T)
  {   // we wish to limit acceleration during blend (puts lower limit on tau)
      // distance largest at: d = 1/4*(v2 - v1)*tau (puts upper limit on tau)

      //(v2 - v1)/(4*tau)*pow(t + tau, 2)+ v1*t+Pi;
      std::vector<rw::math::Transform3D<>> Path;
      std::vector<rw::math::Vector3D<>> Ps;
      float tau = 0.25;
      for ( float t = 0; t < T[1]; t += 0.01f )
          if ( t <= T[0] + tau )
          {
              rw::math::Vector3D<> Pi = P[0].P() + (P[1].P() - P[0].P())*(t - T[0])/(T[1] - T[0]);
              Ps.push_back(Pi);
              rw::math::EAA<> eaaDiff(P[1].R() * rw::math::inverse(P[0].R()));
              rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[0])/(T[1]-T[0]));
              Path.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[0].R() ));
          }

      for ( unsigned int i = 1; i < P.size(); i++ )
      {
          // Create linear intepolation between all P's
          for ( float t = T[i-1]; t < T[i+1]; t += 0.01f )
          {
              if ( T[i-1] + tau <= t && t <= T[i] - tau )
              {
                  rw::math::Vector3D<> Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*(t - T[i-1])/(T[i] - T[i-1]);
                  Ps.push_back(Pi);
                  rw::math::EAA<> eaaDiff(P[i-1].R() * rw::math::inverse(P[i].R()));
                  rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[i])/(T[i-1]-T[i]));
                  Path.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[i].R() ));
              }
              else if ( T[i] - tau <= t && t <= T[i] + tau ) // If within blend interval
              {
                  rw::math::Vector3D<> X = P[i].P();
                  rw::math::Vector3D<> v1 = (P[i-1].P() - P[i].P())/(T[i-1] - T[i]); // (P[i-1]-P[i])/(T[i-1]-T[i]) = P[i]-P[i-1], when all t = 1
                  rw::math::Vector3D<> v2 = (P[i+1].P() - P[i].P())/(T[i+1] - T[i]); // since all t = 1
                  rw::math::Vector3D<> Pblend = (v2 - v1)/(4 * tau) * pow(t - T[i] + tau, 2) + v1*(t-T[i]) + X;
                  Ps.push_back(Pblend);

                  rw::math::EAA<> vR1eaa(P[i-1].R() * rw::math::inverse(P[i].R()));
                  rw::math::Vector3D<> vR1 = rw::math::Vector3D<>(vR1eaa.axis()*vR1eaa.angle() / (T[i-1] - T[i]));

                  rw::math::EAA<> vR2eaa(P[i+1].R() * rw::math::inverse(P[i].R()));
                  rw::math::Vector3D<> vR2 = rw::math::Vector3D<>(vR2eaa.axis()*vR2eaa.angle() / (T[i+1] - T[i]));

                  rw::math::Vector3D <> vBlend = (vR2 - vR1)/(4*tau)*pow(t - T[i] + tau, 2) + vR1*(t-T[i]); // + (X=0)

                  rw::math::Rotation3D<> Rblend = rw::math::EAA<>(vBlend).toRotation3D()*P[i].R();

                  //rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[i-1])/(T[i]-T[i-1]));
                  Path.push_back(rw::math::Transform3D<>( Pblend, Rblend ));
              }

          }

      }

      for ( float t = T[T.size()-2]; t < T[T.size() - 1]; t += 0.01f )
          if ( t >= T[T.size()-2] + tau )
          {
              rw::math::Vector3D<> Pi = P[P.size()-2].P() + (P[P.size()-1].P() - P[P.size()-2].P())*(t - T[T.size()-2])/(T[T.size()-1] - T[T.size()-2]);
              Ps.push_back(Pi);
              rw::math::EAA<> eaaDiff(P[P.size()-1].R() * rw::math::inverse(P[P.size()-2].R()));
              rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[T.size()-2])/(T[T.size()-1]-T[T.size()-2]));
              Path.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[P.size()-2].R() ));
          }

      return Path;
  }

  std::vector<rw::math::Transform3D<>> linearInterpolate(std::vector<rw::math::Transform3D<>> P, std::vector<float> T)
  {
      std::vector<rw::math::Transform3D<>> path;
      for ( unsigned int i = 1; i < P.size(); i++ )
      {
          for ( float t = T[i-1]; t < T[i]; t += 0.01f )
          {
              // Interpolate transforms...
              rw::math::Vector3D<> Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*(t - T[i-1])/(T[i] - T[i-1]);
              rw::math::EAA<> eaaDiff(P[i-1].R() * P[i].R().inverse());
              rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[i])/(T[i-1]-T[i]));
              path.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[i].R() ));
          }
      }
      return path;
  }

  std::vector<rw::math::Q> linerInterpolateQ(std::vector<rw::math::Transform3D<>> Ts, std::vector<float> Times, rw::kinematics::MovableFrame *targetFrame, rw::models::SerialDevice::Ptr UR6, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector)
  {   // Linear in configuration space
      std::vector<rw::math::Q> path;
      std::vector<rw::math::Q> TsQ;
      for ( unsigned int i = 0; i < Ts.size(); i++ )
      {
          targetFrame->moveTo(Ts[i], state);
          std::vector<rw::math::Q> solutions = interpolator::util::getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);
          rw::math::Q configuration = interpolator::util::getCollisionFreeSolution(UR6, state, detector, solutions);
          TsQ.push_back(configuration);
      }


      for ( unsigned int i = 1; i < TsQ.size(); i++ )
      {
          for ( float t = 0.0f; t < 1.0f; t += 0.1f )
          {

              // Interpolate configurations...
              rw::math::Q Ximin1 = TsQ[i-1];
              rw::math::Q Xi = TsQ[i];
              rw::math::Q Xit = Ximin1 + ( Xi - Ximin1 ) * t;
              path.push_back(Xit);
              std::cout << Xit << std::endl;
          }
      }
      return path;
  }



  std::tuple<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> linearInterpolate(std::vector<rw::math::Transform3D<>> P, std::vector<float> T, rw::kinematics::MovableFrame *targetFrame, rw::models::SerialDevice::Ptr UR6, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector)
  {   // Linear in cartesian space
      std::vector<rw::math::Q> path;
      std::vector<rw::math::Transform3D<>> xits;
      for ( unsigned int i = 1; i < P.size(); i++ )
      {
          //std::cout <<  "Pi" <<  P[i] << std::endl;
          for ( float t = T[i-1]; t < T[i]; t += 0.01f )
          {

              // Interpolate transforms...
              rw::math::Vector3D<> Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*(t - T[i-1])/(T[i] - T[i-1]);

              rw::math::EAA<> eaaDiff(P[i].R() * rw::math::inverse(P[i-1].R()));
              rw::math::Vector3D<> vR = rw::math::Vector3D<>(eaaDiff.axis()*eaaDiff.angle() * (t - T[i-1])/(T[i]-T[i-1]));
              rw::math::EAA<> eaaChange(vR);

              xits.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[i-1].R() ));

              targetFrame->moveTo(xits.back(), state);

              std::vector<rw::math::Q> solutions = interpolator::util::getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);

              rw::math::Q configuration;
              if ( path.size() > 0 )
                  configuration = interpolator::util::getCollisionFreeSolution(UR6, state, detector, solutions, path.back());
              else
                  configuration = interpolator::util::getCollisionFreeSolution(UR6, state, detector, solutions);

              path.push_back(configuration);
          }
      }
      return std::make_tuple(path, xits);
  }


}
