#include "estimator.h"

void drawImage(const cv::Mat &img, const vector<cv::Point2f> &pts, string name)
{
  auto draw = img.clone();
  for (unsigned int i = 0; i < pts.size(); i++)
  {
    cv::circle(draw, pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
  }
  cv::imshow(name, draw);
  cv::waitKey(1);
}

Estimator::Estimator()
{
  ROS_INFO("Estimator init begins.");
  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = Eigen::Vector3d(0, 0, 0);
  prev_frame.w_R_c = Eigen::Matrix3d::Identity();
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::reset()
{
  ROS_ERROR("Lost, reset!");
  key_frame = prev_frame;
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::setParameter()
{
  for (int i = 0; i < 2; i++)
  {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " exitrinsic cam " << i << endl
         << ric[i] << endl
         << tic[i].transpose() << endl;
  }

  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = tic[0];
  prev_frame.w_R_c = ric[0];
  key_frame = prev_frame;

  readIntrinsicParameter(CAM_NAMES);

  // transform between left and right camera
  Matrix4d Tl, Tr;
  Tl.setIdentity();
  Tl.block(0, 0, 3, 3) = ric[0];
  Tl.block(0, 3, 3, 1) = tic[0];
  Tr.setIdentity();
  Tr.block(0, 0, 3, 3) = ric[1];
  Tr.block(0, 3, 3, 1) = tic[1];
  Tlr = Tl.inverse() * Tr;
}

void Estimator::readIntrinsicParameter(const vector<string> &calib_file)
{
  for (size_t i = 0; i < calib_file.size(); i++)
  {
    ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera.push_back(camera);
  }
}

bool Estimator::inputImage(ros::Time time_stamp, const cv::Mat &_img, const cv::Mat &_img1)
{

  if (fail_cnt > 20)
  {
    reset();
  }
  std::cout << "receive new image===========================" << std::endl;

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp;
  cur_frame.img = _img;

  // cv::imshow("img", _img);
  // cv::waitKey(1);

  vector<cv::Point2f> left_pts_2d, right_pts_2d;
  vector<cv::Point3f> key_pts_3d;

  c_R_k.setIdentity();
  c_t_k.setZero();

  if (init_finish)
  {
    // To do: match features between the key frame and the current left image

    // To do: undistort the points of the left image and compute relative motion with the key frame.

  }

  // To do: extract new features for the current frame.

  // To do: compute the camera pose of the current frame.

  // To do: undistort the 2d points of the current frame and generate the corresponding 3d points.

  // Change key frame
  if (c_t_k.norm() > TRANSLATION_THRESHOLD || acos(Quaterniond(c_R_k).w()) * 2.0 > ROTATION_THRESHOLD || key_pts_3d.size() < FEATURE_THRESHOLD || !init_finish)
  {
    key_frame = cur_frame;
    ROS_INFO("Change key frame to current frame.");
  }

  prev_frame = cur_frame;

  updateLatestStates(cur_frame);

  init_finish = true;

  return true;
}

bool Estimator::trackFeatureBetweenFrames(const Estimator::frame &keyframe, const cv::Mat &cur_img,
                                          vector<cv::Point3f> &key_pts_3d,
                                          vector<cv::Point2f> &cur_pts_2d)
{

  // To do: track features between the key frame and the current frame to obtain corresponding 2D, 3D points.

  return true;
}

bool Estimator::estimateTBetweenFrames(vector<cv::Point3f> &key_pts_3d,
                                       vector<cv::Point2f> &cur_pts_2d, Matrix3d &R, Vector3d &t)
{

  // To do: calculate relative pose between the key frame and the current frame using the matched 2d-3d points

  return true;
}

void Estimator::extractNewFeatures(const cv::Mat &img, vector<cv::Point2f> &uv)
{

  //To do: extract the new 2d features of img and store them in uv.
}

bool Estimator::trackFeatureLeftRight(const cv::Mat &_img, const cv::Mat &_img1,
                                      vector<cv::Point2f> &left_pts, vector<cv::Point2f> &right_pts)
{

  // To do: track features left to right frame and obtain corresponding 2D points.

  return true;
}

void Estimator::generate3dPoints(const vector<cv::Point2f> &left_pts,
                                 const vector<cv::Point2f> &right_pts,
                                 vector<cv::Point3f> &cur_pts_3d,
                                 vector<cv::Point2f> &cur_pts_2d)
{

  Eigen::Matrix<double, 3, 4> P1, P2;

  P1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
  P2.block(0, 0, 3, 3) = (Tlr.block(0, 0, 3, 3).transpose());
  P2.block(0, 3, 3, 1) = -P2.block(0, 0, 3, 3) * Tlr.block(0, 3, 3, 1);

  vector<uchar> status;

  for (unsigned int i = 0; i < left_pts.size(); ++i)
  {
    Vector2d pl(left_pts[i].x, left_pts[i].y);
    Vector2d pr(right_pts[i].x, right_pts[i].y);
    Vector3d pt3;
    triangulatePoint(P1, P2, pl, pr, pt3);

    if (pt3[2] > 0)
    {
      cur_pts_3d.push_back(cv::Point3f(pt3[0], pt3[1], pt3[2]));
      status.push_back(1);
    }
    else
    {
      status.push_back(0);
    }
  }

  reduceVector<cv::Point2f>(cur_pts_2d, status);
}

bool Estimator::inBorder(const cv::Point2f &pt, const int &row, const int &col)
{
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y &&
         img_y < row - BORDER_SIZE;
}

double Estimator::distance(cv::Point2f pt1, cv::Point2f pt2)
{
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

template <typename Derived>
void Estimator::reduceVector(vector<Derived> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void Estimator::updateLatestStates(frame &latest_frame)
{

  // To do: update the latest_time, latest_pointcloud, latest_P, latest_Q, latest_rel_P and latest_rel_Q.
  // latest_P and latest_Q should be the pose of the body (IMU) in the world frame.
  // latest_rel_P and latest_rel_Q should be the relative pose of the current body frame relative to the body frame of the key frame.
  // latest_pointcloud should be in the current camera frame.
}

void Estimator::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                 Eigen::Vector2d &point0, Eigen::Vector2d &point1,
                                 Eigen::Vector3d &point_3d)
{
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Eigen::Vector4d triangulated_point;
  triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

double Estimator::reprojectionError(Matrix3d &R, Vector3d &t, cv::Point3f &key_pts_3d, cv::Point2f &cur_pts_2d)
{
  Vector3d pt1(key_pts_3d.x, key_pts_3d.y, key_pts_3d.z);
  Vector3d pt2 = R * pt1 + t;
  pt2 = pt2 / pt2[2];
  return sqrt(pow(pt2[0] - cur_pts_2d.x, 2) + pow(pt2[1] - cur_pts_2d.y, 2));
}

vector<cv::Point2f> Estimator::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
  vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++)
  {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}
