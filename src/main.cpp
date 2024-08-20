#include <iostream>
#include <opencv2/opencv.hpp>
#include "ELSED.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <fstream>
#include <string>
#include <dirent.h>
#include <unistd.h>
#include <opencv2/imgproc/types_c.h>
#include <vector>
#include "ELSEDMatch.h"

using namespace std;
vector<int> match1;
vector<int> match2;
Eigen::Matrix3d R, R1;

inline void
drawSegments(cv::Mat img,
             upm::Segments segs,
             const cv::Scalar &color,
             int thickness = 1,
             int lineType = cv::LINE_AA,
             int shift = 0)
{
  for (const upm::Segment &seg : segs)
    cv::line(img, cv::Point2f(seg[0], seg[1]), cv::Point2f(seg[2], seg[3]), color, thickness, lineType, shift);
}

void eq_liftProjective(Eigen::Vector2d p2d, Eigen::Vector3d &p3d)
{
  double lon;
  double lat;
  double left_sphere_up = M_PI / 180 * 90;
  double left_sphere_down = M_PI / 180 * 90;
  int new_rows = 512;
  int new_cols = 1024;
  double x = p2d(0);
  double y = p2d(1);
  lon = (x / double(new_cols) - 0.5) * (2 * M_PI);
  lat = -(y / double(new_rows) * (left_sphere_up + left_sphere_down) - left_sphere_up);
  Eigen::Vector3d tmp_p;
  tmp_p(2) = sin(lat);
  tmp_p(0) = cos(lat) * sin(lon);
  tmp_p(1) = cos(lat) * cos(lon);
  p3d = tmp_p;
}

void eq_spaceToPlane(Eigen::Vector3d p3d, Eigen::Vector2d &p2d)
{
  double lon;
  double lat;
  double left_sphere_up = M_PI / 180 * 90;
  double left_sphere_down = M_PI / 180 * 90;
  int new_rows = 512;
  int new_cols = 1024;
  p3d.normalize();
  double x = p3d(0);
  double y = p3d(1);
  double z = p3d(2);
  lon = atan2(x, y);
  lat = asin(z);
  Eigen::Vector2d tmp_p;
  tmp_p(0) = (lon / (2 * M_PI) + 0.5) * double(new_cols);
  tmp_p(1) = (-lat + left_sphere_up) / (left_sphere_up + left_sphere_down) * double(new_rows);
  if (tmp_p(0) <= 0)
  {
    tmp_p(0) = 0;
  }
  if (tmp_p(1) <= 0)
  {
    tmp_p(1) = 0;
  }
  if (tmp_p(0) >= new_cols - 1)
  {
    tmp_p(0) = new_cols - 1;
  }
  if (tmp_p(1) >= new_rows - 1)
  {
    tmp_p(1) = new_rows - 1;
  }

  p2d = tmp_p;
}

inline void
drawCurveSegments(cv::Mat img,
                  upm::Segments segs,
                  const cv::Scalar &color,
                  camodocal::CameraPtr m_camera, int thickness = 1)
{
  // center_x: 640.991301
  // center_y: 490.937512
  // max_r: 512
  // min_r: 160
  double center_x = 640.991301;
  double center_y = 490.937512;
  double max_r = 512;
  double min_r = 180;
  int id = 0;
  for (const upm::Segment &seg : segs)
  {
    int color_flag = 0;
    Eigen::Vector2d p1(seg[0], seg[1]);
    Eigen::Vector2d p2(seg[2], seg[3]);

    // double distance_center_p1 = sqrt((p1[0] - center_x) * (p1[0] - center_x) + (p1[1] - center_y) * (p1[1] - center_y));
    // double distance_center_p2 = sqrt((p2[0] - center_x) * (p2[0] - center_x) + (p2[1] - center_y) * (p2[1] - center_y));
    // if (distance_center_p1 > max_r || distance_center_p1 < min_r)
    // {
    //   color_flag = 1;
    // }
    // if (distance_center_p2 > max_r || distance_center_p2 < min_r)
    // {
    //   color_flag = 1;
    // }

    Eigen::Vector3d p1_3d, p2_3d;
    // m_camera->liftProjective(p1, p1_3d);
    // m_camera->liftProjective(p2, p2_3d);
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    double theta = acos(p1_3d.dot(p2_3d));

    if (abs(theta) < 5.0 / 180 * M_PI)
    {
      color_flag = 2;
    }
    Eigen::Vector3d axis = p1_3d.cross(p2_3d);
    axis.normalize();

    // for (double t = 0; t < 1; t += 0.01)
    for (double t = 0; t < theta; t += 0.1 / 180 * M_PI)
    {
      // Eigen::Vector3d p = p1_3d * cos(t * theta) + axis * sin(t * theta) * sin(t * theta);
      Eigen::AngleAxisd aa(t, axis);
      Eigen::Vector3d p = aa * p1_3d;
      Eigen::Vector2d p2d;
      // m_camera->spaceToPlane(p, p2d);
      eq_spaceToPlane(p, p2d);
      if (color_flag == 0)
        cv::circle(img, cv::Point2f(p2d[0], p2d[1]), 1, color, thickness);
      else if (color_flag == 1)
        cv::circle(img, cv::Point2f(p2d[0], p2d[1]), 1, CV_RGB(255, 0, 0), thickness);
      else if (color_flag == 2)
        cv::circle(img, cv::Point2f(p2d[0], p2d[1]), 1, CV_RGB(0, 0, 255), thickness);
    }
    // write id to image
    // cv::putText(img, to_string(id), cv::Point2f(seg[0], seg[1]), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1);
    id++;
  }
}

inline void
drawCurveSegments_match(cv::Mat img,
                        upm::Segments segs,
                        const cv::Scalar &color,
                        vector<int> match,
                        camodocal::CameraPtr m_camera, int thickness = 1)
{
  // center_x: 640.991301
  // center_y: 490.937512
  // max_r: 512
  // min_r: 160
  double center_x = 640.991301;
  double center_y = 490.937512;
  double max_r = 512;
  double min_r = 180;
  int id = 0;
  for (const upm::Segment &seg : segs)
  {
    int color_flag = 0;
    Eigen::Vector2d p1(seg[0], seg[1]);
    Eigen::Vector2d p2(seg[2], seg[3]);

    // double distance_center_p1 = sqrt((p1[0] - center_x) * (p1[0] - center_x) + (p1[1] - center_y) * (p1[1] - center_y));
    // double distance_center_p2 = sqrt((p2[0] - center_x) * (p2[0] - center_x) + (p2[1] - center_y) * (p2[1] - center_y));
    // if (distance_center_p1 > max_r || distance_center_p1 < min_r)
    // {
    //   color_flag = 1;
    // }
    // if (distance_center_p2 > max_r || distance_center_p2 < min_r)
    // {
    //   color_flag = 1;
    // }

    Eigen::Vector3d p1_3d, p2_3d;
    // m_camera->liftProjective(p1, p1_3d);
    // m_camera->liftProjective(p2, p2_3d);
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    double theta = acos(p1_3d.dot(p2_3d));

    // if (abs(theta) < 5.0 / 180 * M_PI)
    // {
    //   color_flag = 2;
    // }
    Eigen::Vector3d axis = p1_3d.cross(p2_3d);
    axis.normalize();

    if (match[id] == 1)
    {
      color_flag = 3;
      // cout << "match" << endl;
      // cout << id << endl;
    }

    // for (double t = 0; t < 1; t += 0.01)
    for (double t = 0; t < theta; t += 0.1 / 180 * M_PI)
    {
      // Eigen::Vector3d p = p1_3d * cos(t * theta) + axis * sin(t * theta) * sin(t * theta);
      Eigen::AngleAxisd aa(t, axis);
      Eigen::Vector3d p = aa * p1_3d;
      Eigen::Vector2d p2d;
      // m_camera->spaceToPlane(p, p2d);
      // purple
      cv::Scalar purple = CV_RGB(255, 0, 255);
      eq_spaceToPlane(p, p2d);
      if (color_flag == 0)
        cv::circle(img, cv::Point2f(p2d[0], p2d[1]), 1, color, thickness);
      else if (color_flag == 1)
        cv::circle(img, cv::Point2f(p2d[0], p2d[1]), 1, CV_RGB(255, 0, 0), thickness);
      else if (color_flag == 2)
        cv::circle(img, cv::Point2f(p2d[0], p2d[1]), 1, CV_RGB(0, 0, 255), thickness);
      else if (color_flag == 3)
        cv::circle(img, cv::Point2f(p2d[0], p2d[1]), 1, purple, thickness);
    }
    // write id to image
    // cv::putText(img, to_string(id), cv::Point2f(seg[0], seg[1]), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1);
    id++;
  }
}

int fileNameFilter_png(const struct dirent *cur)
{
  std::string str(cur->d_name);
  if (str.find(".png") != std::string::npos)
  {
    return 1;
  }
  return 0;
}
int fileNameFilter_jpg(const struct dirent *cur)
{
  std::string str(cur->d_name);
  if (str.find(".jpg") != std::string::npos)
  {
    return 1;
  }
  return 0;
}

upm::Segments removeSegment(upm::Segments &segs, camodocal::CameraPtr m_camera)
{
  cout << "segs_raw size: " << segs.size() << endl;
  upm::Segments segs_new;
  double center_x = 640.991301;
  double center_y = 490.937512;
  double max_r = 512;
  double min_r = 180;
  vector<int> status;
  status.resize(segs.size());
  int seg_num_cnt = 0;
  for (const upm::Segment &seg : segs)
  {

    Eigen::Vector2d p1(seg[0], seg[1]);
    Eigen::Vector2d p2(seg[2], seg[3]);

    // double distance_center_p1 = sqrt((p1[0] - center_x) * (p1[0] - center_x) + (p1[1] - center_y) * (p1[1] - center_y));
    // double distance_center_p2 = sqrt((p2[0] - center_x) * (p2[0] - center_x) + (p2[1] - center_y) * (p2[1] - center_y));

    // if (distance_center_p1 > max_r || distance_center_p1 < min_r || distance_center_p2 > max_r || distance_center_p2 < min_r)
    // {
    //   status[seg_num_cnt] = 1;
    // }

    Eigen::Vector3d p1_3d, p2_3d;
    // m_camera->liftProjective(p1, p1_3d);
    // m_camera->liftProjective(p2, p2_3d);
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    double p1_3d_dot_p2_3d = p1_3d.dot(p2_3d);
    if (p1_3d_dot_p2_3d > 1.0)
    {
      p1_3d_dot_p2_3d = 1.0;
    }
    else if (p1_3d_dot_p2_3d < -1.0)
    {
      p1_3d_dot_p2_3d = -1.0;
    }
    double theta = acos(p1_3d_dot_p2_3d);

    if (abs(theta) < 5.0 / 180 * M_PI)
    {
      status[seg_num_cnt] = 1;
    }
    seg_num_cnt++;
  }
  for (int i = 0; i < status.size(); i++)
  {
    if (status[i] == 0)
    {
      segs_new.push_back(segs[i]);
    }
  }
  cout << "segs_new size: " << segs_new.size() << endl;
  return segs_new;
}

vector<vector<Eigen::Vector2d>> getPointFromSegments(upm::Segments &segs, camodocal::CameraPtr m_camera)
{
  vector<vector<Eigen::Vector2d>> points_set;
  for (const upm::Segment &seg : segs)
  {
    Eigen::Vector2d p1(seg[0], seg[1]);
    Eigen::Vector2d p2(seg[2], seg[3]);
    Eigen::Vector3d p1_3d, p2_3d;
    // m_camera->liftProjective(p1, p1_3d);
    // m_camera->liftProjective(p2, p2_3d);
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    double theta = acos(p1_3d.dot(p2_3d));
    Eigen::Vector3d axis = p1_3d.cross(p2_3d);
    axis.normalize();
    Eigen::Vector2d middle_p;
    Eigen::AngleAxisd aa(theta / 2, axis);
    Eigen::Vector3d middle_p_3d = aa * p1_3d;
    // m_camera->spaceToPlane(middle_p_3d, middle_p);
    eq_spaceToPlane(middle_p_3d, middle_p);
    vector<Eigen::Vector2d> points;
    points.push_back(p1);
    points.push_back(middle_p);
    points.push_back(p2);
    points_set.push_back(points);
  }
  // cout << "points_set size: " << points_set.size() << endl;
  return points_set;
}

void drawPointsSet(cv::Mat &img, std::vector<vector<Eigen::Vector2d>> points_set)
{
  // orange
  cv::Scalar orange(0, 165, 255);
  cv::Scalar blue(255, 0, 0);
  cv::Scalar green(0, 255, 0);
  cv::Scalar red(0, 0, 255);
  cv::Scalar yellow(0, 255, 255);
  cv::Scalar purple(255, 0, 255);
  for (int i = 0; i < points_set.size(); i++)
  {
    for (int j = 0; j < points_set[i].size(); j++)
    {
      if (j == 0 || j == 2)
      {
        // cv::circle(img, cv::Point2f(points_set[i][j][0], points_set[i][j][1]), 3, orange, 3);
      }
      else
      {
        // cv::circle(img, cv::Point2f(points_set[i][j][0], points_set[i][j][1]), 3, purple, 3);
      }
    }
  }
}

void segments2Curves(upm::Segments &segs, camodocal::CameraPtr m_camera, std::vector<Curve> &Curves)
{
  Curves.clear();
  double delta_theta = 1.0 / 180 * M_PI;
  for (const upm::Segment &seg : segs)
  {

    Eigen::Vector2d p1(seg[0], seg[1]);
    Eigen::Vector2d p2(seg[2], seg[3]);
    // cout << "p1: " << p1 << endl;
    // cout << "p2: " << p2 << endl;
    Eigen::Vector3d p1_3d, p2_3d;
    // m_camera->liftProjective(p1, p1_3d);
    // m_camera->liftProjective(p2, p2_3d);
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    // cout << "p1_3d: " << p1_3d << endl;
    // cout << "p2_3d: " << p2_3d << endl;
    // cout << "p1_3d.dot(p2_3d): " << p1_3d.dot(p2_3d) << endl;

    double p1_3d_dot_p2_3d = p1_3d.dot(p2_3d);
    if (p1_3d_dot_p2_3d > 1.0)
    {
      p1_3d_dot_p2_3d = 1.0;
    }
    else if (p1_3d_dot_p2_3d < -1.0)
    {
      p1_3d_dot_p2_3d = -1.0;
    }
    double theta = acos(p1_3d_dot_p2_3d);
    Eigen::Vector3d axis = p1_3d.cross(p2_3d);
    axis.normalize();
    Curve curve;
    int numKeyPoints = theta / delta_theta;
    // cout << "theta: " << theta << endl;
    // cout << "numKeyPoints: " << numKeyPoints << endl;
    if (numKeyPoints == 0)
    {
      continue;
    }
    for (int i = 0; i <= numKeyPoints; i++)
    {
      Eigen::AngleAxisd aa(i * delta_theta, axis);
      Eigen::Vector3d p_3d = aa * p1_3d;
      Eigen::Vector2d p;
      // m_camera->spaceToPlane(p_3d, p);
      eq_spaceToPlane(p_3d, p);
      cv::Point2f vKey(p[0], p[1]);
      curve.vKeys.push_back(vKey);
    }
    curve.numKeyP = numKeyPoints + 1;
    curve.Length = theta;
    Curves.push_back(curve);
  }
}

void GetremapMat2(cv::Mat &remap_x, cv::Mat &remap_y, int rows, int cols)
{

  double lon;
  double lat;
  camodocal::CameraPtr m_camera;
  std::string config_file;
  config_file = "../config/mindvision.yaml";
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
  for (int i = 0; i < cols; i++)
  {
    for (int j = 0; j < rows; j++)
    {
      lon = (i / double(cols) - 0.5) * (2 * M_PI);
      lat = -(j / double(rows) - 0.5) * M_PI;

      Eigen::Vector3d tmp_p;
      Eigen::Vector2d tmp_s;

      tmp_p(2) = sin(lat);
      tmp_p(0) = cos(lat) * sin(lon);
      tmp_p(1) = cos(lat) * cos(lon);
      m_camera->spaceToPlane(tmp_p, tmp_s);

      // if(tmp_s(0)>0&&tmp_s(1)>0&& tmp_s(1)<remap_x.rows&&tmp_s(0)<remap_x.cols)
      // {
      // result.at<uchar>(j,i)=img.at<uchar>(tmp_s(1),tmp_s(0));
      remap_x.at<float>(j, i) = tmp_s(0);
      remap_y.at<float>(j, i) = tmp_s(1);
      // }
    }
  }
}

void GetremapMat(cv::Mat &remap_x, cv::Mat &remap_y, int rows, int cols)
{

  double lon;
  double lat;
  camodocal::CameraPtr m_camera;
  std::string config_file;
  config_file = "../config/mindvision.yaml";
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

  double left_sphere_up = M_PI / 180 * 90;
  double left_sphere_down = M_PI / 180 * 90;

  for (int i = 0; i < cols; i++)
  {
    for (int j = 0; j < rows; j++)
    {
      lon = (i / double(cols) - 0.5) * (2 * M_PI);
      // lat=-(j/double(rows)-0.5)*M_PI/3;       //   j/double(rows)-0.5 = (0,1)
      lat = -(j / double(rows) * (left_sphere_up + left_sphere_down) - left_sphere_up);

      Eigen::Vector3d tmp_p;
      Eigen::Vector2d tmp_s;

      tmp_p(2) = sin(lat);
      tmp_p(0) = cos(lat) * sin(lon);
      tmp_p(1) = cos(lat) * cos(lon);
      m_camera->spaceToPlane(tmp_p, tmp_s);

      remap_x.at<float>(j, i) = tmp_s(0);
      remap_y.at<float>(j, i) = tmp_s(1);
    }
  }
}

void GetrotationremapMat(cv::Mat &remap_x, cv::Mat &remap_y, int rows, int cols)
{

  double lon;
  double lat;

  double left_sphere_up = M_PI / 180 * 90;
  double left_sphere_down = M_PI / 180 * 90;

  for (int i = 0; i < cols; i++)
  {
    for (int j = 0; j < rows; j++)
    {
      lon = (i / double(cols) - 0.5) * (2 * M_PI);
      // lat=-(j/double(rows)-0.5)*M_PI/3;       //   j/double(rows)-0.5 = (0,1)
      lat = -(j / double(rows) * (left_sphere_up + left_sphere_down) - left_sphere_up);

      Eigen::Vector3d tmp_p;
      Eigen::Vector2d tmp_s;

      tmp_p(2) = sin(lat);
      tmp_p(0) = cos(lat) * sin(lon);
      tmp_p(1) = cos(lat) * cos(lon);

      // tmp_p = R * tmp_p;
      tmp_p = R1 * R * tmp_p;

      eq_spaceToPlane(tmp_p, tmp_s);

      remap_x.at<float>(j, i) = tmp_s(0);
      remap_y.at<float>(j, i) = tmp_s(1);
    }
  }
}
void SegmentRotated(upm::Segments &segs, upm::Segments &segs_rotated)
{
  for (const upm::Segment &seg : segs)
  {
    Eigen::Vector2d p1(seg[0], seg[1]);
    Eigen::Vector2d p2(seg[2], seg[3]);
    Eigen::Vector3d p1_3d, p2_3d;
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    p1_3d = (R1 * R).transpose() * p1_3d;
    p2_3d = (R1 * R).transpose() * p2_3d;
    p1_3d.normalize();
    p2_3d.normalize();
    Eigen::Vector2d p1_rotated, p2_rotated;
    eq_spaceToPlane(p1_3d, p1_rotated);
    eq_spaceToPlane(p2_3d, p2_rotated);
    upm::Segment seg_rotated;
    seg_rotated[0] = p1_rotated(0);
    seg_rotated[1] = p1_rotated(1);
    seg_rotated[2] = p2_rotated(0);
    seg_rotated[3] = p2_rotated(1);
    segs_rotated.push_back(seg_rotated);
  }
}

// cmp函数
// bool cmp(const std::pair<double, upm::Segment> &a, const std::pair<double, upm::Segment> &b)
// {
//   return a.first > b.first;
// }

bool cmp(const std::pair<double, std::pair<int, upm::Segment>> &a, const std::pair<double, std::pair<int, upm::Segment>> &b)
{
  return a.first > b.first;
}

Eigen::Vector2d get_d_orth_Rep_LE(upm::Segments &segs, upm::Segments &segs2)
{
  int Repeat = 0;
  double Repeat_rate = 0;
  double LE_num = 0;
  double LE_error_sum = 0;
  int num_segs = segs.size();
  int num_segs2 = segs2.size();
  match1.clear();
  match2.clear();
  match1.resize(num_segs);
  for (int i = 0; i < num_segs; i++)
  {
    match1[i] = -1;
  }
  match2.resize(num_segs2);
  for (int i = 0; i < num_segs2; i++)
  {
    match2[i] = -1;
  }
  if (num_segs == 0 || num_segs == 0)
  {
    cout << "no segs!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    return Eigen::Vector2d(0, 0);
  }
  vector<Eigen::Vector3d> segs_fit, segs_fit2;
  // segs fit
  for (int i = 0; i < num_segs; i++)
  {
    Eigen::Vector2d p1(segs[i][0], segs[i][1]);
    Eigen::Vector2d p2(segs[i][2], segs[i][3]);
    Eigen::Vector3d p1_3d, p2_3d;
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    Eigen::Vector3d rotation_vec;
    rotation_vec = p1_3d.cross(p2_3d);
    rotation_vec.normalize();
    segs_fit.push_back(rotation_vec);
  }
  // segs2 fit
  for (int i = 0; i < num_segs2; i++)
  {
    Eigen::Vector2d p1(segs2[i][0], segs2[i][1]);
    Eigen::Vector2d p2(segs2[i][2], segs2[i][3]);
    Eigen::Vector3d p1_3d, p2_3d;
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    Eigen::Vector3d rotation_vec;
    rotation_vec = p1_3d.cross(p2_3d);
    rotation_vec.normalize();
    segs_fit2.push_back(rotation_vec);
  }
  // upm::Segments segs2_tmp;
  // vector<std::pair<double, upm::Segment>> abscos_segs2_tmp;
  vector<std::pair<double, std::pair<int, upm::Segment>>> abscos_segs2_tmp;
  // 1degree
  double cos_theta_th = cos(1.0 * M_PI / 180);

  for (int i = 0; i < num_segs; i++)
  {
    Eigen::Vector3d seg_fit = segs_fit[i];
    abscos_segs2_tmp.clear();

    for (int j = 0; j < num_segs2; j++)
    {
      Eigen::Vector3d seg_fit2 = segs_fit2[j];
      double cos_theta = seg_fit.dot(seg_fit2);
      if (cos_theta > cos_theta_th)
      {
        // abscos_segs2_tmp.push_back(make_pair(cos_theta, segs2[j]));
        abscos_segs2_tmp.push_back(make_pair(cos_theta, make_pair(j, segs2[j])));
      }
      if (cos_theta < -cos_theta_th)
      {
        Eigen::Vector2d p1(segs2[j][2], segs2[j][3]);
        Eigen::Vector2d p2(segs2[j][0], segs2[j][1]);
        upm::Segment seg_tmp;
        seg_tmp[0] = p1(0);
        seg_tmp[1] = p1(1);
        seg_tmp[2] = p2(0);
        seg_tmp[3] = p2(1);
        abscos_segs2_tmp.push_back(make_pair(cos_theta, make_pair(j, seg_tmp)));
      }
    }
    // 从大到小排序
    sort(abscos_segs2_tmp.begin(), abscos_segs2_tmp.end(), cmp);
    int flag = 2;
    int flag2 = 2;
    double theta_seg = 0;
    double theta_seg2 = 0;
    double overlap_theta = 0;
    double overlap = 0;
    double overlap_theshold = 0.5;

    for (int k = 0; k < abscos_segs2_tmp.size(); k++)
    {
      upm::Segment seg1 = segs[i];
      upm::Segment seg2 = abscos_segs2_tmp[k].second.second;
      Eigen::Vector2d p1(seg1[0], seg1[1]);
      Eigen::Vector2d p2(seg1[2], seg1[3]);
      Eigen::Vector2d p3(seg2[0], seg2[1]);
      Eigen::Vector2d p4(seg2[2], seg2[3]);
      Eigen::Vector3d p1_3d, p2_3d, p3_3d, p4_3d;
      eq_liftProjective(p1, p1_3d);
      p1_3d.normalize();
      eq_liftProjective(p2, p2_3d);
      p2_3d.normalize();
      eq_liftProjective(p3, p3_3d);
      p3_3d.normalize();
      eq_liftProjective(p4, p4_3d);
      p4_3d.normalize();
      theta_seg = acos(p1_3d.dot(p2_3d));
      theta_seg2 = acos(p3_3d.dot(p4_3d));
      double dot1_3 = segs_fit[i].dot((p1_3d.cross(p3_3d)));
      double dot2_4 = segs_fit[i].dot((p2_3d.cross(p4_3d)));
      if (dot1_3 > 0)
      {
        flag = 1;
      }
      else
      {
        flag = 0;
      }
      if (dot2_4 > 0)
      {
        flag2 = 1;
      }
      else
      {
        flag2 = 0;
      }
      if (flag == 1 && flag2 == 1)
      {
        overlap_theta = acos(p2_3d.dot(p3_3d));
        // overlap = overlap_theta / max(theta_seg, theta_seg2);
        overlap = overlap_theta / min(theta_seg, theta_seg2);
        if (overlap > overlap_theshold)
        {
          match1[i] = 1;
          match2[abscos_segs2_tmp[k].second.first] = 1;
          Repeat++;
          LE_num++;
          Eigen::Vector3d normal_vec_seg2 = p3_3d.cross(p4_3d);
          Eigen::Vector3d nearest_p1_3d = normal_vec_seg2.cross(p1_3d.cross(normal_vec_seg2));
          nearest_p1_3d.normalize();
          Eigen::Vector2d nearest_p1_2d;
          eq_spaceToPlane(nearest_p1_3d, nearest_p1_2d);
          double L1_p1 = sqrt(pow(nearest_p1_2d(0) - p1(0), 2) + pow(nearest_p1_2d(1) - p1(1), 2));

          Eigen::Vector3d nearest_p2_3d = normal_vec_seg2.cross(p2_3d.cross(normal_vec_seg2));
          nearest_p2_3d.normalize();
          Eigen::Vector2d nearest_p2_2d;
          eq_spaceToPlane(nearest_p2_3d, nearest_p2_2d);
          double L1_p2 = sqrt(pow(nearest_p2_2d(0) - p2(0), 2) + pow(nearest_p2_2d(1) - p2(1), 2));
          double LE_error = (L1_p1 + L1_p2) / 2;
          LE_error_sum += LE_error;
          break;
        }
      }
      if (flag == 0 && flag2 == 0)
      {
        overlap_theta = acos(p1_3d.dot(p4_3d));
        // overlap = overlap_theta / max(theta_seg, theta_seg2);
        overlap = overlap_theta / min(theta_seg, theta_seg2);
        if (overlap > overlap_theshold)
        {
          match1[i] = 1;
          match2[abscos_segs2_tmp[k].second.first] = 1;
          Repeat++;
          LE_num++;
          Eigen::Vector3d normal_vec_seg2 = p3_3d.cross(p4_3d);
          Eigen::Vector3d nearest_p1_3d = normal_vec_seg2.cross(p1_3d.cross(normal_vec_seg2));
          nearest_p1_3d.normalize();
          Eigen::Vector2d nearest_p1_2d;
          eq_spaceToPlane(nearest_p1_3d, nearest_p1_2d);
          double L1_p1 = sqrt(pow(nearest_p1_2d(0) - p1(0), 2) + pow(nearest_p1_2d(1) - p1(1), 2));

          Eigen::Vector3d nearest_p2_3d = normal_vec_seg2.cross(p2_3d.cross(normal_vec_seg2));
          nearest_p2_3d.normalize();
          Eigen::Vector2d nearest_p2_2d;
          eq_spaceToPlane(nearest_p2_3d, nearest_p2_2d);
          double L1_p2 = sqrt(pow(nearest_p2_2d(0) - p2(0), 2) + pow(nearest_p2_2d(1) - p2(1), 2));
          double LE_error = (L1_p1 + L1_p2) / 2;
          LE_error_sum += LE_error;
          break;
        }
      }
      if (flag == 1 && flag2 == 0)
      {
        overlap_theta = acos(p3_3d.dot(p4_3d));
        // overlap = overlap_theta / max(theta_seg, theta_seg2);
        overlap = overlap_theta / min(theta_seg, theta_seg2);
        if (overlap > overlap_theshold)
        {
          match1[i] = 1;
          match2[abscos_segs2_tmp[k].second.first] = 1;
          Repeat++;
          LE_num++;
          Eigen::Vector3d normal_vec_seg2 = p3_3d.cross(p4_3d);
          Eigen::Vector3d nearest_p1_3d = normal_vec_seg2.cross(p1_3d.cross(normal_vec_seg2));
          nearest_p1_3d.normalize();
          Eigen::Vector2d nearest_p1_2d;
          eq_spaceToPlane(nearest_p1_3d, nearest_p1_2d);
          double L1_p1 = sqrt(pow(nearest_p1_2d(0) - p1(0), 2) + pow(nearest_p1_2d(1) - p1(1), 2));

          Eigen::Vector3d nearest_p2_3d = normal_vec_seg2.cross(p2_3d.cross(normal_vec_seg2));
          nearest_p2_3d.normalize();
          Eigen::Vector2d nearest_p2_2d;
          eq_spaceToPlane(nearest_p2_3d, nearest_p2_2d);
          double L1_p2 = sqrt(pow(nearest_p2_2d(0) - p2(0), 2) + pow(nearest_p2_2d(1) - p2(1), 2));
          double LE_error = (L1_p1 + L1_p2) / 2;
          LE_error_sum += LE_error;
          break;
        }
      }
      if (flag == 0 && flag2 == 1)
      {
        overlap_theta = acos(p1_3d.dot(p2_3d));
        // overlap = overlap_theta / max(theta_seg, theta_seg2);
        overlap = overlap_theta / min(theta_seg, theta_seg2);
        if (overlap > overlap_theshold)
        {
          match1[i] = 1;
          match2[abscos_segs2_tmp[k].second.first] = 1;
          Repeat++;
          LE_num++;
          Eigen::Vector3d normal_vec_seg2 = p3_3d.cross(p4_3d);
          Eigen::Vector3d nearest_p1_3d = normal_vec_seg2.cross(p1_3d.cross(normal_vec_seg2));
          nearest_p1_3d.normalize();
          Eigen::Vector2d nearest_p1_2d;
          eq_spaceToPlane(nearest_p1_3d, nearest_p1_2d);
          double L1_p1 = sqrt(pow(nearest_p1_2d(0) - p1(0), 2) + pow(nearest_p1_2d(1) - p1(1), 2));

          Eigen::Vector3d nearest_p2_3d = normal_vec_seg2.cross(p2_3d.cross(normal_vec_seg2));
          nearest_p2_3d.normalize();
          Eigen::Vector2d nearest_p2_2d;
          eq_spaceToPlane(nearest_p2_3d, nearest_p2_2d);
          double L1_p2 = sqrt(pow(nearest_p2_2d(0) - p2(0), 2) + pow(nearest_p2_2d(1) - p2(1), 2));
          double LE_error = (L1_p1 + L1_p2) / 2;
          LE_error_sum += LE_error;
          break;
        }
      }
    }
  }
  Repeat_rate = static_cast<double>(Repeat) / num_segs;
  // cout << "Repeat_rate:" << Repeat_rate << endl;
  // cout << "Repeat:" << Repeat << endl;
  // cout << "num_segs:" << num_segs << endl;
  Eigen::Vector2d Rep_LE_vec;
  Rep_LE_vec(0) = Repeat_rate;
  if (LE_num != 0)
  {
    Rep_LE_vec(1) = LE_error_sum / LE_num;
  }
  else
  {
    Rep_LE_vec(1) = 0;
  }
  return Rep_LE_vec;
}

bool smp(const std::pair<double, upm::Segment> &a, const std::pair<double, upm::Segment> &b)
{
  return a.first < b.first;
}

Eigen::Vector2d get_d_s_Rep_LE(upm::Segments &segs, upm::Segments &segs2)
{
  int Repeat = 0;
  double Repeat_rate = 0;
  double LE_num = 0;
  double LE_error_sum = 0;
  int num_segs = segs.size();
  int num_segs2 = segs2.size();
  if (num_segs == 0 || num_segs == 0)
  {
    cout << "no segs!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    return Eigen::Vector2d(0, 0);
  }
  vector<Eigen::Vector3d> segs_fit, segs_fit2;
  // segs fit
  for (int i = 0; i < num_segs; i++)
  {
    Eigen::Vector2d p1(segs[i][0], segs[i][1]);
    Eigen::Vector2d p2(segs[i][2], segs[i][3]);
    Eigen::Vector3d p1_3d, p2_3d;
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    Eigen::Vector3d rotation_vec;
    rotation_vec = p1_3d.cross(p2_3d);
    rotation_vec.normalize();
    segs_fit.push_back(rotation_vec);
  }
  // segs2 fit
  for (int i = 0; i < num_segs2; i++)
  {
    Eigen::Vector2d p1(segs2[i][0], segs2[i][1]);
    Eigen::Vector2d p2(segs2[i][2], segs2[i][3]);
    Eigen::Vector3d p1_3d, p2_3d;
    eq_liftProjective(p1, p1_3d);
    eq_liftProjective(p2, p2_3d);
    p1_3d.normalize();
    p2_3d.normalize();
    Eigen::Vector3d rotation_vec;
    rotation_vec = p1_3d.cross(p2_3d);
    rotation_vec.normalize();
    segs_fit2.push_back(rotation_vec);
  }
  vector<std::pair<double, upm::Segment>> pixel_error;
  double pixel_error_theshold = 5;
  for (int i = 0; i < num_segs; i++)
  {
    pixel_error.clear();
    Eigen::Vector3d seg_fit = segs_fit[i];
    for (int j = 0; j < num_segs2; j++)
    {
      Eigen::Vector3d seg_fit2 = segs_fit2[j];
      Eigen::Vector2d p1, p2, p3, p4;
      p1 = Eigen::Vector2d(segs[i][0], segs[i][1]);
      p2 = Eigen::Vector2d(segs[i][2], segs[i][3]);
      if (seg_fit.dot(seg_fit2) > 0)
      {
        p3 = Eigen::Vector2d(segs2[j][0], segs2[j][1]);
        p4 = Eigen::Vector2d(segs2[j][2], segs2[j][3]);
      }
      else
      {
        p3 = Eigen::Vector2d(segs2[j][2], segs2[j][3]);
        p4 = Eigen::Vector2d(segs2[j][0], segs2[j][1]);
      }
      double L1_p1_p3 = sqrt(pow(p1(0) - p3(0), 2) + pow(p1(1) - p3(1), 2));
      double L1_p2_p4 = sqrt(pow(p2(0) - p4(0), 2) + pow(p2(1) - p4(1), 2));
      if (L1_p1_p3 < pixel_error_theshold && L1_p2_p4 < pixel_error_theshold)
      // if ((L1_p1_p3 + L1_p2_p4) / 2 < pixel_error_theshold)
      {
        // p3 p4
        upm::Segment seg;
        seg[0] = p3(0);
        seg[1] = p3(1);
        pixel_error.push_back(std::make_pair((L1_p1_p3 + L1_p2_p4) / 2, seg));
      }
    }
    sort(pixel_error.begin(), pixel_error.end(), smp);
    // if (pixel_error.size() >= 2)
    // {
    //   cout << "pixel_error.size():" << pixel_error.size() << endl;
    //   for (int k = 0; k < pixel_error.size(); k++)
    //   {
    //     cout << pixel_error[k].first << endl;
    //   }
    // }
    if (pixel_error.size() > 0)
    {
      Repeat++;
      LE_error_sum += pixel_error[0].first;
      LE_num++;
    }
  }
  Repeat_rate = static_cast<double>(Repeat) / num_segs;
  // cout << "Repeat_rate:" << Repeat_rate << endl;
  // cout << "Repeat:" << Repeat << endl;
  // cout << "num_segs:" << num_segs << endl;
  Eigen::Vector2d Rep_LE_vec;
  Rep_LE_vec(0) = Repeat_rate;
  if (LE_num != 0)
  {
    Rep_LE_vec(1) = LE_error_sum / LE_num;
  }
  else
  {
    Rep_LE_vec(1) = 0;
  }
  return Rep_LE_vec;
}

int main()
{

  std::cout << "******************************************************" << std::endl;
  std::cout << "******************* ELSED main demo ******************" << std::endl;
  std::cout << "******************************************************" << std::endl;

  //! for test
  R = Eigen::AngleAxisd(M_PI * 2 / 3, Eigen::Vector3d::UnitZ());
  R1 = Eigen::AngleAxisd(M_PI * 2 / 3, Eigen::Vector3d::UnitX());

  cv::Mat img = cv::imread("../test_img/pano_aicubdhvkwkwwq.jpg");
  if (img.empty())
  {
    std::cerr << "Error reading input image" << std::endl;
    return -1;
  }
  int rows = img.rows;
  int cols = img.cols;
  int new_rows = 512;
  int new_cols = 1024;

  static cv::Mat remap_x = cv::Mat::zeros(new_rows, new_cols, CV_32FC1);
  static cv::Mat remap_y = cv::Mat::zeros(new_rows, new_cols, CV_32FC1);

  GetremapMat(remap_x, remap_y, new_rows, new_cols);

  GetrotationremapMat(remap_x, remap_y, new_rows, new_cols);

  cv::remap(img, img, remap_x, remap_y, 0);

  auto start = std::chrono::steady_clock::now();
  upm::ELSED elsed;
  upm::Segments segs = elsed.detect(img);
  std::cout << "ELSED detected: " << segs.size() << " (large) segments" << std::endl;
  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "time: " << double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << " s" << std::endl;

  camodocal::CameraPtr m_camera;
  std::string config_file;
  config_file = "../config/mindvision.yaml";
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

  drawCurveSegments(img, segs, CV_RGB(0, 255, 0), m_camera);
  cv::imshow("ELSED long", img);
  cv::waitKey(0);

  img = cv::imread("../test_img/pano_aicubdhvkwkwwq.jpg");
  if (img.empty())
  {
    std::cerr << "Error reading input image" << std::endl;
    return -1;
  }

  segs = elsed.detect(img);
  drawCurveSegments(img, segs, CV_RGB(0, 255, 0), m_camera);

  //! added by wz
  struct dirent **up_imagelist;
  // std::string up_image_file_name = "/home/wt/LF-VIO/line_detection/Outdoor/drive-download-20230504T130542Z-001/CVRG-Pano/test/rgb/";
  std::string up_image_file_name = "../test_img/";
  int up_image_data_num = scandir(up_image_file_name.c_str(), &up_imagelist, fileNameFilter_png, alphasort);
  int first_flag = 1;
  int first_des_flag = 1;
  cv::Mat prev_img, cur_img;
  cv::Mat prev_des, cur_des;
  upm::Segments prev_segs, cur_segs;
  vector<double> Rep;
  Rep.reserve(up_image_data_num);
  double d_orth_Rep_sum = 0;
  double d_orth_LE_sum = 0;
  double d_s_Rep_sum = 0;
  double d_s_LE_sum = 0;
  double Time_sum = 0;
  double Seg_num_sum = 0;

  for (auto i = 0; i < up_image_data_num; i++)
  {
    std::string up_image_file_name_jpg = up_image_file_name + up_imagelist[i]->d_name;
    cv::Mat img = cv::imread(up_image_file_name_jpg);
    cv::resize(img, img, cv::Size(1024, 512));
    cv::Mat show_img;
    show_img = img.clone();
    upm::ELSED elsed;
    auto start = std::chrono::steady_clock::now();
    upm::Segments segs = elsed.detect(img);
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    Time_sum += double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    std::cout << "ELSED detected: " << segs.size() << " (large) segments" << std::endl;
    Seg_num_sum += segs.size();
    vector<vector<Eigen::Vector2d>> points_set = getPointFromSegments(segs, m_camera);
    drawPointsSet(show_img, points_set);
    cv::Mat img_rotated;
    cv::remap(img, img_rotated, remap_x, remap_y, 0);
    cv::Mat show_img_rotated;
    show_img_rotated = img_rotated.clone();
    upm::Segments segs_rotated = elsed.detect(show_img_rotated);
    upm::Segments segs_1_2;
    SegmentRotated(segs, segs_1_2);

    Eigen::Vector2d d_orth_Rep_LE = get_d_orth_Rep_LE(segs_1_2, segs_rotated);
    Eigen::Vector2d d_s_Rep_LE = get_d_s_Rep_LE(segs_1_2, segs_rotated);

    Rep.push_back(d_orth_Rep_LE(0));

    drawCurveSegments_match(show_img, segs, CV_RGB(0, 255, 0), match1, m_camera, 1);

    drawCurveSegments_match(show_img_rotated, segs_rotated, CV_RGB(0, 255, 0), match2, m_camera, 1);

    cv::imshow("ELSED", show_img);
    cv::imshow("ELSED rotated", show_img_rotated);
    cv::waitKey(0);
  }

  return 0;
}