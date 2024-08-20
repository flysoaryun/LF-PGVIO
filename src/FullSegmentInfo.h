#ifndef ELSED_FULLSEGMENTINFO_H_
#define ELSED_FULLSEGMENTINFO_H_

#include <ostream>
#include <memory>
#include "Utils.h"

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

namespace upm
{

  /**
   * This class represents a line segment and its associated information such as:
   * - List of supporting pixels
   * - Line equation
   * - Endpoints
   *
   * The implementation is highly optimized to add or subtract new pixels and recompute all parameters.
   * Internally,
   */
  class FullSegmentInfo
  {
  public:
    // Creates the FullSegmentInfo object but do not initialize internal structures
    FullSegmentInfo(const std::vector<Pixel> &pts);
    // Creates the FullSegmentInfo object and initializes internal structures
    FullSegmentInfo(const std::vector<Pixel> &pts, int startIdx);
    // Initialize the internal data structure from scratch
    void init(const std::vector<Pixel> &pts, int startIdx);
    // Checks if the point (x, y) is an inlier given the current line equation and a distance th.
    // TODO (check)
    // inline bool isInlier(int x, int y, double lineFitErrThreshold)
    // {
    //   const double pointToLineDis = equation[0] * x + equation[1] * y + equation[2];
    //   return UPM_ABS(pointToLineDis) < lineFitErrThreshold;
    // }
    //! added by wz
    // inline bool isInlier(int x, int y, double sphereFitErrThreshold)
    // {
    //   Eigen::Vector3d tmp_p;
    //   m_camera->liftProjective(Eigen::Vector2d(x, y), tmp_p);
    //   // std::cout << "tmp_p!!!!!!!!!!!!: " << tmp_p << std::endl;
    //   // tmp_p[0] = visionMap.at<cv::Vec3f>(y, x)[0];
    //   // tmp_p[1] = visionMap.at<cv::Vec3f>(y, x)[1];
    //   // tmp_p[2] = visionMap.at<cv::Vec3f>(y, x)[2];
    //   // std::cout << "tmp_p: " << tmp_p << std::endl;
    //   tmp_p.normalize();
    //   Eigen ::Vector3d equation_unit_shpere;

    //   if (isHorizontal)
    //   {
    //     equation_unit_shpere[0] = equation[0];
    //     equation_unit_shpere[1] = equation[1];
    //     equation_unit_shpere[2] = equation[2];
    //   }
    //   else
    //   {
    //     equation_unit_shpere[0] = equation[1];
    //     equation_unit_shpere[1] = equation[0];
    //     equation_unit_shpere[2] = equation[2];
    //   }

    //   double cos_theta = abs(tmp_p.dot(equation_unit_shpere));
    //   // cos(87deg) =  0.0523359;
    //   // sphereFitErrThreshold = 0.0523359;
    //   // cos(87deg) =  0.0523359;
    //   // sphereFitErrThreshold = 0.0523359;
    //   // cos(88deg) =  0.0348995;
    //   // sphereFitErrThreshold = 0.0348995;
    //   // cos(89deg) =  0.0174524;
    //   // sphereFitErrThreshold = 0.0;
    //   // cos(89.5deg) =  0.00999983;
    //   sphereFitErrThreshold = 0.00999983;
    //   // if (cos_theta < sphereFitErrThreshold)
    //   if (cos_theta < 0.1)
    //   {
    //     // std::cout << "inlier!!!!!!!!!!!!" << std::endl;
    //     // std::cout << "cos_theta: " << cos_theta << std::endl;
    //     return true;
    //   }
    //   else
    //   {
    //     // std::cout << "outlier!!!!!!!!!!!!" << std::endl;
    //     // std::cout << "cos_theta: " << cos_theta << std::endl;
    //     return false;
    //   }
    // }
    void eq_liftProjective(Eigen::Vector2d p2d, Eigen::Vector3d &p3d) const
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

    void eq_spaceToPlane(Eigen::Vector3d p3d, Eigen::Vector2d &p2d) const
    {
      double lon;
      double lat;
      double left_sphere_up = M_PI / 180 * 90;
      double left_sphere_down = M_PI / 180 * 90;
      int new_rows = 512;
      int new_cols = 1024;
      double x = p3d(0);
      double y = p3d(1);
      double z = p3d(2);
      lon = atan2(x, y);
      lat = asin(z);
      Eigen::Vector2d tmp_p;
      tmp_p(0) = (lon / (2 * M_PI) + 0.5) * double(new_cols);
      tmp_p(1) = (-lat + left_sphere_up) / (left_sphere_up + left_sphere_down) * double(new_rows);
      p2d = tmp_p;
    }

    inline bool isInlier(int x, int y, double lineFitErrThreshold)
    {
      Eigen::Vector3d tmp_p, tmp_p_vertical;
      Eigen::Vector2d tmp_vertical;
      // m_camera->liftProjective(Eigen::Vector2d(x, y), tmp_p);
      eq_liftProjective(Eigen::Vector2d(x, y), tmp_p);
      tmp_p.normalize();

      tmp_p_vertical = equation_unit_shpere.cross(tmp_p.cross(equation_unit_shpere).normalized());
      tmp_p_vertical.normalize();
      // m_camera->spaceToPlane(tmp_p_vertical, tmp_vertical);
      eq_spaceToPlane(tmp_p_vertical, tmp_vertical);
      const double pointToLineDis = sqrt(pow(tmp_vertical[0] - x, 2) + pow(tmp_vertical[1] - y, 2));

      const double pointToLineDis_2 = UPM_ABS(equation[0] * x + equation[1] * y + equation[2]);
      double weight = 1.0;
      const double pointToLineDis_all = weight * pointToLineDis + (1 - weight) * pointToLineDis_2;

      return UPM_ABS(pointToLineDis_all) < lineFitErrThreshold;
    }

    // Adds a new pixel (x, y) to the segment and update its parameters.
    // pixelIndexInEdge is the index of the pixel in the edge structure.
    // isPixelAtTheEnd True if the pixel is at the end of the segment or false if it is at the start
    void addPixel(int x, int y, int pixelIndexInEdge, bool isPixelAtTheEnd = true);

    // Finishes the detection of segment.
    void finish();

    void skipPositions();
    // TODO (check)
    // inline double getFitError() const
    // {
    //   double dist, fitError = 0;
    //   for (int i = firstPxIndex; i <= lastPxIndex; i++)
    //   {
    //     dist = equation[0] * (*pixels)[i].x + equation[1] * (*pixels)[i].y + equation[2];
    //     fitError += dist * dist;
    //   }
    //   return fitError / N;
    // }
    //! added by wz
    // inline double getFitError() const
    // {
    //   double dist, fitError = 0;
    //   for (int i = firstPxIndex; i <= lastPxIndex; i++)
    //   {
    //     Eigen::Vector3d tmp_p;
    //     m_camera->liftProjective(Eigen::Vector2d((*pixels)[i].x, (*pixels)[i].y), tmp_p);
    //     // tmp_p[0] = visionMap.at<cv::Vec3f>((*pixels)[i].y, (*pixels)[i].x)[0];
    //     // tmp_p[1] = visionMap.at<cv::Vec3f>((*pixels)[i].y, (*pixels)[i].x)[1];
    //     // tmp_p[2] = visionMap.at<cv::Vec3f>((*pixels)[i].y, (*pixels)[i].x)[2];
    //     tmp_p.normalize();
    //     Eigen ::Vector3d equation_unit_shpere;
    //     equation_unit_shpere[0] = equation[0];
    //     equation_unit_shpere[1] = equation[1];
    //     equation_unit_shpere[2] = equation[2];
    //     dist = tmp_p.dot(equation_unit_shpere);
    //     // std::cout << "tmp_p: " << tmp_p << std::endl;
    //     // std::cout << "equation_unit_shpere: " << equation_unit_shpere << std::endl;
    //     fitError += dist * dist;
    //   }
    //   // std::cout << "fitError: " << fitError << std::endl;
    //   // fitError = 0;
    //   return fitError / N;
    // }

    //! added by wz
    inline double getFitError() const
    {
      double dist, dist2, fitError = 0;
      for (int i = firstPxIndex; i <= lastPxIndex; i++)
      {
        Eigen::Vector3d tmp_p, tmp_p_vertical;
        Eigen::Vector2d tmp_vertical;
        // m_camera->liftProjective(Eigen::Vector2d((*pixels)[i].x, (*pixels)[i].y), tmp_p);
        eq_liftProjective(Eigen::Vector2d((*pixels)[i].x, (*pixels)[i].y), tmp_p);
        tmp_p.normalize();
        // std::cout << "tmp_p: " << tmp_p << std::endl;
        tmp_p_vertical = equation_unit_shpere.cross(tmp_p.cross(equation_unit_shpere).normalized());
        // std::cout << "equation_unit_shpere: " << equation_unit_shpere << std::endl;
        // std::cout << "tmp_p_vertical: " << tmp_p_vertical << std::endl;
        tmp_p_vertical.normalize();
        // m_camera->spaceToPlane(tmp_p_vertical, tmp_vertical);
        eq_spaceToPlane(tmp_p_vertical, tmp_vertical);
        // tmp_p_vertical = tmp_p_vertical + Eigen::Vector3d(0, 0, 0.2);
        // tmp_p_vertical.normalize();
        // std::cout << "tmp_p_vertical2: " << tmp_p_vertical << std::endl;
        dist = pow(tmp_vertical[0] - (*pixels)[i].x, 2) + pow(tmp_vertical[1] - (*pixels)[i].y, 2);
        // dist = 0;
        // std::cout << "tmp_vertical: " << tmp_vertical << std::endl;
        // std::cout << "tmp_vertical0: " << tmp_vertical[0] << std::endl;
        // std::cout << "tmp_vertical1: " << tmp_vertical[1] << std::endl;
        // std::cout << "(*pixels)[i].x: " << (*pixels)[i].x - tmp_vertical[0] << std::endl;
        // std::cout << "(*pixels)[i].y: " << static_cast<double>((*pixels)[i].y - tmp_vertical[1]) << std::endl;
        // dist = tmp_p.dot(equation_unit_shpere);
        // std::cout << "dist: " << dist << std::endl;
        double weight = 1.0;
        fitError += dist * weight;
        // std::cout << "dist: " << dist << std::endl;
        dist2 = pow(equation[0] * (*pixels)[i].x + equation[1] * (*pixels)[i].y + equation[2], 2);
        // std::cout << "dist2: " << dist2 << std::endl;
        fitError += dist2 * (1 - weight);
      }
      return fitError / N;
    }

    void removeLastPx(bool removeFromTheEnd = true);

    inline void reset()
    {
      sum_x_i = 0, sum_y_i = 0, sum_x_i_y_i = 0, sum_x_i_2 = 0, N = 0;
      //! added by wz
      sphere_sum_x_i_2 = 0, sphere_sum_y_i_2 = 0, sphere_sum_z_i_2 = 0, sphere_sum_x_i_y_i = 0, sphere_sum_x_i_z_i = 0, sphere_sum_y_i_z_i = 0;
      firstPxIndex = -1;
      lastPxIndex = -1;
      arePixelsSorted = true;
      firstEndpointExtended = false;
      secondEndpointExtended = false;
    }
    //! changed by wz
    // inline void reset()
    // {
    //   sphere_sum_x_i_2 = 0, sphere_sum_y_i_2 = 0, sphere_sum_z_i_2 = 0, sphere_sum_x_i_y_i = 0, sphere_sum_x_i_z_i = 0, sphere_sum_y_i_z_i = 0;
    //   sum_x_i = 0, sum_y_i = 0, sum_x_i_y_i = 0, sum_x_i_2 = 0, N = 0;
    //   firstPxIndex = -1;
    //   lastPxIndex = -1;
    //   arePixelsSorted = true;
    //   firstEndpointExtended = false;
    //   secondEndpointExtended = false;
    // }

    ///////////////////// Getters and setters /////////////////////

    inline bool horizontal() const { return isHorizontal; }

    inline int getNumOfPixels() const { return N; }

    inline const Segment &getEndpoints() const { return endpoints; }

    ImageEdge getPixels() const;

    // Returns the Pixel in the first extreme of the segment
    inline const Pixel &getFirstPixel() const { return firstPx; }

    // Returns the Pixel in the first extreme of the segment
    inline const Pixel &getLastPixel() const { return lastPx; }

    // Returns a pointer to the first pixel of the segment
    inline const Pixel *begin() const { return &((*pixels)[firstPxIndex]); }

    // Returns a pointer to the position after the last pixel of the segment
    inline const Pixel *end() const { return (&((*pixels)[lastPxIndex])) + 1; }

    inline const cv::Vec3d &getLineEquation() const { return equation; }

    inline bool hasSecondSideElements() const { return !arePixelsSorted; }

    // True if the segment has been already extended in the direction of the first endpoint
    bool firstEndpointExtended = false;
    // Idem for the second one
    bool secondEndpointExtended = false;

    //! added by wz
    camodocal::CameraPtr m_camera;

  private:
    // Fits the line segment from scratch
    void leastSquareLineFit(const std::vector<Pixel> &pts, int startIndex = 0);
    //! added by wz
    void leastSquareSphereLineFit(const std::vector<Pixel> &pts, int startIndex = 0);
    // Adds a new pixel to the internal structures
    void leastSquaresLineFitNewPoint(int x, int y);
    //! added by wz
    void leastSquaresSphereLineFitNewPoint(int x, int y);
    // Subtracts a pixel from the internal structures
    void subtractPointFromModel(const Pixel &p);
    //! added by wz
    void subtractSpherePointFromModel(const Pixel &p);
    // Computes the line equation given the internal parameters (very efficient)
    void calculateLineEq();
    //! added by wz
    void calculateSphereLineEq();
    // Computes the segment endpoints given the internal parameters (very efficient)
    void calcSegmentEndpoints();
    //! added by wz
    void calcSphereSegmentEndpoints();

    int64_t sum_x_i = 0, sum_y_i = 0, sum_x_i_y_i = 0, sum_x_i_2 = 0;
    double sphere_sum_x_i_y_i = 0, sphere_sum_x_i_z_i = 0, sphere_sum_y_i_z_i = 0,
           sphere_sum_x_i_2 = 0, sphere_sum_y_i_2 = 0, sphere_sum_z_i_2 = 0;
    uint32_t N = 0;
    int dx, dy;
    // Tru if the line is mainly horizontal. When the line is closer to vertical,
    // we invert the least squares and use y as independent coordinate and x as dependent coord.
    bool isHorizontal;
    // Pre-computed line segment endpoints
    Segment endpoints;
    // Line equation in format ax + by + c = 0
    cv::Vec3d equation;
    // normal vector of the plane
    // !added by wz
    Eigen::Vector3d equation_unit_shpere;
    // Important pixels that are part of the segment
    Pixel firstPx, prevFirstPx, lastPx, prevLastPx;
    // Indices of some pixels in the chain
    int firstPxIndex = -1, lastPxIndex = -1;
    // Relevance of the segment
    float salience = -1;
    // Pointer to the chain of edge pixels
    const ImageEdge *pixels;
    // Indicates whenever the list of pixels from firstPxIndex to lastPxIndex is sorted
    bool arePixelsSorted = true;

    //! added by wz
    // camodocal::CameraPtr m_camera;
    cv::Mat visionMap;
    int height, width;
  };

}

#endif // ELSED_FULLSEGMENTINFO_H_
