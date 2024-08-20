#pragma once
#include "iostream"
#include "fstream"
#include <ctime>
#include <vector>
#include <list>
#include <opencv2/line_descriptor.hpp>

using namespace cv::line_descriptor;

typedef enum _ORIENT_CODE
{
    ORIENT_HORIZONTAL = 1, // horizontal
    ORIENT_VERTICAL = 2,   // vertical

} ORIENT_CODE;

typedef struct image_int8u_s
{
    unsigned char *data;
    unsigned int xsize, ysize;
} *image_int8u_p;

typedef struct image_int16s_s
{
    short *data;
    unsigned int xsize, ysize;
} *image_int16s_p;

class Curve
{
public:
    unsigned int numKeyP;
    float Length;
    int level;
    cv::Point2f dir;
    std::vector<cv::Point2f> vKeys;

    Curve() : Length(0.f), numKeyP(0), level(0) {}
    Curve(std::vector<cv::Point2f> _vKeys) : Length(0.f), numKeyP(_vKeys.size()), vKeys(_vKeys) {}

    bool operator<(const Curve &c) const
    {
        return this->Length < c.Length;
    }

    bool operator>(const Curve &c) const
    {
        return this->Length > c.Length;
    }
};

class ELESDMatch
{
public:
    ELESDMatch();

    int CalDescriptor(const cv::Mat &gray);

    std::vector<cv::line_descriptor::KeyLine> vKeyLines_;
    // store the descriptors
    cv::Mat descriptors_;

    void initvKeyLines();

    void initImg(const cv::Mat &gray);

    bool NeedAjustDirection(Curve &curve);

    std::vector<Curve> curves;

    image_int16s_p dxImg_;

    image_int16s_p dyImg_;

    int imageWidth, imageHeight;

    cv::Mat grayImg_;
};
