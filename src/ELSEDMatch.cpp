#include "ELSEDMatch.h"
using namespace std;

#ifndef ROUND
#define ROUND (0.5F)
#endif

static const int combinations[32][2] =
    {
        {0, 1},
        {0, 2},
        {0, 3},
        {0, 4},
        {0, 5},
        {0, 6},
        {1, 2},
        {1, 3},
        {1, 4},
        {1, 5},
        {1, 6},
        {2, 3},
        {2, 4},
        {2, 5},
        {2, 6},
        {2, 7},
        {2, 8},
        {3, 4},
        {3, 5},
        {3, 6},
        {3, 7},
        {3, 8},
        {4, 5},
        {4, 6},
        {4, 7},
        {4, 8},
        {5, 6},
        {5, 7},
        {5, 8},
        {6, 7},
        {6, 8},
        {7, 8}};

static inline int get2Pow(int i)
{
    assert(i >= 0 && i <= 7);
    return 1 << i;
}
static unsigned char binaryConversion(float *f1, float *f2)
{
    uchar result = 0;
    for (int i = 0; i < 8; i++)
    {
        if (f1[i] > f2[i])
            result += (uchar)get2Pow(i);
    }

    return result;
}

static int GetLinePixelsNums(float startX, float startY, float endX, float endY)
{
    int num;
    int nstarX, nstartY, nendX, nendY;

    nstarX = (int)(startX + ROUND);
    nstartY = (int)(startY + ROUND);
    nendX = (int)(endX + ROUND);
    nendY = (int)(endY + ROUND);

    num = MAX(abs(nendX - nstarX) + 1, abs(nendY - nstartY) + 1);

    return num;
}

static image_int8u_p new_image_int8u(unsigned int xsize, unsigned int ysize)
{
    image_int8u_p image = NULL;

    /* get memory */
    image = new image_int8u_s[1];
    image->data = new unsigned char[xsize * ysize];

    /* set image size */
    image->xsize = xsize;
    image->ysize = ysize;

    return image;
}

static image_int16s_p new_image_int16s(unsigned int xsize, unsigned int ysize)
{
    image_int16s_p image = NULL;

    /* get memory */
    image = new image_int16s_s[1];
    image->data = new short[xsize * ysize];

    /* set image size */
    image->xsize = xsize;
    image->ysize = ysize;

    return image;
}

static void sobel_edge(ORIENT_CODE oriention, image_int8u_p src, image_int16s_p dst)
{
    unsigned char *psrc = NULL;
    unsigned int nsize;
    unsigned int i, j, _center, offset_up, offset_down;
    unsigned int _tp, _td, _t;

    nsize = src->xsize * src->ysize;

    // no edge processing
    // memset(dst->data, nsize, sizeof(short));

    psrc = src->data;
    // cout << "sobel_edge" << endl;
    // cout << "src->xsize:" << src->xsize << endl;
    // cout << "src->ysize:" << src->ysize << endl;
    switch (oriention)
    {
    case ORIENT_HORIZONTAL:
        for (i = 1; i < src->ysize - 1; i++)
        {
            // cout << "i:" << i << endl;
            _center = i * src->xsize;
            // cout << "src->xsize:" << src->xsize << endl;
            // cout << "_center:" << _center << endl;
            offset_up = _center - src->xsize;
            offset_down = _center + src->xsize;
            for (j = 1; j < src->xsize - 1; j++)
            {
                _tp = offset_up + j;
                _td = offset_down + j;
                _t = _center + j;
                dst->data[_t] = ((short)psrc[_tp + 1] - (short)psrc[_tp - 1]) + ((short)psrc[_td + 1] - (short)psrc[_td - 1]) + (((short)psrc[_t + 1] - (short)psrc[_t - 1]) << 1);
            }
        }
        break;

    case ORIENT_VERTICAL:
        for (i = 1; i < src->ysize - 1; i++)
        {
            _center = i * src->xsize;
            offset_up = _center - src->xsize;
            offset_down = _center + src->xsize;
            for (j = 1; j < src->xsize - 1; j++)
            {
                _tp = offset_up + j;
                _td = offset_down + j;
                _t = _center + j;

                dst->data[_t] = -((short)psrc[_tp - 1] + (((short)psrc[_tp]) << 1) + (short)psrc[_tp + 1]) + ((short)psrc[_td - 1] + (((short)psrc[_td]) << 1) + (short)psrc[_td + 1]);
            }
        }
        break;

    default:
        printf("sobel oriention is wrong!");
        break;
    }

    psrc = NULL;
}

static void mcv_sobel(ORIENT_CODE oriention, image_int8u_p src, image_int16s_p dst)
{
    sobel_edge(oriention, src, dst);
}

ELESDMatch::ELESDMatch()
{
    // imageWidth = 1280;
    // imageHeight = 960;
    // dxImg_ = NULL;
    // dyImg_ = NULL;
    // dxImg_ = new image_int16s_s;
    // dyImg_ = new image_int16s_s;
}

bool ELESDMatch::NeedAjustDirection(Curve &curve)
{
    short *gradx = dxImg_->data;
    short *grady = dyImg_->data;
    float index = 1 / 255.f; // or 1/(255*4) (sobel)
    float cx = curve.vKeys[0].x, cy = curve.vKeys[0].y;
    // cout<<cx<<","<<cy<<endl;
    float gx = (float)gradx[(uint)cvRound(cx) + (uint)cvRound(cy) * imageWidth] * index;
    float gy = (float)grady[(uint)cvRound(cx) + (uint)cvRound(cy) * imageWidth] * index;
    float weight = sqrt(gx * gx + gy * gy);
    float wsum = weight;
    float wx = gx * weight;
    float wy = gy * weight;

    for (uint i = 1; i < curve.numKeyP; i++)
    {
        cx = (cx + curve.vKeys[i].x) * 0.5;
        cy = (cy + curve.vKeys[i].y) * 0.5;
        gx = (float)gradx[(uint)cvRound(cx) + (uint)cvRound(cy) * imageWidth] * index;
        gy = (float)grady[(uint)cvRound(cx) + (uint)cvRound(cy) * imageWidth] * index;
        weight = sqrt(gx * gx + gy * gy);
        wx += weight * gx;
        wy += weight * gy;
        wsum += weight;

        cx = curve.vKeys[i].x;
        cy = curve.vKeys[i].y;
        gx = (float)gradx[(uint)cvRound(cx) + (uint)cvRound(cy) * imageWidth] * index;
        gy = (float)grady[(uint)cvRound(cx) + (uint)cvRound(cy) * imageWidth] * index;
        weight = sqrt(gx * gx + gy * gy);
        wx += weight * gx;
        wy += weight * gy;
        wsum += weight;
    }
    wx /= wsum;
    wy /= wsum;

    float dx = curve.vKeys.back().x - curve.vKeys[0].x;
    float dy = curve.vKeys.back().y - curve.vKeys[0].y;

    float thdw = (std::atan2(dy, dx) - std::atan2(wy, wx)) * 180 / CV_PI;
    thdw = (thdw > 180) ? (thdw - 360) : ((thdw < -180) ? (thdw + 360) : thdw);

    float len = sqrt(wx * wx + wy * wy);
    curve.dir = cv::Point2f(wx, wy) / len * 20;

    return thdw > 0.f;
}

void ELESDMatch::initvKeyLines()
{
    int id = 0;
    float scaleFactor = 1.2;
    float response_index = 1 / (scaleFactor * 1280);
    int flag = 1;
    for (Curve &curve : curves)
    {
        // cout << "curve.numKeyP: " << curve.numKeyP << endl;
        // if (NeedAjustDirection(curve))
        // {
        std::reverse(curve.vKeys.begin(), curve.vKeys.end());
        // }
        for (uint i = 0; i < curve.numKeyP - 1; i++)
        {
            // std::reverse(curv  e.vKeys.begin(), curve.vKeys.end());
            // if (flag == 0)
            // {
            //     continue;
            // }

            KeyLine keyline;
            keyline.sPointInOctaveX = curve.vKeys[i].x;
            keyline.sPointInOctaveY = curve.vKeys[i].y;
            keyline.ePointInOctaveX = curve.vKeys[i + 1].x;
            keyline.ePointInOctaveY = curve.vKeys[i + 1].y;

            keyline.startPointX = curve.vKeys[i].x;
            keyline.startPointY = curve.vKeys[i].y;
            keyline.endPointX = curve.vKeys[i + 1].x;
            keyline.endPointY = curve.vKeys[i + 1].y;

            float dx, dy;
            dx = keyline.endPointX - keyline.startPointX;
            dy = keyline.endPointY - keyline.startPointY;
            keyline.lineLength = sqrtf(dx * dx + dy * dy);
            keyline.angle = std::atan2(dy, dx);
            keyline.class_id = id;
            keyline.octave = 0;
            keyline.response = keyline.lineLength * response_index;
            keyline.numOfPixels = GetLinePixelsNums(keyline.startPointX, keyline.startPointY, keyline.endPointX, keyline.endPointY);
            keyline.pt.x = (keyline.startPointX + keyline.endPointX) / 2;
            keyline.pt.y = (keyline.startPointY + keyline.endPointY) / 2;
            vKeyLines_.push_back(keyline);
            id++;
            // flag = 0;
        }
    }
    return;
}

void ELESDMatch::initImg(const cv::Mat &gray)
{
    grayImg_ = gray.clone();
    imageWidth = grayImg_.cols;
    // cout << imageWidth << endl;
    imageHeight = grayImg_.rows;
    // cout << imageHeight << endl;
    // cout << "00000000000" << endl;
    image_int8u_p image = new image_int8u_s;
    image->xsize = static_cast<unsigned int>(grayImg_.cols);
    // cout << image->xsize << endl;
    image->ysize = imageHeight;
    // cout << image->ysize << endl;
    image->data = grayImg_.data;
    // cout << "11111111111111" << endl;
    // cout << image->data << endl;
    // for (int i = 0; i < imageWidth * imageHeight; i++)
    // {
    //     cout << "image->data[" << i << "] = " << (int)image->data[i] << endl;
    // }
    // cout << "222222222222" << endl;
    // cout << "333333333333" << endl;
    dxImg_ = new_image_int16s(imageWidth, imageHeight);
    dyImg_ = new_image_int16s(imageWidth, imageHeight);
    mcv_sobel(ORIENT_HORIZONTAL, image, dxImg_);
    mcv_sobel(ORIENT_VERTICAL, image, dyImg_);
}

int ELESDMatch::CalDescriptor(const cv::Mat &gray)
{

    cv::Mat lbds;
    cv::Ptr<BinaryDescriptor> lbd = BinaryDescriptor::createBinaryDescriptor();
    lbd->compute(gray, vKeyLines_, lbds, true);
    cout << "vKeyLines_.size() = " << vKeyLines_.size() << endl;
    // for (int i = 0; i < vKeyLines_.size(); i++)
    // {
    //     cout << "vKeyLines_[" << i << "].lineLength = " << vKeyLines_[i].lineLength << endl;
    //     cout << "vKeyLines_[" << i << "].sPointInOctaveX = " << vKeyLines_[i].sPointInOctaveX << endl;
    //     cout << "vKeyLines_[" << i << "].sPointInOctaveY = " << vKeyLines_[i].sPointInOctaveY << endl;
    //     cout << "vKeyLines_[" << i << "].ePointInOctaveX = " << vKeyLines_[i].ePointInOctaveX << endl;
    //     cout << "vKeyLines_[" << i << "].ePointInOctaveY = " << vKeyLines_[i].ePointInOctaveY << endl;
    //     cout << "vKeyLines_[" << i << "].startPointX = " << vKeyLines_[i].startPointX << endl;
    //     cout << "vKeyLines_[" << i << "].startPointY = " << vKeyLines_[i].startPointY << endl;
    //     cout << "vKeyLines_[" << i << "].endPointX = " << vKeyLines_[i].endPointX << endl;
    //     cout << "vKeyLines_[" << i << "].endPointY = " << vKeyLines_[i].endPointY << endl;
    //     cout << "vKeyLines_[" << i << "].angle = " << vKeyLines_[i].angle << endl;
    //     cout << "vKeyLines_[" << i << "].class_id = " << vKeyLines_[i].class_id << endl;
    //     cout << "vKeyLines_[" << i << "].octave = " << vKeyLines_[i].octave << endl;
    //     cout << "vKeyLines_[" << i << "].response = " << vKeyLines_[i].response << endl;
    //     cout << "vKeyLines_[" << i << "].size = " << vKeyLines_[i].size << endl;
    //     cout << "vKeyLines_[" << i << "].pt.x = " << vKeyLines_[i].pt.x << endl;
    //     cout << "vKeyLines_[" << i << "].pt.y = " << vKeyLines_[i].pt.y << endl;
    // }
    cout << "lbds.size() = " << lbds.size() << endl;
    // for (int i = 0; i < lbds.rows; i++)
    // {
    //     for (int j = 0; j < lbds.cols; j++)
    //     {
    //         cout << lbds.at<float>(i, j) << " ";
    //     }
    //     cout << endl;
    // }

    cv::Mat cbds(curves.size(), lbds.cols, CV_32FC1, cv::Scalar(0.f));
    descriptors_ = cv::Mat(curves.size(), 32, CV_8UC1);

    uint row_lbds = 0;
    for (uint i = 0; i < curves.size(); i++)
    {
        for (uint j = 0; j < curves[i].numKeyP - 1; j++)
        {
            // cbds.row(i) += lbds.row(row_lbds) * vKeyLines_[row_lbds].lineLength;
            cbds.row(i) += lbds.row(row_lbds);
            row_lbds++;
        }
        // cbds.row(i) /= curves[i].Length;
        uchar *pointerToRow = descriptors_.ptr(i);
        float *desVec = cbds.ptr<float>(i);
        for (uint comb = 0; comb < 32; comb++)
        {
            *pointerToRow = binaryConversion(&desVec[8 * combinations[comb][0]], &desVec[8 * combinations[comb][1]]);
            pointerToRow++;
        }
    }

    return 0;
}