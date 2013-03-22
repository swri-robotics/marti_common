// *****************************************************************************
//
// Copyright (C) 2011 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-R8248
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <image_util/draw_util.h>

#include <algorithm>

#include <QPolygonF>
#include <QPointF>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ros/ros.h>

namespace image_util
{

  void DrawOverlap(
      const std::string& title,
      const cv::Mat& image1,
      const cv::Mat& image2,
      const cv::Mat& transform)
  {
    cv::Mat image2_warped;
    cv::warpAffine(
      image2,
      image2_warped,
      transform,
      cv::Size(image2.cols, image2.rows));
      
    cv::Mat sub = image1 - image2_warped;
    
    cv::imshow(title, sub);
  }

  void DrawMatches(
      cv::Mat& image_out,
      const cv::Mat image1,
      const cv::Mat image2,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color,
      bool draw_image_borders)
  {
    cv::Size size(image1.cols + image2.cols, std::max(image1.rows, image2.rows));
    image_out.create( size, CV_MAKETYPE(image1.depth(), 3));
    cv::Mat draw_image1 = image_out(cv::Rect(0, 0, image1.cols, image1.rows));
    cv::Mat draw_image2 = image_out(cv::Rect(image1.cols, 0, image2.cols, image2.rows));

    if(image1.type() == CV_8U)
    {
      cvtColor(image1, draw_image1, CV_GRAY2BGR);
    }
    else
    {
      image1.copyTo(draw_image1);
    }

    if(image2.type() == CV_8U)
    {
      cvtColor(image2, draw_image2, CV_GRAY2BGR);
    }
    else
    {
      image2.copyTo(draw_image2);
    }

    if(draw_image_borders)
    {
      cv::rectangle(draw_image1,
                    cv::Point(0,0),
                    cv::Point(image1.cols, image1.rows),
                    cv::Scalar(0,0,0),
                    2);

      cv::rectangle(draw_image2,
                    cv::Point(0,0),
                    cv::Point(image2.cols, image2.rows),
                    cv::Scalar(0,0,0),
                    2);

    }

    cv::RNG rng = cv::theRNG();
    bool rand_color = color == cv::Scalar::all(-1);

    for (int i = 0; i < points1.rows; i++)
    {
      cv::Scalar match_color = rand_color ? cv::Scalar(rng(256), rng(256), rng(256)) : color;
      cv::Point2f center1(cvRound(points1.at<cv::Vec2f>(0, i)[0] * 16.0), cvRound(points1.at<cv::Vec2f>(0, i)[1] * 16.0));
      cv::Point2f center2(cvRound(points2.at<cv::Vec2f>(0, i)[0] * 16.0), cvRound(points2.at<cv::Vec2f>(0, i)[1] * 16.0));
      cv::Point2f dcenter2 = cv::Point2f(std::min(center2.x + draw_image1.cols * 16.0, float(image_out.cols - 1) * 16.0), center2.y);
      circle(draw_image1, center1, 48, match_color, 1, CV_AA, 4);
      circle(draw_image2, center2, 48, match_color, 1, CV_AA, 4);
      line(image_out, center1, dcenter2, match_color, 1, CV_AA, 4);
    }
  }

  void DrawMatches(
      const std::string& title,
      const cv::Mat image1,
      const cv::Mat image2,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color,
      bool draw_image_borders)
  {
    cv::Mat image_out;
    DrawMatches(image_out,
                image1,
                image2,
                points1,
                points2,
                color,
                draw_image_borders);

    imshow(title, image_out);
  }

  void DrawMatches(
      const std::string& title,
      const cv::Mat image,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color1,
      const cv::Scalar& color2,
      bool draw_image_borders)
  {
    cv::Mat draw_image;
    if(image.type() == CV_8U)
    {
      cvtColor(image, draw_image, CV_GRAY2BGR);
    }
    else
    {
      draw_image = image.clone();
    }

    for (int i = 0; i < points1.rows; i++)
    {
      cv::Point2f center1(cvRound(points1.at<cv::Vec2f>(0, i)[0] * 16.0), cvRound(points1.at<cv::Vec2f>(0, i)[1] * 16.0));
      cv::Point2f center2(cvRound(points2.at<cv::Vec2f>(0, i)[0] * 16.0), cvRound(points2.at<cv::Vec2f>(0, i)[1] * 16.0));
      circle(draw_image, center1, 48, color1, 1, CV_AA, 4);
      line(draw_image, center1, center2, color2, 1, CV_AA, 4);
    }

    imshow(title, draw_image);
  }
}
