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
/*
 * image_normalization.cpp
 *
 *  Created on: Jan 19, 2012
 *      Author: kkozak
 */

#include <image_util/image_normalization.h>

namespace image_util
{
  void normalize_illumination(cv::Mat NormImage,
                              cv::Mat SourceImage,
                              cv::Mat& DestImage)
  {
    // note that Norm image should be of type CV_32FC1 and scaled between 0 and
    // 1 (but not exactly equal to 0 anywhere)
    cv::Mat tempIm1;
    SourceImage.convertTo(tempIm1, CV_32FC1, 1.0, 0.0);
    cv::divide(tempIm1, NormImage, tempIm1, 1.0);
    cv::Mat firstImage;
    cv::Mat secondImage;
    tempIm1.convertTo(DestImage, CV_8UC1);
  }

  cv::Mat generate_normalization_image(const std::vector<cv::Mat>& image_list)
  {
    cv::Mat norm_image;
    if (image_list.empty())
    {
      return cv::Mat();
    }

    const cv::Mat& rep_im = image_list.front();

    cv::Mat image_sum(cv::Size(rep_im.cols, rep_im.rows), CV_64F);

    for (uint32_t i = 0; i < image_list.size(); i++)
    {
      cv::Mat temp_im;
      image_list[i].convertTo(temp_im, CV_64F, 1.0, 0.0);

      image_sum = image_sum + temp_im;
    }

    image_sum = image_sum / static_cast<double>(image_list.size());

    cv::Mat mean_image;
    image_sum.convertTo(mean_image, CV_8U);
    cv::Mat temp_norm_image;

    cv::medianBlur(mean_image, temp_norm_image, 45);

    cv::Mat temp_norm_image2;
    temp_norm_image.convertTo(temp_norm_image2, CV_32F);
    double max1 = 0;
    for (int32_t i = 0; i < temp_norm_image2.rows; i++)
    {
      for (int32_t j = 0; j < temp_norm_image2.cols; j++)
      {
        if (temp_norm_image2.at<float>(i, j) > max1)
        {
          max1 = temp_norm_image2.at<float>(i, j);
        }
      }
    }

    temp_norm_image2 = temp_norm_image2 * (255.0 / max1);

    cv::Mat temp_norm_image3;
    temp_norm_image2.convertTo(temp_norm_image3, CV_8U, 1.0, 0.0);


    cv::GaussianBlur(temp_norm_image3,
                     norm_image,
                     cv::Size(25, 25),
                     5,
                     5);

    return norm_image;
  }

  cv::Mat scale_2_8bit(const cv::Mat& image)
	{
		if (image.type() == CV_8UC1)
		  return image;
		cv::Mat Image8Bit(image.rows, image.cols, CV_8U), Image8BitColor; //Define an 8bit image
		//Convert the image to 32bit float
		cv::Mat ImageFloat;
		image.convertTo(ImageFloat, CV_32F, 1, 0);
		double maxVal; //Define the max value of image
		cv::minMaxLoc(ImageFloat.reshape(1,1), NULL, &maxVal);//Extract the max value of image
		//ReScale the image to 0 to 255
		ImageFloat = ImageFloat*((1 << 8)/pow(2, ceil(log(maxVal)/log(2))));
		ImageFloat.convertTo(Image8Bit,CV_8U, 1, 0);
		return Image8Bit;
	}

	cv::Mat scale_2_8bit_color(const cv::Mat& image)
	{
		if (image.type() == CV_8UC3)
		  return image;
		cv::Mat Image8Bit = scale_2_8bit(image), Image8BitColor;
		cvtColor(Image8Bit, Image8BitColor, CV_GRAY2BGR);
		return Image8BitColor;
	}
}
