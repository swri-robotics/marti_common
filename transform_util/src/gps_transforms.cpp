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

#include <cmath>

#include <transform_util/gps_transforms.h>

#include <ros/ros.h>

namespace transform_util
{

  tf::Transform GetRelativeTransform(double latitude, double longitude, double yaw, double reference_latitude, double reference_longitude, double reference_yaw)
  {
    tf::Transform transform = tf::Transform::getIdentity();

    tf::Quaternion reference_rotation = tf::Quaternion::getIdentity();
    reference_rotation.setRotation(tf::Vector3(0,0,1), reference_yaw);

    tf::Quaternion rotation = tf::Quaternion::getIdentity();
    rotation.setRotation(tf::Vector3(0,0,1), yaw);

    transform.setRotation(reference_rotation.inverse() * rotation);

    double x, y;
    ToLocalXY(x, y, latitude, longitude, reference_latitude, reference_longitude);


    tf::Vector3 origin = tf::Transform(reference_rotation) * tf::Vector3(x, y, 0);
    transform.setOrigin(origin);

    return transform;
  }

  void ToLocalXY(double& x, double& y, double latitude, double longitude, double reference_latitude, double reference_longitude)
  {
  	double rlat = latitude * _pi / 180.0;
	  double rlon = longitude * _pi / 180.0;

	  double ref_rlat = reference_latitude  * _pi / 180.0;
	  double ref_rlon = reference_longitude  * _pi / 180.0;
	  double tmp = _earth_eccentricity * std::sin(ref_rlat);
	  tmp = 1.0 - tmp * tmp;
	  double sqt = std::sqrt(tmp);
	  double rho_e = _earth_equator_radius * (1.0 - _earth_eccentricity * _earth_eccentricity) / (sqt * tmp);
	  double rho_n = _earth_equator_radius / sqt;
	  double rad_lat = rho_e; /* - depth; */
	  double rad_lon = (rho_n /* - depth  */) * cos(ref_rlat);

	  y = (rlat - ref_rlat) * rad_lat;
	  x = (rlon - ref_rlon) * rad_lon;
  }

  void ToLatLon(double& latitude, double& longitude, double x, double y, double reference_latitude, double reference_longitude)
  {
    double ref_rlat = reference_latitude  * _pi / 180.0;
    double ref_rlon = reference_longitude  * _pi / 180.0;

    double tmp = _earth_eccentricity * std::sin(ref_rlat);
    tmp = 1.0 - tmp * tmp;
    double sqt = std::sqrt(tmp);
    double rho_e = _earth_equator_radius * (1.0 - _earth_eccentricity * _earth_eccentricity) / (sqt * tmp);
    double rho_n = _earth_equator_radius / sqt;
    double rad_lat = rho_e; /* - depth; */
    double rad_lon = (rho_n /* - depth  */) * cos(ref_rlat);

    latitude = (y / rad_lat + ref_rlat) * 180.0 / _pi;
    longitude = (x / rad_lon + ref_rlon) * 180.0 / _pi;
  }

  uint32_t GetZone(double longitude)
  {
    int32_t zone = static_cast<int32_t>((longitude + 180.0) / 6.0) + 1;
    if (zone < 1) zone = 1;
    if (zone > 60) zone = 60;

    return static_cast<uint32_t>(zone);
  }

  char GetBand(double latitude)
  {
    //This routine determines the correct UTM letter designator for the given latitude
    //returns 'Z' if latitude is outside the UTM limits of 84N to 80S
    //Written by Chuck Gantz- chuck.gantz@globalstar.com

    char band;

    if((84 <= latitude) && (latitude >= 72)) band = 'X';
    else if((72 > latitude) && (latitude >= 64)) band = 'W';
    else if((64 > latitude) && (latitude >= 56)) band = 'V';
    else if((56 > latitude) && (latitude >= 48)) band = 'U';
    else if((48 > latitude) && (latitude >= 40)) band = 'T';
    else if((40 > latitude) && (latitude >= 32)) band = 'S';
    else if((32 > latitude) && (latitude >= 24)) band = 'R';
    else if((24 > latitude) && (latitude >= 16)) band = 'Q';
    else if((16 > latitude) && (latitude >= 8)) band = 'P';
    else if(( 8 > latitude) && (latitude >= 0)) band = 'N';
    else if(( 0 > latitude) && (latitude >= -8)) band = 'M';
    else if((-8 > latitude) && (latitude >= -16)) band = 'L';
    else if((-16 > latitude) && (latitude >= -24)) band = 'K';
    else if((-24 > latitude) && (latitude >= -32)) band = 'J';
    else if((-32 > latitude) && (latitude >= -40)) band = 'H';
    else if((-40 > latitude) && (latitude >= -48)) band = 'G';
    else if((-48 > latitude) && (latitude >= -56)) band = 'F';
    else if((-56 > latitude) && (latitude >= -64)) band = 'E';
    else if((-64 > latitude) && (latitude >= -72)) band = 'D';
    else if((-72 > latitude) && (latitude >= -80)) band = 'C';
    else band = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits

    return band;
  }

  UtmTransforms::UtmTransforms()
  {
    // Initialize lat long projection.
    lat_lon_ = pj_init_plus("+proj=latlong +ellps=WGS84");

    // Initialize projection for each UTM zone.
    char args[64];
    for (int i = 0; i < 60; i++)
    {
      sprintf(args, "+proj=utm +ellps=WGS84 +zone=%d", i + 1);
      utm_north_[i] = pj_init_plus(args);

      sprintf(args, "+proj=utm +ellps=WGS84 +zone=%d +south", i + 1);
      utm_south_[i] = pj_init_plus(args);
    }
  }

  UtmTransforms::~UtmTransforms()
  {
    pj_free(lat_lon_);

    // Cleanup projection memory.
    for (int i = 0; i < 60; i++)
    {
      pj_free(utm_north_[i]);
      pj_free(utm_south_[i]);
    }
  }

  void UtmTransforms::ToUtm(double latitude, double longitude, int& zone, char& band, double& easting, double& northing) const
  {
    zone = GetZone(longitude);
    band = GetBand(latitude);

    double x = longitude * _pi / 180.0;
    double y = latitude * _pi / 180.0;

    // Get easting and northing values.
    if (band <= 'N')
    {
      pj_transform(lat_lon_, utm_north_[zone - 1], 1, 0, &x, &y, NULL);
    }
    else
    {
      pj_transform(lat_lon_, utm_north_[zone - 1], 1, 0, &x, &y, NULL);
    }

    easting = x;
    northing = y;
  }

  void UtmTransforms::ToLatLon(int zone, char band, double easting, double northing, double& latitude, double& longitude) const
  {
    double x = easting;
    double y = northing;

    if (band <= 'N')
    {
      pj_transform(utm_north_[zone - 1], lat_lon_, 1, 0, &x, &y, NULL);
    }
    else
    {
      pj_transform(utm_north_[zone - 1], lat_lon_, 1, 0, &x, &y, NULL);
    }

    longitude = x * 180.0 / _pi;
    latitude = y * 180.0 / _pi;
  }

}
