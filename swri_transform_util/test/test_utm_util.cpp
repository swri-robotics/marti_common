// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <cmath>
#include <cstdlib>

#include <gtest/gtest.h>

#include <swri_transform_util/utm_util.h>

TEST(UtmUtilTests, GetZone)
{
  EXPECT_EQ(11, swri_transform_util::GetZone(-118.408056));  // LAX
  EXPECT_EQ(17, swri_transform_util::GetZone( -80.290556));  // MIA
  EXPECT_EQ(30, swri_transform_util::GetZone(  -0.461389));  // LHR
  EXPECT_EQ(37, swri_transform_util::GetZone(  37.414722));  // SVO
  EXPECT_EQ(54, swri_transform_util::GetZone( 139.781111));  // HND
  EXPECT_EQ( 4, swri_transform_util::GetZone(  -157.9225));  // HNL
  EXPECT_EQ(54, swri_transform_util::GetZone(  138.530556)); // ADL
}

TEST(UtmUtilTests, GetBand)
{
  EXPECT_EQ('S', swri_transform_util::GetBand( 33.9425));    // LAX
  EXPECT_EQ('R', swri_transform_util::GetBand( 25.793333));  // MIA
  EXPECT_EQ('U', swri_transform_util::GetBand( 51.4775));    // LHR
  EXPECT_EQ('U', swri_transform_util::GetBand( 55.972778));  // SVO
  EXPECT_EQ('S', swri_transform_util::GetBand( 35.553333));  // HND
  EXPECT_EQ('Q', swri_transform_util::GetBand( 21.318611));  // HNL
  EXPECT_EQ('F', swri_transform_util::GetBand(-54.843333));  // USH
  EXPECT_EQ('H', swri_transform_util::GetBand(-34.945));     // ADL

  EXPECT_EQ('Z', swri_transform_util::GetBand(84.5));
  EXPECT_EQ('Z', swri_transform_util::GetBand(-80.5));
}

TEST(UtmUtilTests, ToUtm)
{
  swri_transform_util::UtmUtil utm_util;

  double easting, northing;
  int zone;
  char band;

  // LAX
  utm_util.ToUtm(33.9425, -118.408056, zone, band, easting, northing);
  EXPECT_EQ(11, zone);
  EXPECT_EQ('S', band);
  EXPECT_NEAR(369877.0, easting, 0.5);
  EXPECT_FLOAT_EQ(3756673.0, northing);

  utm_util.ToUtm(33.9425, -118.408056, easting, northing);
  EXPECT_NEAR(369877.0, easting, 0.5);
  EXPECT_FLOAT_EQ(3756673.0, northing);

  // MIA
  utm_util.ToUtm(25.793333, -80.290556, zone, band, easting, northing);
  EXPECT_EQ(17, zone);
  EXPECT_EQ('R', band);
  EXPECT_NEAR(571124.0, easting, 0.5);
  EXPECT_FLOAT_EQ(2852989.0, northing);

  utm_util.ToUtm(25.793333, -80.290556, easting, northing);
  EXPECT_NEAR(571124.0, easting, 0.5);
  EXPECT_FLOAT_EQ(2852989.0, northing);

  // USH
  utm_util.ToUtm(-54.843333, -68.295556, zone, band, easting, northing);
  EXPECT_EQ(19, zone);
  EXPECT_EQ('F', band);
  EXPECT_FLOAT_EQ(545237.0, easting);
  EXPECT_FLOAT_EQ(3922415.0, northing);

  utm_util.ToUtm(-54.843333, -68.295556, easting, northing);
  EXPECT_FLOAT_EQ(545237.0, easting);
  EXPECT_FLOAT_EQ(3922415.0, northing);

  // ADL
  utm_util.ToUtm(-34.945, 138.530556, zone, band, easting, northing);
  EXPECT_EQ(54, zone);
  EXPECT_EQ('H', band);
  EXPECT_NEAR(274484.0, easting, 0.5);
  EXPECT_FLOAT_EQ(6130272.0, northing);

  utm_util.ToUtm(-34.945, 138.530556, easting, northing);
  EXPECT_NEAR(274484.0, easting, 0.5);
  EXPECT_FLOAT_EQ(6130272.0, northing);
}

TEST(UtmUtilTests, ToWgs84)
{
  swri_transform_util::UtmUtil utm_util;

  double lat, lon;

  // LAX
  utm_util.ToLatLon(11, 'S', 369877.0, 3756673.0, lat, lon);
  EXPECT_FLOAT_EQ(33.9425, lat);
  EXPECT_NEAR(-118.408056, lon, .000005);

  // MIA
  utm_util.ToLatLon(17, 'R', 571124.0, 2852989.0, lat, lon);
  EXPECT_FLOAT_EQ(25.793333, lat);
  EXPECT_NEAR(-80.290556, lon, .000005);

  // USH
  utm_util.ToLatLon(19, 'F', 545237.0, 3922415.0, lat, lon);
  EXPECT_FLOAT_EQ(-54.843333, lat);
  EXPECT_FLOAT_EQ(-68.295556, lon);

  // ADL
  utm_util.ToLatLon(54, 'H', 274484.0, 6130272.0, lat, lon);
  EXPECT_FLOAT_EQ(-34.945, lat);
  EXPECT_FLOAT_EQ(138.530556, lon);
}

TEST(UtmUtilTests, Continuity)
{
  swri_transform_util::UtmUtil utm_util;

  // (FOR) - Fortaleza International Airport
  double easting = 551940.0;
  double northing = 9582637.0;

  double last_lon = 0;

  for (int i = 0; i < 1000; i++)
  {
    double new_lat;
    double new_lon;
    double new_easting;
    double new_northing;
    int zone;
    char band;
    utm_util.ToLatLon(24, 'M', easting + i * 1.11 / 100.0, northing, new_lat, new_lon);
    utm_util.ToUtm(new_lat, new_lon, zone, band, new_easting, new_northing);

    EXPECT_FLOAT_EQ(easting + i * 1.11 / 100.0, new_easting);
    EXPECT_FLOAT_EQ(northing, new_northing);

    if (i > 0)
    {
      // The difference should be 1.11cm which is approximately
      // 1/10th of 1 microdegree near the equator
      EXPECT_NEAR(0.0000001, std::fabs(new_lon - last_lon), 0.00000001);
    }

    last_lon = new_lon;
  }
}

TEST(UtmUtilTests, Random)
{
  swri_transform_util::UtmUtil utm_util;

  std::srand(0);

  for (int i = 0; i < 1000; i++)
  {
    double lon = (static_cast<double>(std::rand()) / RAND_MAX) * 360.0 - 180;
    double lat = (static_cast<double>(std::rand()) / RAND_MAX) * 140.0 - 70;

    char band;
    int zone;
    double easting;
    double northing;
    utm_util.ToUtm(lat, lon, zone, band, easting, northing);

    double new_lat;
    double new_lon;
    utm_util.ToLatLon(zone, band, easting, northing, new_lat, new_lon);

    EXPECT_FLOAT_EQ(lat, new_lat);
    EXPECT_FLOAT_EQ(lon, new_lon);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
