// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef SWRI_ROSCPP_OPTIONAL_DIAGNOSED_PUBLISHER_H_
#define SWRI_ROSCPP_OPTIONAL_DIAGNOSED_PUBLISHER_H_

#include <diagnostic_updater/publisher.h>

namespace swri
{
  /**
   * The DiagnosedPublisher class is handy, but it has on problem: it can't be
   * turned off.  If you ever stop publishing, it will flag an error because
   * your publish rate is 0.  If you are intentionally not publishing because you
   * don't have any subscribers in order to avoid unnecessary processing, that is
   * inconvenient.
   *
   * This class simply wraps the DiagnosedPublisher and adds a setEnabled method
   * that can be used to enable or disable it.  When disabled, it will simply
   * publish a diagnostic with an "OK" status.
   *
   * In theory it could dynamically determine whether you have subscribers or not and
   * change it status based on that, but that might not be what you want in the event
   * that you don't have subscribers but still have some kind of error to report.
   * @tparam T The type of message being published.
   */
  template<class T>
  class OptionalDiagnosedPublisher : public diagnostic_updater::DiagnosedPublisher<T>
  {
  public:
    OptionalDiagnosedPublisher(const ros::Publisher& pub,
                               diagnostic_updater::Updater& diag,
                               const diagnostic_updater::FrequencyStatusParam& freq,
                               const diagnostic_updater::TimeStampStatusParam& stamp) :
        diagnostic_updater::DiagnosedPublisher<T>(pub, diag, freq, stamp),
        is_diagnostic_enabled_(true)
    {}

    virtual ~OptionalDiagnosedPublisher()
    {}

    virtual void run(diagnostic_updater::DiagnosticStatusWrapper& stat)
    {
      if (is_diagnostic_enabled_)
      {
        diagnostic_updater::CompositeDiagnosticTask::run(stat);
      }
      else
      {
        diagnostic_updater::DiagnosticStatusWrapper wrapper;
        wrapper.level = 0;
        wrapper.message = "Diagnostic disabled.";
        stat.summary(wrapper);
      }
    }

    /**
     * Controls whether the diagnostic for this publisher is enabled.
     * @param enabled "true" to enable the diagnostic.
     */
    virtual void setEnabled(bool enabled)
    {
      is_diagnostic_enabled_ = enabled;
    }

  private:
    bool is_diagnostic_enabled_;
  };
}

#endif
