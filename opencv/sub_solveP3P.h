/**
 * Kosuke Takahashi, Shohei Nobuhara and Takashi Matsuyama: A New Mirror-based
 * Extrinsic Camera Calibration Using an Orthogonality Constraint, CVPR2012
 *
 * For further information, please visit our web page.
 *   http://vision.kuee.kyoto-u.ac.jp/~nob/proj/mirror
 *
 *
 *
 * Copyright (c) 2012, Kosuke Takahashi, Shohei Nobuhara and Takashi Matsuyama
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the Graduate School of Informatics, Kyoto
 *      University, Japan nor the names of its contributors may be used to
 *      endorse or promote products derived from this software without specific
 *      prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SUB_SOLVEP3P_H
#define SUB_SOLVEP3P_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/**
 * Solves P3P (projective-three-points) problem
 * 
 * @param Xp [in] 3D coordiates of three reference points (3x3)
 * @param q [in] 2D projections of Xp (3x2)
 * @param in_param [in] Intrinsic parameter (3x3)
 * @param Cp [out] Up to four possible positions of Xp in the camera coordiate system
 */
inline void sub_solveP3P(const cv::Mat & Xp,
                         const cv::Mat & q,
                         const cv::Mat & in_param,
                         std::vector<cv::Mat> & Cp) {
  CV_Assert(3 == Xp.rows);
  CV_Assert(3 == Xp.cols);
  CV_Assert(3 == q.rows);
  CV_Assert(2 == q.cols);
  CV_Assert(3 == in_param.rows);
  CV_Assert(3 == in_param.cols);

  cv::Mat q_t(3, 3, CV_64FC(1), cvScalar(1));
  q_t(cv::Range(0,2), cv::Range::all()) = q.t();

  const cv::Mat Xp_t = Xp.t();

  const cv::Mat ovec = in_param.inv() * q_t;

  cv::Mat novec(ovec);
  novec(cv::Range::all(), cv::Range(0,1)) /= cv::norm(ovec(cv::Range::all(), cv::Range(0,1)));
  novec(cv::Range::all(), cv::Range(1,2)) /= cv::norm(ovec(cv::Range::all(), cv::Range(1,2)));
  novec(cv::Range::all(), cv::Range(2,3)) /= cv::norm(ovec(cv::Range::all(), cv::Range(2,3)));

  const double cosa = cv::Mat(novec(cv::Range::all(), cv::Range(1,2)).t() * novec(cv::Range::all(), cv::Range(2,3))).at<double>(0,0);
  const double cosb = cv::Mat(novec(cv::Range::all(), cv::Range(2,3)).t() * novec(cv::Range::all(), cv::Range(0,1))).at<double>(0,0);
  const double cosc = cv::Mat(novec(cv::Range::all(), cv::Range(0,1)).t() * novec(cv::Range::all(), cv::Range(1,2))).at<double>(0,0);

  const double a = cv::norm(Xp_t(cv::Range::all(), cv::Range(1,2)) - Xp_t(cv::Range::all(), cv::Range(2,3)));
  const double b = cv::norm(Xp_t(cv::Range::all(), cv::Range(2,3)) - Xp_t(cv::Range::all(), cv::Range(0,1)));
  const double c = cv::norm(Xp_t(cv::Range::all(), cv::Range(0,1)) - Xp_t(cv::Range::all(), cv::Range(1,2)));

  const double D4 = 4 * b*b * c*c * cosa*cosa - (a*a - b*b - c*c) * (a*a - b*b - c*c);

  const double D3 = -4 * c*c * (a*a + b*b - c*c) * cosa * cosb
    - 8 * b*b * c*c * cosa*cosa * cosc
    + 4 * (a*a - b*b - c*c) * (a*a - b*b)* cosc;

  const double D2 = 4 * c*c * (a*a - c*c) * cosb*cosb 
    + 8 * c*c * (a*a + b*b) * cosa * cosb * cosc 
    + 4 * c*c * (b*b - c*c) * cosa*cosa 
    - 2 * (a*a - b*b - c*c) * (a*a - b*b + c*c)
    - 4 * (a*a - b*b) * (a*a - b*b) * cosc*cosc;

  const double D1 = -8 * a*a * c*c * cosb*cosb * cosc 
    - 4 * c*c * (b*b - c*c) * cosa * cosb 
    - 4 * a*a * c*c * cosa * cosb 
    + 4 * (a*a - b*b) * (a*a - b*b + c*c) * cosc;

  const double D0 = 4 * a*a * c*c * cosb*cosb - (a*a - b*b + c*c) * (a*a - b*b + c*c);

  const cv::Mat D = (cv::Mat_<double>(1,5) << D0/D4, D1/D4, D2/D4, D3/D4, D4/D4);
  cv::Mat u;
  cv::solvePoly(D, u);

  std::vector<double> ru_;
  for(int r=0 ; r<u.rows ; r++) {
    if( fabs(u.at<double>(r,1)) < 1e-8 ) {
      ru_.push_back(u.at<double>(r,0));
    }
  }
  const cv::Mat ru = cv::Mat(1, ru_.size(), CV_64F, &(ru_[0])).t();

  cv::Mat v(ru.rows, 1, CV_64FC(1));
  cv::Mat s1(ru.rows, 1, CV_64FC(1));
  for(int i=0 ; i<ru.rows ; i++) {
    double temp1 = - 1 * ((a*a - b*b - c*c) * ru.at<double>(i) * ru.at<double>(i) 
                          + 2 * (b*b - a*a) * cosc * ru.at<double>(i) 
                          + a*a - b*b + c*c);
    double temp2 = 2 * c*c * (cosa * ru.at<double>(i) - cosb);
    v.at<double>(i) = temp1 / temp2;
  }
  for(int i=0 ; i<ru.rows ; i++) {
    double temp = a*a / (ru.at<double>(i)*ru.at<double>(i) + v.at<double>(i)*v.at<double>(i) - 2 * ru.at<double>(i) * v.at<double>(i) * cosa);
    s1.at<double>(i) = sqrt(temp);
  }

  const cv::Mat s2 = s1.mul(ru);
  const cv::Mat s3 = s1.mul(v);

  Cp.resize(ru.rows);
  for(int i=0 ; i<ru.rows ; i++) {
    cv::Mat p(3, 3, CV_64F);
    p(cv::Range(0,1), cv::Range::all()) = novec(cv::Range::all(),cv::Range(0,1)).mul(s1.at<double>(i)).t();
    p(cv::Range(1,2), cv::Range::all()) = novec(cv::Range::all(),cv::Range(1,2)).mul(s2.at<double>(i)).t();
    p(cv::Range(2,3), cv::Range::all()) = novec(cv::Range::all(),cv::Range(2,3)).mul(s3.at<double>(i)).t();
    Cp[i] = p;
  }

  // for regular P3P, we should solve "absolute orientation" between Xp and Cp[i] to get R and T.
  // but we stop here since TNM does not require them.
}

#endif //SUB_SOLVEP3P_H
