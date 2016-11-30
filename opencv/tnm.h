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

#ifndef TNM_H
#define TNM_H

#include <vector>
#include <limits>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "sub_solveP3P.h"

/**
 * Estimate the relative position of the camera and the reference as well as the mirror parameters.
 *
 * @param Xp [in] reference
 * @param q [in] 2D projections of the reference via mirrors
 * @param in_param [in] intrinsic camera parameter
 * @param R [out] relative R from the camera to the reference
 * @param T [out] relative T from the camera to the reference
 * @param n [out] mirror normals in the camera coordinate system
 * @param d [out] distances from the camera to the mirrors
 */
void tnm(const cv::Mat & Xp, const std::vector<cv::Mat> & q, const cv::Mat & in_param, cv::Mat & R, cv::Mat & T, cv::Mat & n, cv::Mat & d);


/**
 * Selects the best combination of reflections by mirrors
 * 
 * @param Cp_candidates [in] possible positions of reflections
 * @param Cp [out] the best positions of reflections
 */
void sub_tnm_orth(const std::vector< std::vector<cv::Mat> > & Cp_candidates, std::vector<cv::Mat> & Cp);


/**
 * Compute R, T, n, d from Xp and Cp
 * 
 * @param Xp [in] reference
 * @param Cp [in] reflections in the camera coordinate system
 * @param R [out] relative R from the camera to the reference
 * @param T [out] relative T from the camera to the reference
 * @param n [out] mirror normals in the camera coordinate system
 * @param d [out] distances from the camera to the mirrors
 */
void sub_tnm_rt(const cv::Mat & Xp, const std::vector<cv::Mat> & Cp, cv::Mat & R, cv::Mat & T, cv::Mat & n, cv::Mat & d);




/*********** Implementations *************/

inline void sub_tnm_orth(const std::vector< std::vector<cv::Mat> > & Cp_candidates, std::vector<cv::Mat> & Cp) {
  const unsigned int num_of_mirror_poses = Cp_candidates.size();

  // How many combinations do we have?
  unsigned int num_of_combinations=1;
  for(unsigned int i=0 ; i<num_of_mirror_poses ; i++) {
    num_of_combinations *= Cp_candidates[i].size();
  }


  // Create "combination" matrix (num_of_combinations x num_of_mirror_poses).
  // Each row stores a set of indices denoting an element of Cp_candidates to be used.
  cv::Mat combinations(Cp_candidates[0].size(), 1, CV_32S);
  for(int i=0 ; i<combinations.rows ; i++) {
    combinations.at<int>(i) = i;
  }
  for(unsigned int i=1 ; i<num_of_mirror_poses ; i++) {
    cv::Mat tmp(combinations.rows * Cp_candidates[i].size(), i+1, CV_32S);
    for(unsigned int j=0 ; j<Cp_candidates[i].size() ; j++) {
      cv::Mat t(tmp(cv::Range(combinations.rows*j,combinations.rows*(j+1)),cv::Range(0,combinations.cols)));
      combinations.copyTo(t);
    }
    combinations = tmp;
    for(int j=0 ; j<combinations.rows ; j++) {
      combinations.at<int>(j,i) = j / (combinations.rows/Cp_candidates[i].size());
    }
  }
  CV_Assert(num_of_combinations == (unsigned int)combinations.rows);
  CV_Assert(num_of_mirror_poses == (unsigned int)combinations.cols);


  // Select the best combination, by seeking the smallest rho
  Cp.resize(num_of_mirror_poses);
  double rho_min = std::numeric_limits<double>::max();
  for(int i=0 ; i<combinations.rows ; i++) {
    double rho = 0;
    // try combinations(i,:)
    for(unsigned int j=0 ; j<num_of_mirror_poses-1 ; j++) {
      for(unsigned int k=j+1 ; k<num_of_mirror_poses ; k++) {
        const cv::Mat Cp_1 = Cp_candidates[j][combinations.at<int>(i,j)];
        const cv::Mat Cp_2 = Cp_candidates[k][combinations.at<int>(i,k)];
        const cv::Mat Q = Cp_1 - Cp_2;
        const cv::Mat M = Q.t() * Q;
        cv::Mat V, D;
        cv::eigen(M, D, V);
        rho += D.at<double>(2) / (D.at<double>(0) + D.at<double>(1) + D.at<double>(2));
      }
    }
    // lower is better
    if(rho < rho_min) {
      rho_min = rho;
      for(unsigned int j=0 ; j<num_of_mirror_poses ; j++) {
        Cp[j] = Cp_candidates[j][combinations.at<int>(i,j)];
      }
    }
  }
}


// build the lhs of Eq (14) in the paper
inline static cv::Mat makeLHS(const cv::Mat & Xp, const cv::Mat & n) {
  const unsigned int num_of_mirror_poses = n.rows;
  const unsigned int num_of_points = Xp.rows;

  // A
  cv::Mat lhs(num_of_points * num_of_mirror_poses * 3, 9 + num_of_mirror_poses, CV_64FC(1), cv::Scalar(0));

  // A(:,1:3)
  const cv::Mat eye = cv::Mat::eye(3,3,CV_64FC(1));
  for(unsigned int i=0 ; i<num_of_points * num_of_mirror_poses ; i++) {
    cv::Mat tmp(lhs(cv::Range(i*3,(i+1)*3), cv::Range(0,3)));
    eye.copyTo(tmp);
  }

  // A(:,4:6)
  for(unsigned int i=0 ; i<num_of_mirror_poses ; i++) {
    for(unsigned int j=0 ; j<num_of_points ; j++) {
      cv::Mat tmp(lhs(cv::Range((i*num_of_points+j)*3,(i*num_of_points+j+1)*3), cv::Range(i+3,i+4)));
      cv::Mat ni(n(cv::Range(i,i+1),cv::Range::all()).t());
      ni.copyTo(tmp);
      tmp *= 2;
    }
  }

  // A(:,7:12)
  for(unsigned int i=0 ; i<num_of_mirror_poses ; i++) {
    for(unsigned int j=0 ; j<num_of_points ; j++) {
      cv::Mat tmp_x(lhs(cv::Range((i*num_of_points+j)*3,(i*num_of_points+j+1)*3), cv::Range(3+num_of_mirror_poses, 3+num_of_mirror_poses+3)));
      cv::Mat tmp_y(lhs(cv::Range((i*num_of_points+j)*3,(i*num_of_points+j+1)*3), cv::Range(3+num_of_mirror_poses+3, 3+num_of_mirror_poses+6)));
      eye.copyTo(tmp_x);
      eye.copyTo(tmp_y);
      tmp_x *= Xp.at<double>(j,0);
      tmp_y *= Xp.at<double>(j,1);
      CV_Assert(Xp.at<double>(j,2) == 0);
    }
  }

  return lhs;
}

// build the rhs of Eq (14) in the paper
inline static cv::Mat makeRHS(const std::vector<cv::Mat> & Cp, const cv::Mat & n) {
  const unsigned int num_of_mirror_poses = n.rows;
  const unsigned int num_of_points = Cp[0].rows;

  CV_Assert(Cp.size() == num_of_mirror_poses);

  // B
  cv::Mat rhs(num_of_points * num_of_mirror_poses * 3, 1, CV_64FC(1), cv::Scalar(0));

  for(unsigned int i=0 ; i<num_of_mirror_poses ; i++) {
    for(unsigned int j=0 ; j<num_of_points ; j++) {
      cv::Mat tmp(rhs(cv::Range((i*num_of_points+j)*3,(i*num_of_points+j+1)*3), cv::Range::all()));
      cv::Mat ni(n(cv::Range(i,i+1),cv::Range::all()).t());
      cv::Mat pij(Cp[i](cv::Range(j,j+1),cv::Range::all()).t());
      tmp = ni * (-2 * ni.t() * pij) + pij;
    }
  }

  return rhs;
}


inline void sub_tnm_rt(const cv::Mat & Xp, const std::vector<cv::Mat> & Cp,
                       cv::Mat & R, cv::Mat & T, cv::Mat & n, cv::Mat & d) {
  const unsigned int num_of_mirror_poses = Cp.size();

  // compute m-vectors
  std::vector<cv::Mat> m_all(num_of_mirror_poses);
  for(unsigned int j=0 ; j<num_of_mirror_poses-1 ; j++) {
    for(unsigned int k=j+1 ; k<num_of_mirror_poses ; k++) {
      const cv::Mat Q = Cp[j] - Cp[k];
      const cv::Mat M = Q.t() * Q;
      cv::Mat V, D;
      cv::eigen(M, D, V);

      cv::Mat v(V(cv::Range(2,3), cv::Range::all()));
      m_all[j].push_back(v);
      m_all[k].push_back(v);
    }
  }

  // compute n-vectors
  n.release();
  for(unsigned int i=0 ; i<num_of_mirror_poses ; i++) {
    cv::Mat V, D;
    cv::eigen(m_all[i].t() * m_all[i], D, V);
    cv::Mat v(V(cv::Range(2,3), cv::Range::all()));
    if(v.at<double>(2) > 0) {
      v *= -1;
    }
    n.push_back(v);
  }

  // solve A*X=B for X
  cv::Mat LHS = makeLHS(Xp, n);
  cv::Mat RHS = makeRHS(Cp, n);
  cv::Mat X = LHS.inv(cv::DECOMP_SVD) * RHS;

  // extract results from X
  T = X(cv::Range(0,3),cv::Range::all());
  d = X(cv::Range(3,3+num_of_mirror_poses),cv::Range::all());
  cv::Mat r1 = X(cv::Range(3+num_of_mirror_poses,6+num_of_mirror_poses),cv::Range::all());
  cv::Mat r2 = X(cv::Range(6+num_of_mirror_poses,9+num_of_mirror_poses),cv::Range::all());

  // refine R linearly
  r1 /= cv::norm(r1);
  r2 /= cv::norm(r2);
  cv::Mat r3 = r1.cross(r2);
  r3 /= cv::norm(r3);

  R.create(3,3,CV_64FC(1));
  R.col(0) = r1 + cv::Scalar(0);
  R.col(1) = r2 + cv::Scalar(0);
  R.col(2) = r3 + cv::Scalar(0);
  cv::SVD svd(R);
  R = svd.u * svd.vt;
}


inline void tnm(const cv::Mat & Xp, const std::vector<cv::Mat> & q, const cv::Mat & in_param, cv::Mat & R, cv::Mat & T, cv::Mat & n, cv::Mat & d) {
  std::vector< std::vector<cv::Mat> > Cp_candidates(q.size());
  for(unsigned int i=0 ; i<q.size() ; i++) {
    sub_solveP3P(Xp, q[i], in_param, Cp_candidates[i]);
  }

  std::vector<cv::Mat> Cp;
  sub_tnm_orth(Cp_candidates, Cp);

  sub_tnm_rt(Xp, Cp, R, T, n, d);
}


#endif //TNM_H
