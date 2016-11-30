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

#include <iostream>
#include <fstream>
#include <sstream>

#include "tnm.h"


/**
 * Load data from file to cv::Mat
 *
 * @param filename [in] filename
 * @param m [out] matrix
 */
void load(const char * filename, cv::Mat & m) {
  CV_Assert(m.type() == CV_64FC(1));

  std::cout << "loading '" << filename << "' = ";

  std::ifstream ifs(filename);
  for(int r=0 ; r<m.rows ; r++) {
    std::string line;
    std::getline(ifs, line);
    std::istringstream iss(line);
    for(int c=0 ; c<m.cols ; c++) {
      iss >> m.at<double>(r,c);
    }
  }
  ifs.close();

  std::cout << m << "\n\n";
}

// returns average reprojection error, for evaluation only.
double sub_reproj(const cv::Mat & Xp,
                  const std::vector<cv::Mat> & q,
                  const cv::Mat & R,
                  const cv::Mat & T,
                  const cv::Mat & n,
                  const cv::Mat & d,
                  const cv::Mat & in_param) {
  const unsigned int num_of_mirror_poses = n.rows;
  const unsigned int num_of_points = q[0].rows;

  CV_Assert(num_of_mirror_poses == q.size());
  CV_Assert(num_of_mirror_poses == (unsigned int)d.rows);

  double e = 0;
  for(unsigned int i=0 ; i<num_of_mirror_poses ; i++) {

    // Householder transformation matrix
    cv::Mat H = cv::Mat::eye(4,4,CV_64FC(1));
    H(cv::Range(0,3),cv::Range(0,3)) = cv::Mat::eye(3,3,CV_64FC(1)) - 2 * n.row(i).t() * n.row(i);
    H(cv::Range(0,3),cv::Range(3,4)) = n.row(i).t() * (-2 * d.at<double>(i));

    for(unsigned int j=0 ; j<num_of_points ; j++) {
      // reflection of j-th reference point by i-th mirror
      cv::Mat Cp = H(cv::Range(0,3),cv::Range(0,3)) * (R * Xp.row(j).t() + T) + H(cv::Range(0,3),cv::Range(3,4));

      // projection
      cv::Mat q2 = in_param * Cp;
      q2 /= q2.at<double>(2);

      // diff
      q2(cv::Range(0,2),cv::Range::all()) -= q[i].row(j).t();
      q2.at<double>(2) = 0;

      e += cv::norm(q2);
    }
  }

  return e / (num_of_mirror_poses * num_of_points);
}

int main() {
  // init vars
  std::vector<cv::Mat> input(3);
  input[0].create(3,2,CV_64FC(1));
  input[1].create(3,2,CV_64FC(1));
  input[2].create(3,2,CV_64FC(1));
  cv::Mat model(3, 3, CV_64FC(1));
  cv::Mat camera(3, 3, CV_64FC(1));

  // load input data from files
  load("data/input1.txt", input[0]);
  load("data/input2.txt", input[1]);
  load("data/input3.txt", input[2]);
  load("data/model.txt", model);
  load("data/camera.txt", camera);

  // calibration
  cv::Mat R, T, n, d;
  tnm(model, input, camera, R, T, n, d);

  // evaluation
  double e = sub_reproj(model, input, R, T, n, d, camera);

  // output
  std::cout << "\n\nAverage reprojection error by TNM : " << e << " pixels\n\n"
            << "==== Parameters by TNM ====\n\n"
            << "R  = " << R << "\n\n"
            << "T  = " << T << "\n\n"
            << "n1 = " << n.row(0).t() << "\n\n"
            << "n2 = " << n.row(1).t() << "\n\n"
            << "n3 = " << n.row(2).t() << "\n\n"
            << "d1 = " << d.at<double>(0) << "\n\n"
            << "d2 = " << d.at<double>(1) << "\n\n"
            << "d3 = " << d.at<double>(2) << "\n\n";

  return 0;
}

