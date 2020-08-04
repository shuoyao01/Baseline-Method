
// This is NWNC (No Warranty No Copyright) Software.
// astar.ai
// Nov 16, 2018
// Modified by: Shuo Yao

#include <opencv2/opencv.hpp>

////////////////////////////////////////////////////////////////////////////////

bool      live = false;
//To run live mode, you need a CaliCam from www.astar.ai

int       vfov_bar =  0, width_bar =   0, height_bar =   0;
int       vfov_max = 120, width_max = 1280, height_max = 960;
int       vfov_now = 80, width_now = 630, height_now = 480;

int       ndisp_bar =  1, wsize_bar = 2;
int       ndisp_max =  2, wsize_max = 4;
int       ndisp_now = 32, wsize_now = 7;

int       cap_cols, cap_rows, img_width;
bool      changed = false;
bool      is_sgbm = true;
cv::Mat   Translation, Kl, Kr, Dl, Dr, xil, xir, Rl, Rr, smap[2][2], Knew;

std::string cam_model;

////////////////////////////////////////////////////////////////////////////////

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackAngle(int, void*) {
  vfov_now = 60 + vfov_bar;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackWidth(int, void*) {
  width_now = 480 + width_bar;
  if (width_now % 2 == 1)
    width_now--;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackHeight(int, void*) {
  height_now = 360 + height_bar;
  if (height_now % 2 == 1)
    height_now--;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackNdisp(int, void*) {
  ndisp_now = 16 + 16 * ndisp_bar;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void OnTrackWsize(int, void*) {
  wsize_now = 3 + 2 * wsize_bar;
  changed = true;
}

////////////////////////////////////////////////////////////////////////////////

void LoadParameters(std::string file_name) {
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Failed to open ini parameters" << std::endl;
    exit(-1);
  }

  cv::Size cap_size;
  fs["cam_model"] >> cam_model;
  fs["cap_size" ] >> cap_size;
  fs["Kl"       ] >> Kl;
  fs["Dl"       ] >> Dl;
  fs["xil"      ] >> xil;
  Rl = cv::Mat::eye(3, 3, CV_64F);
  if (cam_model == "stereo") {
    fs["Rl"       ] >> Rl;
    fs["Kr"       ] >> Kr;
    fs["Dr"       ] >> Dr;
    fs["xir"      ] >> xir;
    fs["Rr"       ] >> Rr;
    fs["T"        ] >> Translation;
  }
  fs.release();

  img_width = cap_size.width;
  cap_cols  = cap_size.width;
  cap_rows  = cap_size.height;

  if (cam_model == "stereo")
    img_width  = cap_size.width / 2;
}

////////////////////////////////////////////////////////////////////////////////

void InitUndistortRectifyMap(cv::Mat K, cv::Mat D, cv::Mat xi, cv::Mat R, 
                             cv::Mat P, cv::Size size, 
                             cv::Mat& map1, cv::Mat& map2) {
  map1 = cv::Mat(size, CV_32F);
  map2 = cv::Mat(size, CV_32F);

  double fx = K.at<double>(0,0);
  double fy = K.at<double>(1,1);
  double cx = K.at<double>(0,2);
  double cy = K.at<double>(1,2);
  double s  = K.at<double>(0,1);

  double xid = xi.at<double>(0,0);

  double k1 = D.at<double>(0,0);
  double k2 = D.at<double>(0,1);
  double p1 = D.at<double>(0,2);
  double p2 = D.at<double>(0,3);

  cv::Mat KRi = (P * R).inv();

  for (int i = 0; i < size.height; ++i) {
    double x = i * KRi.at<double>(0,1) + KRi.at<double>(0,2);
    double y = i * KRi.at<double>(1,1) + KRi.at<double>(1,2);
    double w = i * KRi.at<double>(2,1) + KRi.at<double>(2,2);
    for (int j = 0; j < size.width; ++j, x += KRi.at<double>(0,0),
                                         y += KRi.at<double>(1,0),
                                         w += KRi.at<double>(2,0)) {
      double r  = sqrt(x * x + y * y + w * w);
      double xs = x / r;
      double ys = y / r;
      double zs = w / r;

      double xu = xs / (zs + xid);
      double yu = ys / (zs + xid);

      double r2 = xu * xu + yu * yu;
      double r4 = r2 * r2;
      double xd = (1 + k1 * r2 + k2 * r4) * xu + 2 * p1 * xu * yu
                                               + p2 * (r2 + 2 * xu * xu);
      double yd = (1 + k1 * r2 + k2 * r4) * yu + 2 * p2 * xu * yu
                                               + p1 * (r2 + 2 * yu * yu);

      double u = fx * xd + s * yd + cx;
      double v = fy * yd + cy;

      map1.at<float>(i,j) = (float) u;
      map2.at<float>(i,j) = (float) v;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void InitRectifyMap() {
  double vfov_rad = vfov_now * CV_PI / 180.;
  double focal = height_now / 2. / tan(vfov_rad / 2.);
  Knew = (cv::Mat_<double>(3, 3) << focal, 0., width_now  / 2. - 0.5,
                                    0., focal, height_now / 2. - 0.5,
                                    0., 0., 1.);

  cv::Size img_size(width_now, height_now);

  InitUndistortRectifyMap(Kl, Dl, xil, Rl, Knew, 
                          img_size, smap[0][0], smap[0][1]);

  std::cout << "Width: "  << width_now  << "\t"
            << "Height: " << height_now << "\t"
            << "V.Fov: "  << vfov_now   << "\n";
  std::cout << "K Matrix: \n" << Knew << std::endl;

  if (cam_model == "stereo") {
    InitUndistortRectifyMap(Kr, Dr, xir, Rr, Knew, 
                            img_size, smap[1][0], smap[1][1]);
    std::cout << "Ndisp: " << ndisp_now << "\t"
              << "Wsize: " << wsize_now << "\n";
  }
  std::cout << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void DisparityImage(const cv::Mat& recl, const cv::Mat& recr, cv::Mat& disp) {
  cv::Mat disp16s;
  int N = ndisp_now, W = wsize_now, C = recl.channels();
  if (is_sgbm) {
    cv::Ptr<cv::StereoSGBM> sgbm =
        cv::StereoSGBM::create(0, N, W, 8 * C * W * W, 32 * C * W * W);
    sgbm->compute(recl, recr, disp16s);
  } else {
    cv::Mat grayl, grayr;
    cv::cvtColor(recl, grayl, CV_BGR2GRAY);
    cv::cvtColor(recr, grayr, CV_BGR2GRAY);

    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(N, W);
    sbm->setPreFilterCap(31);
    sbm->setMinDisparity(0);
    sbm->setTextureThreshold(10);
    sbm->setUniquenessRatio(15);
    sbm->setSpeckleWindowSize(100);
    sbm->setSpeckleRange(32);
    sbm->setDisp12MaxDiff(1);
    sbm->compute(grayl, grayr, disp16s);
  }

  double minVal, maxVal;
  minMaxLoc(disp16s, &minVal, &maxVal);
  disp16s.convertTo(disp, CV_8UC1, 255 / (maxVal - minVal));
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  std::string file_name = argc == 2 ? argv[1] : "./calib.yml";
  LoadParameters(file_name);
  InitRectifyMap();

  for (int i = 1; i < 3900; i++) {
      cv::Mat raw_img;
      cv::VideoCapture vcapture;

      int count = 5 - patch::to_string(i).length();
      std::string fn("0");
      for (int j = 1; j < count; j++) {
      fn.append("0");
      }
      fn.append(patch::to_string(i));
      std::cout << fn;
      std::cout << '\n';
      raw_img = cv::imread("./st/" + fn + ".png", cv::IMREAD_COLOR);

    char win_name[256];
    sprintf(win_name, "Raw Image: %d x %d", img_width, cap_rows);
    std::string param_win_name(win_name);
    cv::namedWindow(param_win_name);
  
    cv::createTrackbar("V. FoV:  80    +", param_win_name,
                       &vfov_bar,   vfov_max,   OnTrackAngle);
    cv::createTrackbar("Width:  480 +", param_win_name,
                       &width_bar,  width_max,  OnTrackWidth);
    cv::createTrackbar("Height: 360 +", param_win_name,
                       &height_bar, height_max, OnTrackHeight);

    cv::Mat raw_imgl, raw_imgr, rect_imgl, rect_imgr;

    if (raw_img.total() == 0) {
      std::cout << "Image capture error" << std::endl;
      exit(-1);
    }

    if (cam_model == "stereo") {
      raw_img(cv::Rect(        0, 0, img_width, cap_rows)).copyTo(raw_imgl);
      raw_img(cv::Rect(img_width, 0, img_width, cap_rows)).copyTo(raw_imgr);

      cv::remap(raw_imgl, rect_imgl, smap[0][0], smap[0][1], 1, 0);
      cv::remap(raw_imgr, rect_imgr, smap[1][0], smap[1][1], 1, 0);
    } else {
      raw_imgl = raw_img;
      cv::remap(raw_img, rect_imgl, smap[0][0], smap[0][1], 1, 0);
    }


    cv::imwrite("./cam0/" + fn + ".png", rect_imgl);
    cv::imwrite("./cam0/" + fn + ".png", rect_imgr);
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////



