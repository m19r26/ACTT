#include "actt_pc/pose_2d.hpp"
#include "actt_pc/scan_simulator_2d.hpp"
#include "actt_pc/distance_transform.hpp"

using namespace actt_auto;

ScanSimulator2D::ScanSimulator2D(
    int num_beams_, 
    double field_of_view_, 
    double scan_std_dev_, 
    double ray_tracing_epsilon_,
    int theta_discretization) 
  : num_beams(num_beams_),
    field_of_view(field_of_view_),
    scan_std_dev(scan_std_dev_),
    ray_tracing_epsilon(ray_tracing_epsilon_),
    theta_discretization(theta_discretization)
{
   // LiDARの設定の初期化
   // 角度増分(rad/ビーム) = LiDARの測定可能角度(6.28rad) / ビーム数
   angle_increment = field_of_view/(num_beams - 1); 

   // 出力の初期化
   scan_output = std::vector<double>(num_beams);

   // ノイズの初期化
   noise_generator = std::mt19937(std::random_device{}());
   noise_dist = std::normal_distribution<double>(0., scan_std_dev);

   // sinとcosを事前に計算
   // thetaインデックスの増加 = thetaの離散化(2000) * 角度増分/単位円の円周(2*3.14)
   theta_index_increment = theta_discretization * angle_increment/(2 * M_PI); 
   // sines, cosinesという動的なサイズの配列を生成
   sines = std::vector<double>(theta_discretization + 1);
   cosines = std::vector<double>(theta_discretization + 1);
   for (int i = 0; i <= theta_discretization; i++) {
      // theta = 2*3.14*i番目 / thetaの離散化(2000)
      double theta = (2 * M_PI * i)/((double) theta_discretization);
      sines[i] = std::sin(theta);
      cosines[i] = std::cos(theta);
   }
}

const std::vector<double> ScanSimulator2D::scan(const Pose2D & pose) {
  scan(pose, scan_output.data());
  return scan_output;
}

void ScanSimulator2D::scan(const Pose2D & pose, double * scan_data) {
   // [-pi, pi]を[0, theta_discretization]にマッピングしてthetaを離散化
   double theta_index = 
          theta_discretization * (pose.theta - field_of_view/2.)/(2 * M_PI);

   // 正しくラップされていることの確認
   // index = index - ((int)index-discretization)　* discretization
   theta_index = std::fmod(theta_index, theta_discretization);
   while (theta_index < 0) theta_index += theta_discretization;

   // 各々のビームをスウィープ
   for (int i = 0; i < num_beams; i++) {
      // 最も近いポイントの距離を計算
      scan_data[i] = trace_ray(pose.x, pose.y, theta_index);

      // レイトレースのためにガウシアンノイズを追加
      if (scan_std_dev > 0)
         scan_data[i] += noise_dist(noise_generator);

      // スキャンを増加
      theta_index += theta_index_increment;
      // 範囲[0, theta_discretization]内にあるか確認
      while (theta_index >= theta_discretization) 
         theta_index -= theta_discretization;
   }
}

double ScanSimulator2D::trace_ray(double x, double y, double theta_index) const {
   // 四捨五入するために0.5を足す
   int    theta_index_ = theta_index + 0.5;
   double s            = sines[theta_index_];
   double c            = cosines[theta_index_];

   // 最も近い障害物との距離を初期化
   double distance_to_nearest = distance_transform(x, y);
   double total_distance      = distance_to_nearest;

   while (distance_to_nearest > ray_tracing_epsilon) { // ray->0.0001
      // distance_to_nearestによって光線方向へ移動
      x += distance_to_nearest * c;
      y += distance_to_nearest * s;
    
      // そのポイントで最も近い距離を計算
      distance_to_nearest = distance_transform(x, y);
      total_distance += distance_to_nearest;
  }

  return total_distance;
}

double ScanSimulator2D::distance_transform(double x, double y) const {
   // 状態をグリッドセルに変換
   int cell = xy_to_cell(x, y);
   if (cell < 0) return 0;

   return dt[cell];
}

void ScanSimulator2D::set_map(
    const std::vector<double> & map, 
    size_t height_, 
    size_t width_, 
    double resolution_,
    const Pose2D & origin_,
    double free_threshold) {

  // Assign parameters
  height = height_;
  width = width_;
  resolution = resolution_;
  origin = origin_;
  origin_c = std::cos(origin.theta);
  origin_s = std::sin(origin.theta);

  // Threshold the map
  dt = std::vector<double>(map.size());
  for (size_t i = 0; i < map.size(); i++) {
    if (0 <= map[i] and map[i] <= free_threshold) {
      dt[i] = 99999; // Free
    } else {
      dt[i] = 0; // Occupied
    }
  }
  DistanceTransform::distance_2d(dt, width, height, resolution);
}

void ScanSimulator2D::xy_to_row_col(double x, double y, int * row, int * col) const {
   // 原点で状態を変換
   // trans = 現在のLiDAR位置 - 原点
   double x_trans = x - origin.x;
   double y_trans = y - origin.y;

   // 状態をマップ上で回転
   // x_rot = x_trans * x成分のtheta + y_trans　* y成分のtheta
   double x_rot =   x_trans * origin_c + y_trans * origin_s;
   double y_rot = - x_trans * origin_s + y_trans * origin_c;

   // 状態を切り取ってセルにする
   if(x_rot < 0 or x_rot >= width * resolution or
      y_rot < 0 or y_rot >= height * resolution) {
      *col = -1;
      *row = -1;
   } else { // 状態を行と列に分離
      *col = std::floor(x_rot/resolution); // 切り捨て
      *row = std::floor(y_rot/resolution);
   }
}

int ScanSimulator2D::row_col_to_cell(int row, int col) const {
  return row * width + col;
}

int ScanSimulator2D::xy_to_cell(double x, double y) const {
   int row, col;
   xy_to_row_col(x, y, &row, &col);
   return row_col_to_cell(row, col);
}

