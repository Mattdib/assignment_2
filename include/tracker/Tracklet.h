#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include "KalmanFilter.h"

class Tracklet
{
  
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  double getPathLength() const;
  double getX() { return kf_.getX(); }
  double getY() { return kf_.getY(); }
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  int getLossCount() { return loss_count_; }
  int getId() const{ return id_; }

private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;

  //vettore che memorizza la traiettoria del tracklet, ovvero le posizioni (coordinate x e y) del tracklet nel tempo. Ogni volta che il tracklet viene aggiornato con una nuova misura reale del sensore, la posizione corrente viene aggiunta a questo vettore.
  std::vector<Eigen::Vector2d> trajectory_;
};

#endif // TRACKLET_H_
