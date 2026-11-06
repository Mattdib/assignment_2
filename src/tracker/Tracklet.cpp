#include "tracker/Tracklet.h"
#include <cmath>
#include <iostream>

Tracklet::Tracklet(int idTrack, double x, double y)
{
  // set id
  id_ = idTrack; 

  // initialize filter
  kf_.init(0.1); // inizializzo il filtro di Kalman con un delta time di 0.1 secondi
  kf_.setState(x, y); // setto lo stato iniziale del filtro di Kalman con le coordinate x e y

  trajectory_.push_back(Eigen::Vector2d(x, y)); // aggiungo le coordinate iniziali del tracklet al vettore della traiettoria

  // set loss count to 0
  loss_count_ = 0; // inizializzo il contatore a 0, indica il numero di frame in cui il tracklet non è stato aggiornato con una misura reale del sensore
}

Tracklet::~Tracklet()
{
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict(); // predizione dello stato successivo del filtro di Kalman
  loss_count_++;
}

double Tracklet::getPathLength() const {
    if (trajectory_.size() < 2) {
        return 0.0;
    }

    double total_length = 0.0;
    // Soglia minima di spostamento per considerare il movimento reale (es. 5-10 cm)
    const double min_movement_threshold = 0.15; // 15 cm

    Eigen::Vector2d last_valid_point = trajectory_[0];

    for (size_t i = 1; i < trajectory_.size(); ++i) {
        double step_distance = (trajectory_[i] - last_valid_point).norm();

        if (step_distance >= min_movement_threshold) {
            total_length += step_distance;
            last_valid_point = trajectory_[i]; // Sposta l'ancora al nuovo punto valido
        }
    }

    return total_length;
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus)
{
  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2); // vettore delle misure grezze, che contiene le coordinate x e y misurate del sensore

  // measurement update
  if (lidarStatus) // se il sensore LIDAR è attivo, aggiorno lo stato del filtro di Kalman con le misure grezze
  {
    raw_measurements_ << x, y; // setto le misure grezze con le coordinate x e y misurate del sensore
    kf_.update(raw_measurements_); //aggiorno lo stato del filtro di Kalman con le misure grezze
    trajectory_.push_back(Eigen::Vector2d(kf_.getX(), kf_.getY())); //aggiungo al vettore della traiettoria le coordinate x e y stimate del filtro di Kalman al passo corrente
    loss_count_ = 0; // resetto il contatore a 0, indica che il tracklet è stato aggiornato con una misura reale del sensore
  }

  /*
  // Stampa di verifica temporanea dentro Tracklet::update
  std::cout << "[Track " << id_ << "] Punti registrati: " << trajectory_.size()
            << " - Posizione stimata: (" << trajectory_.back().x() << ", " 
            << trajectory_.back().y() << ")" << std::endl;
  */
}
