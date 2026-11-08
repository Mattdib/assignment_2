#include "tracker/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::init(double dt) // inizializzo il Kalman Filter con il delta time dt (intervallo di tempo tra due misurazioni consecutive)
{
  dt_ = dt;

  // create a 4D state vector
  x_ = Eigen::VectorXd(4); //inizializzo un vettore di stato a 4 dimensioni (posizione lungo x, posizione lungo y, velocità lungo x, velocità lungo y)

  // TODO: Initialize the state covariance matrix P
  P_ = Eigen::MatrixXd(4, 4); //inizializzo la matrice di covarianza dello stato a 4 dimensioni (posizione lungo x, posizione lungo y, velocità lungo x, velocità lungo y)
  P_ << 9999., 0., 0., 0.,
      0., 9999., 0., 0.,
      0., 0., 9999., 0.,
      0., 0., 0., 9999.;

  // measurement covariance (varianza della misura del sensore pari a 0.0225 m^2, quindi deviazione standard pari a 0.15 m)
  R_ = Eigen::MatrixXd(2, 2); //inizializzo la matrice di covarianza della misura a 2 dimensioni (posizione lungo x, posizione lungo y)
  R_ << 0.0225, 0.,
      0., 0.0225;
  
  /*
  // caso sovraffidamento del sensore (varianza della misura del sensore pari a 0.001 m^2, quindi deviazione standard pari a 0.0316 m)
  R_ = Eigen::MatrixXd(2, 2); //inizializzo la matrice di covarianza della misura a 2 dimensioni (posizione lungo x, posizione lungo y)
  R_ << 0.001, 0.,
      0., 0.001;
  */

  /*
  // caso eccessiva fiducia del modello, ossia LiDAR molto rumoroso (varianza della misura del sensore pari a 0.5 m^2, quindi deviazione standard pari a 0.7071 m)
  R_ = Eigen::MatrixXd(2, 2); //inizializzo la matrice di covarianza della misura a 2 dimensioni (posizione lungo x, posizione lungo y)
  R_ << 0.5, 0.,
      0., 0.5;
  */

  // measurement matrix
  H_ = Eigen::MatrixXd(2, 4); //inizializzo la matrice di misura, dove seleziono solo le prime due dimensioni dello stato (posizione lungo x, posizione lungo y) e ignoro le altre due dimensioni (velocità lungo x, velocità lungo y)
  H_ << 1., 0., 0., 0.,
      0., 1., 0., 0.;

  // the transition matrix F
  F_ = Eigen::MatrixXd(4, 4); //inizializzo la matrice di transizione dello stato, dove considero il delta time dt per aggiornare la posizione in base alla velocità e alla posizione precedente
  F_ << 1., 0., dt_, 0.,
      0., 1., 0., dt_,
      0., 0., 1., 0.,
      0., 0., 0., 1.;
  
  // set the acceleration noise components
  double noise_ax_ = 2.; 
  double noise_ay_ = 2.;

  /*
  // eccessiva fiducia del modello (si assume che il pedone mantenga velocità costante)
  double noise_ax_ = 0.05; 
  double noise_ay_ = 0.05;
  */

  double dt_2 = dt_ * dt_;
  double dt_3 = dt_2 * dt_;
  double dt_4 = dt_3 * dt_;

  // set the process covariance matrix Q
  Q_ = Eigen::MatrixXd(4, 4); //inizializzo la matrice di covarianza del processo, dove considero il rumore di accelerazione lungo x e y per aggiornare la posizione e la velocità in base al delta time dt
  Q_ << dt_4 / 4. * noise_ax_, 0., dt_3 / 2. * noise_ax_, 0.,
      0., dt_4 / 4. * noise_ay_, 0., dt_3 / 2. * noise_ay_,
      dt_3 / 2. * noise_ax_, 0., dt_2 * noise_ax_, 0.,
      0., dt_3 / 2. * noise_ay_, 0., dt_2 * noise_ay_;
}

void KalmanFilter::predict()
{
  // TODO
  // Implement Kalman Filter Predict
  x_ = F_ * x_; //predizione dello stato successivo
  P_ = F_ * P_ * F_.transpose() + Q_; //predizione della matrice di covarianza dello stato successivo
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
  // TODO
  // Implement Kalman Filter Update

  Eigen::VectorXd y = z - H_ * x_; //calcolo dell'errore di misura (differenza tra la misura del sensore e la predizione dello stato del modello cinematico)
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; //calcolo della matrice di covarianza dell'errore di misura
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); //calcolo del guadagno di Kalman (quanto peso dare alla misura del sensore rispetto alla predizione dello stato del modello cinematico)

  // new estimate
  x_ = x_ + (K * y); //aggiornamento dello stato calcolato dal modello cinematico in base alla misura ottenuta dal sensore e al guadagno di Kalman
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size()); //creo la matrice indentità utile al calcolo successivo
  P_ = (I - K * H_) * P_; //aggiornamento della matrice di covarianza dello stato
}

void KalmanFilter::setState(double x, double y) //setto lo stato iniziale del filtro di Kalman con le coordinate x e y
{
  x_ << x, y, 0., 0.;
}