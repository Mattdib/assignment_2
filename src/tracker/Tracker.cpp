#include "tracker/Tracker.h"
#include <cmath>
#include <limits>
#include <algorithm>

Tracker::Tracker()
{
    cur_id_ = 0;
    //distance_threshold_ = 3.0;    // Soglia sulla distanza di Mahalanobis (~3 sigma, gate Chi-quadro con 2 DoF al ~99% di confidenza)
    distance_threshold_ = 1.0;
    covariance_threshold = 5.0;   // Soglia massima per la covarianza del filtro di Kalman, che rapprenta l'incertezza della stima della posizione del tracklet. Se la covarianza supera questa soglia, il tracklet viene considerato non affidabile e quindi rimosso.
    loss_threshold = 5;           // Numero massimo di frame consecutivi senza associazione
}

Tracker::~Tracker()
{
}

/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks()
{
    std::vector<Tracklet> tracks_to_keep;

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        // Verifica se la traccia non è stata associata per un numero eccessivo di frame
        bool lost_frames_exceeded = (tracks_[i].getLossCount() >= loss_threshold);

        // Verifica se la covarianza su x o su y supera la soglia consentita
        bool covariance_exceeded = (tracks_[i].getXCovariance() > covariance_threshold || 
                                    tracks_[i].getYCovariance() > covariance_threshold);

        // Mantiene la traccia solo se entrambi i criteri sono rispettati
        bool logic_to_keep = !lost_frames_exceeded && !covariance_exceeded;
        if (logic_to_keep)
        {
            tracks_to_keep.push_back(tracks_[i]); // Aggiunge la traccia alla lista di quelle da mantenere
        }
    }

    tracks_.swap(tracks_to_keep); // Sostituisce il vettore originale con quello filtrato, rimuovendo le tracce obsolete

}//removeTracks

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections, 
                        const std::vector<double> &centroids_x, 
                        const std::vector<double> &centroids_y)
{
    // Aggiunge le rilevazioni non associate come nuovi tracklet
    for (size_t i = 0; i < associated_detections.size(); ++i)
    {
        if (!associated_detections[i]) // Se la rilevazione non è stata associata a nessuna traccia esistente
        {
            // Crea un nuovo tracklet con un ID unico e le coordinate del centroide
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
        }
    }

}//addTracks

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection (an empty vector to host the associated detection)
        centroids_x & centroids_y (measurements representing the detected objects)
*/

/* 
Mahalanobis distance IMPLEMENTATION:
void Tracker::dataAssociation(std::vector<bool> &associated_detections,
                              const std::vector<double> &centroids_x,
                              const std::vector<double> &centroids_y)
{
    associated_track_det_ids_.clear();

    // Matrice di covarianza del rumore di misura LiDAR R = diag(0.0225, 0.0225) (sigma = 0.15 m),
    // definita nel KalmanFilter. Viene inclusa nella covarianza dell'innovazione S = H*P*H^T + R.
    const double R_x = 0.0225;
    const double R_y = 0.0225;

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        // Lettura della posizione stimata tramite i getter di Tracklet
        // (recupero la posizione predetta dal Kalman Filter al passo corrente)
        double track_x = tracks_[i].getX();
        double track_y = tracks_[i].getY();

        // Matrice di covarianza dell'innovazione S = H * P * H^T + R:
        // Nel modello cinematico 2D disaccoppiato (x e y indipendenti), S è diagonale:
        // S_xx = P_xx + R_x
        // S_yy = P_yy + R_y
        // Dove P_xx e P_yy sono le covarianze stimate della traccia lungo x e y.
        // Si applica un limite inferiore (clamp) a 1e-6 per evitare divisioni per zero.
        double S_x = std::max(tracks_[i].getXCovariance() + R_x, 1e-6);
        double S_y = std::max(tracks_[i].getYCovariance() + R_y, 1e-6);

        double inv_S_x = 1.0 / S_x;
        double inv_S_y = 1.0 / S_y;

        for (size_t j = 0; j < associated_detections.size(); ++j)
        {
            if (!associated_detections[j]) // Se una misurazione al frame corrente non è ancora stata associata
            {
                // Vettore errore di predizione (innovazione): d = misurazione - predizione
                double dx = centroids_x[j] - track_x;
                double dy = centroids_y[j] - track_y;

                // Calcolo della distanza di Mahalanobis:
                // D_M = sqrt( d^T * S^(-1) * d ) = sqrt( (dx^2 / S_xx) + (dy^2 / S_yy) )
                double mahalanobis_dist = std::sqrt(dx * dx * inv_S_x + dy * dy * inv_S_y);

                if (mahalanobis_dist < min_dist)
                {
                    min_dist = mahalanobis_dist;
                    closest_point_id = static_cast<int>(j); // associo l'indice della misurazione con minore distanza di Mahalanobis
                }
            }
        }

        // Associa il rilevamento più vicino se entro la soglia di Mahalanobis (gating Chi-quadro)
        if (closest_point_id != -1 && min_dist <= distance_threshold_)
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, static_cast<int>(i))); // associazione (det_id, track_id)
            associated_detections[closest_point_id] = true; // Segno il rilevamento come associato per evitare associazioni multiple
        }
    }

} //dataAssociation
*/

// Euclidean distance IMPLEMENTATION:
void Tracker::dataAssociation(std::vector<bool> &associated_detections,
                              const std::vector<double> &centroids_x,
                              const std::vector<double> &centroids_y){
    associated_track_det_ids_.clear();

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        // Lettura della posizione stimata tramite i getter di Tracklet (recupero la posizione teorica di un cluster predetta dal Kalman Filter (fase di predizione) al passo corrente)
        double track_x = tracks_[i].getX();
        double track_y = tracks_[i].getY();

        for (size_t j = 0; j < associated_detections.size(); ++j)
        {
            if (!associated_detections[j]) // Se una misurazione al passo/frame corrente non è stata ancora associata a nessuna traccia
            {
                // Calcolo della distanza euclidea tra predizione della traccia e la misurazione j-esima (centroide del cluster j-esimo rilevato al passo/frame corrente)
                double dx = track_x - centroids_x[j];
                double dy = track_y - centroids_y[j];
                double dist = std::hypot(dx, dy);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest_point_id = static_cast<int>(j); // associo l'indice della misurazione più vicina alla traccia i-esima
                }
            }
        }

        // Associa il rilevamento più vicino se entro la threshold
        if (closest_point_id != -1 && min_dist <= distance_threshold_)
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, static_cast<int>(i))); // associo l'indice del rilevamento/centroide con l'indice della traccia
            associated_detections[closest_point_id] = true; // Segno il rilevamento come associato per evitare che venga associato a un'altra traccia
        }
    }

}

std::pair<int, double> Tracker::getLongestTrack() const {
    int best_id = -1;
    double max_length = -1.0;

    auto check_track = [&](const Tracklet& track) {
        double len = track.getPathLength();
        if (len > max_length) {
            max_length = len;
            best_id = track.getId();
        }
    };

    // Controllo tracce attualmente attive
    for (const auto& track : tracks_) {
        check_track(track);
    }

    if (best_id == -1) {
        return {-1, 0.0}; // Nessuna traccia valida trovata
    }

    return {best_id, max_length};

} //getLongestTrack

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus)
{
    std::vector<bool> associated_detections(centroids_x.size(), false);

    // 1. Predizione dello stato per ciascun tracklet
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        tracks_[i].predict();
    }

    // 2. Associazione delle predizioni con i rilevamenti attuali
    dataAssociation(associated_detections, centroids_x, centroids_y);

    // 3. Aggiornamento delle tracce abbinate
    for (size_t i = 0; i < associated_track_det_ids_.size(); ++i)
    {
        auto det_id = associated_track_det_ids_[i].first;
        auto track_id = associated_track_det_ids_[i].second;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);
    }

    // 4. Rimozione dei tracklet obsoleti o con covarianza eccessiva
    removeTracks();

    // 5. Inizializzazione di nuovi tracklet per i rilevamenti non associati
    addTracks(associated_detections, centroids_x, centroids_y);

} //track