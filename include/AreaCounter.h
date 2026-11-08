#ifndef AREA_COUNTER_H_
#define AREA_COUNTER_H_

#include <set>
#include <vector>
#include "tracker/Tracklet.h"

/**
 * @class AreaCounter
 * @brief Definisce un'area di monitoraggio rettangolare e conta le persone distinte
 *        (tracklet) che vi sono entrate.
 *
 * L'area è definita nel piano 2D (x, y) dai suoi vertici di minimo e massimo. 
 * Ogni "tracklet" che entra nell'area viene conteggiato esattamente una volta, anche se
 * esce e rientra.
 */
class AreaCounter
{
public:
    /**
     * @brief Costruttore che definisce un AreaCounter con i suoi 4 lati.
     * @param x_min  Lato sinistro dell'area.
     * @param x_max  Lato destro dell'area.
     * @param y_min  Lato inferiore dell'area.
     * @param y_max  Lato superiore dell'area.
     */
    AreaCounter(double x_min, double x_max, double y_min, double y_max);
    ~AreaCounter();

    /**
     * @brief Aggiorna il contatore verificando quali tracklet si trovano attualmente
     *        all'interno dell'area. Gli ID dei tracklet rilevati come inediti vengono registrati.
     * @param tracks L'elenco attuale dei tracklet attivi.
     */
    void update(const std::vector<Tracklet> &tracks);

    /**
     * @brief Ottieni il numero di persone distinte che sono entrate nell'area.
     * @return Il numero di tracklet univoci che sono nell'area.
     */
    int getCount() const;

    /**
     * @brief Ottieni l'insieme degli ID dei tracklet attualmente presenti nell'area.
     * @return Insieme degli ID di tracklet che si trovano all'interno dell'area.
     */
    const std::set<int> &getCurrentInsideIds() const;

    // Getters for the area boundaries (used for rendering)
    double getXMin() const { return x_min_; }
    double getXMax() const { return x_max_; }
    double getYMin() const { return y_min_; }
    double getYMax() const { return y_max_; }

private:
    // Bordi dell'area rettangolare
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;

    // Set dei traklet ID che sono entrati nell'area almeno una volta
    std::set<int> entered_ids_;

    // Set dei traklet ID che sono attualmente all'interno dell'area
    std::set<int> current_inside_ids_;

    /**
     * @brief Verifica se un punto (x,y) risiede nell'area.
     */
    bool isInside(double x, double y) const;
};

#endif // AREA_COUNTER_H_
