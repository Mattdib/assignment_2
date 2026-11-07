#include "AreaCounter.h"

AreaCounter::AreaCounter(double x_min, double x_max, double y_min, double y_max)
    : x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max)
{
}

AreaCounter::~AreaCounter()
{
}

//funzione che verifica se un tracklet è all'interno dell'area
bool AreaCounter::isInside(double x, double y) const
{
    return (x >= x_min_ && x <= x_max_ && y >= y_min_ && y <= y_max_); //verifico che le coordinate del tracklet siano comprese all'interno dell'area
}

//funzione eseguita ad ogni frame, verificando quali tracklet sono attualmente all'interno dell'area
void AreaCounter::update(const std::vector<Tracklet> &tracks)
{
    current_inside_ids_.clear();

    for (const auto &track : tracks)
    {
        double x = track.getX(); //prendo la componenti x di un tracklet
        double y = track.getY(); //prendo la componenti y di un tracklet
        int id = track.getId();

        if (isInside(x, y))
        {
            current_inside_ids_.insert(id);

            // registro l'ID del tracklet (essendo un set, se esso era già entrato nell'area in un frame precedente, esso non verrà registrato nuovamente)
            entered_ids_.insert(id);
        }
    }
}

//restituisce il numero di persone uniche presenti nell'area
int AreaCounter::getCount() const
{
    return static_cast<int>(entered_ids_.size()); 
}

//restituisce il riferiamento al set di ID dei trackleti attualmente dentro l'area
const std::set<int> &AreaCounter::getCurrentInsideIds() const
{
    return current_inside_ids_;
}
