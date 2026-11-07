#ifndef AREA_COUNTER_H_
#define AREA_COUNTER_H_

#include <set>
#include <vector>
#include "tracker/Tracklet.h"

/**
 * @class AreaCounter
 * @brief Defines a rectangular monitoring area and counts unique persons
 *        (tracklets) that have entered it.
 *
 * The area is defined in the 2D (x, y) plane by its min/max corners.
 * Each tracklet that enters the area is counted exactly once, even if
 * it leaves and re-enters.
 */
class AreaCounter
{
public:
    /**
     * @brief Construct an AreaCounter with the given rectangular bounds.
     * @param x_min  Left boundary of the area.
     * @param x_max  Right boundary of the area.
     * @param y_min  Bottom boundary of the area.
     * @param y_max  Top boundary of the area.
     */
    AreaCounter(double x_min, double x_max, double y_min, double y_max);
    ~AreaCounter();

    /**
     * @brief Update the counter by checking which tracklets are currently
     *        inside the area. Newly detected tracklet IDs are recorded.
     * @param tracks  The current list of active tracklets from the Tracker.
     */
    void update(const std::vector<Tracklet> &tracks);

    /**
     * @brief Get the number of unique persons that have entered the area.
     * @return The count of unique tracklet IDs that have been inside the area.
     */
    int getCount() const;

    /**
     * @brief Get the set of tracklet IDs currently inside the area.
     * @return A set of IDs of tracklets that are inside the area right now.
     */
    const std::set<int> &getCurrentInsideIds() const;

    // Getters for the area boundaries (used for rendering)
    double getXMin() const { return x_min_; }
    double getXMax() const { return x_max_; }
    double getYMin() const { return y_min_; }
    double getYMax() const { return y_max_; }

private:
    // Rectangular area boundaries
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;

    // Set of unique tracklet IDs that have ever entered the area
    std::set<int> entered_ids_;

    // Set of tracklet IDs currently inside the area
    std::set<int> current_inside_ids_;

    /**
     * @brief Check whether a point (x, y) is inside the defined area.
     */
    bool isInside(double x, double y) const;
};

#endif // AREA_COUNTER_H_
