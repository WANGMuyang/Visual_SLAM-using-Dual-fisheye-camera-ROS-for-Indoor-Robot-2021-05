#ifndef OPENVSLAM_DATA_STATION_H
#define OPENVSLAM_DATA_STATION_H

#include "openvslam/type.h"

#include <map>
#include <mutex>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <nlohmann/json_fwd.hpp>

namespace openvslam {
namespace data {

class frame;

class keyframe;

class map_database;

class station {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    station(const Vec3_t& pos_w, map_database* map_db);

    //! constructor for map loading with computing parameters which can be recomputed
    station(const Vec3_t& pos_w, const unsigned int id, map_database* map_db);

    //! set world coordinates of this station
    void set_pos_of_station_in_world(const Vec3_t& pos_w);
    //! get world coordinates of this station
    Vec3_t get_pos_of_station_in_world() const;

    //! erase this station from database
    void prepare_for_erasing_station();

    //! encode station information as JSON
    nlohmann::json to_json() const;

public:
    unsigned int id_;
    static std::atomic<unsigned int> next_id_;

private:
    //! world coordinates of this station
    Vec3_t pos_w_;

    //! map database
    map_database* map_db_;

    mutable std::mutex mtx_position_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_STATION_H
