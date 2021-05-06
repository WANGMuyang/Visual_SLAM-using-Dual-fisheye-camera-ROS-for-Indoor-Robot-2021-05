#include "openvslam/data/station.h"
#include "openvslam/data/map_database.h"

#include <nlohmann/json.hpp>

namespace openvslam {
namespace data {

std::atomic<unsigned int> station::next_id_{0};

station::station(const Vec3_t& pos_w, map_database* map_db)
    : id_(next_id_++), pos_w_(pos_w), map_db_(map_db){}

station::station(const Vec3_t& pos_w, const unsigned int id, map_database* map_db)
    : id_(id), pos_w_(pos_w), map_db_(map_db) {}

void station::set_pos_of_station_in_world(const Vec3_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    pos_w_ = pos_w;
}

Vec3_t station::get_pos_of_station_in_world() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return pos_w_;
}

void station::prepare_for_erasing_station() {
    // std::lock_guard<std::mutex> lock1(mtx_observations_);
    // std::lock_guard<std::mutex> lock2(mtx_position_);
    map_db_->erase_station(this);
}


nlohmann::json station::to_json() const {
    return {{"pos_w", {pos_w_(0), pos_w_(1), pos_w_(2)}}};  // 두번 감싸야한다!!
}

} // namespace data
} // namespace openvslam
