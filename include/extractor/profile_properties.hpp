#ifndef PROFILE_PROPERTIES_HPP
#define PROFILE_PROPERTIES_HPP

#include <algorithm>
#include <boost/assert.hpp>
#include <boost/numeric/conversion/cast.hpp>

namespace osrm
{
namespace extractor
{

const constexpr auto DEFAULT_MAX_SPEED = 180 / 3.6; // 180kmph -> m/s

struct ProfileProperties
{
    enum
    {
        MAX_WEIGHT_NAME_LENGTH = 255
    };

    ProfileProperties()
        : max_speed_for_map_matching(DEFAULT_MAX_SPEED), traffic_signal_penalty(0),
          continue_straight_at_waypoint(true), use_turn_restrictions(false),
          left_hand_driving(false), weight_name{"duration"}
    {
        BOOST_ASSERT(weight_name[MAX_WEIGHT_NAME_LENGTH] == '\0');
    }

    double GetTrafficSignalPenalty() const { return traffic_signal_penalty / 10.; }

    void SetTrafficSignalPenalty(const double traffic_signal_penalty_)
    {
        traffic_signal_penalty = boost::numeric_cast<int>(traffic_signal_penalty_ * 10.);
    }

    double GetMaxSpeedForMapMatching() const { return max_speed_for_map_matching; }

    void SetMaxSpeedForMapMatching(const double max_speed_for_map_matching_)
    {
        max_speed_for_map_matching = max_speed_for_map_matching_;
    }

    void SetWeightName(const std::string &name)
    {
        auto count = std::min<std::size_t>(name.length(), MAX_WEIGHT_NAME_LENGTH) + 1;
        std::copy_n(name.c_str(), count, weight_name);
        // Make sure this is always zero terminated
        BOOST_ASSERT(weight_name[count - 1] == '\0');
        BOOST_ASSERT(weight_name[MAX_WEIGHT_NAME_LENGTH] == '\0');
    }

    std::string GetWeightName() const
    {
        // Make sure this is always zero terminated
        BOOST_ASSERT(weight_name[MAX_WEIGHT_NAME_LENGTH] == '\0');
        return std::string(weight_name);
    }

    double max_speed_for_map_matching;
    //! penalty to cross a traffic light in deci-seconds
    std::int32_t traffic_signal_penalty;
    //! depending on the profile, force the routing to always continue in the same direction
    bool continue_straight_at_waypoint;
    //! flag used for restriction parser (e.g. used for the walk profile)
    bool use_turn_restrictions;
    bool left_hand_driving;
    //! stores the name of the weight (e.g. 'duration', 'distance', 'safety')
    char weight_name[MAX_WEIGHT_NAME_LENGTH + 1];
};
}
}

#endif
