#ifndef ROUTE_STEP_HPP
#define ROUTE_STEP_HPP

#include "extractor/travel_mode.hpp"
#include "engine/guidance/step_maneuver.hpp"
#include "util/coordinate.hpp"
#include "util/guidance/bearing_class.hpp"
#include "util/guidance/entry_class.hpp"

#include "extractor/guidance/turn_lane_types.hpp"
#include "util/guidance/turn_lanes.hpp"

#include <cstddef>
#include <string>
#include <vector>

#include <boost/range/iterator_range.hpp>

namespace osrm
{
namespace engine
{
namespace guidance
{
// Given the following turn from a,b to b,c over b:
//  a --> b --> c
// this struct saves the information of the segment b,c.
// Notable exceptions are Departure and Arrival steps.
// Departure: s --> a --> b. Represents the segment s,a with location being s.
// Arrive: a --> b --> t. The segment (b,t) is already covered by the previous segment.

// A representation of intermediate intersections
struct IntermediateIntersection
{
    static const constexpr std::size_t NO_INDEX = std::numeric_limits<std::size_t>::max();
    util::Coordinate location;
    std::vector<short> bearings;
    std::vector<bool> entry;
    std::size_t in;
    std::size_t out;

    // turn lane information
    util::guidance::LaneTuple lanes;
    extractor::guidance::TurnLaneDescription lane_description;
};

inline IntermediateIntersection getInvalidIntersection()
{
    return {util::Coordinate{util::FloatLongitude{0.0}, util::FloatLatitude{0.0}},
            {},
            {},
            IntermediateIntersection::NO_INDEX,
            IntermediateIntersection::NO_INDEX,
            util::guidance::LaneTuple(),
            {}};
}

struct RouteStep
{
    unsigned name_id;
    std::string name;
    std::string ref;
    std::string pronunciation;
    std::string destinations;
    std::string rotary_name;
    std::string rotary_pronunciation;
    double duration;
    double distance;
    double weight;
    extractor::TravelMode mode;
    StepManeuver maneuver;
    // indices into the locations array stored the LegGeometry
    std::size_t geometry_begin;
    std::size_t geometry_end;
    std::vector<IntermediateIntersection> intersections;

    // remove all information from the route step, marking it as invalid (used to indicate empty
    // steps to be removed).
    void Invalidate();

    // Elongate by another step in front
    RouteStep &AddInFront(const RouteStep &preceeding_step);

    // Elongate by another step in back
    RouteStep &ElongateBy(const RouteStep &following_step);

    /* Elongate without prior knowledge of in front, or in back, convenience function if you
     * don't know if step is augmented in front or at the back */
    RouteStep &MergeWith(const RouteStep &by_step);

    // copy all strings from origin into the step, apart from rotary names
    RouteStep &AdaptStepSignage(const RouteStep &origin);

    LaneID NumLanesToTheRight() const;

    LaneID NumLanesToTheLeft() const;

    auto LanesToTheLeft() const;

    auto LanesToTheRight() const;
};

inline void RouteStep::Invalidate()
{
    name_id = EMPTY_NAMEID;
    name.clear();
    ref.clear();
    pronunciation.clear();
    destinations.clear();
    rotary_name.clear();
    rotary_pronunciation.clear();
    duration = 0;
    distance = 0;
    mode = TRAVEL_MODE_INACCESSIBLE;
    maneuver = getInvalidStepManeuver();
    geometry_begin = 0;
    geometry_end = 0;
    intersections.clear();
    intersections.push_back(getInvalidIntersection());
}

// Elongate by another step in front
inline RouteStep &RouteStep::AddInFront(const RouteStep &preceeding_step)
{
    BOOST_ASSERT(preceeding_step.geometry_end == geometry_begin + 1);
    BOOST_ASSERT(mode == preceeding_step.mode);
    duration += preceeding_step.duration;
    distance += preceeding_step.distance;

    geometry_begin = preceeding_step.geometry_begin;
    intersections.insert(intersections.begin(),
                         preceeding_step.intersections.begin(),
                         preceeding_step.intersections.end());

    return *this;
}

// Elongate by another step in back
inline RouteStep &RouteStep::ElongateBy(const RouteStep &following_step)
{
    BOOST_ASSERT(geometry_end == following_step.geometry_begin + 1);
    BOOST_ASSERT(mode == following_step.mode);
    duration += following_step.duration;
    distance += following_step.distance;

    geometry_end = following_step.geometry_end;
    intersections.insert(intersections.end(),
                         following_step.intersections.begin(),
                         following_step.intersections.end());

    return *this;
}

// Elongate without prior knowledge of in front, or in back.
inline RouteStep &RouteStep::MergeWith(const RouteStep &by_step)
{
    // if our own geometry ends, where the next begins, we elongate by
    if (geometry_end == by_step.geometry_begin + 1)
        return AddInFront(by_step);
    else
        return ElongateBy(by_step);
}

// copy all strings from origin into the step, apart from rotary names
inline RouteStep &RouteStep::AdaptStepSignage(const RouteStep &origin)
{
    name_id = origin.name_id;
    name = origin.name;
    pronunciation = origin.pronunciation;
    destinations = origin.destinations;
    ref = origin.ref;

    return *this;
}

inline LaneID RouteStep::NumLanesToTheRight() const
{
    return intersections.front().lanes.first_lane_from_the_right;
}

inline LaneID RouteStep::NumLanesToTheLeft() const
{
    LaneID const total = intersections.front().lane_description.size();
    return total - (intersections.front().lanes.lanes_in_turn +
                    intersections.front().lanes.first_lane_from_the_right);
}

inline auto RouteStep::LanesToTheLeft() const
{
    const auto &description = intersections.front().lane_description;
    LaneID num_lanes_left = NumLanesToTheLeft();
    return boost::make_iterator_range(description.begin(), description.begin() + num_lanes_left);
}

inline auto RouteStep::LanesToTheRight() const
{
    const auto &description = intersections.front().lane_description;
    LaneID num_lanes_right = NumLanesToTheRight();
    return boost::make_iterator_range(description.end() - num_lanes_right, description.end());
}

} // namespace guidance
} // namespace engine
} // namespace osrm

#endif
