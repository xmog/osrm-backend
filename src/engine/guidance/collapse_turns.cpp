#include "engine/guidance/collapse_turns.hpp"
#include "extractor/guidance/turn_instruction.hpp"
#include "util/guidance/name_announcements.hpp"
#include "util/debug.hpp"

#include <cstddef>

#include <boost/assert.hpp>
#include <boost/optional.hpp>
#include <iterator>

using osrm::extractor::guidance::TurnInstruction;
using namespace osrm::extractor::guidance;

namespace osrm
{
namespace engine
{
namespace guidance
{

using RouteSteps = std::vector<RouteStep>;
using RouteStepIterator = typename RouteSteps::iterator;
namespace
{

// check if a step is completely without turn type
bool hasTurnType(const RouteStep &step)
{
    return step.maneuver.instruction.type != TurnType::NoTurn;
}
bool hasWaypointType(const RouteStep &step)
{
    return step.maneuver.waypoint_type != WaypointType::None;
}

//
RouteStepIterator findPreviousTurn(RouteStepIterator current_step)
{
    BOOST_ASSERT(!hasWaypointType(*current_step));
    // find the first element preceeding the current step that has an actual turn type (not
    // necessarily announced)
    do
    {
        // safety to do this loop is asserted in collapseTurnInstructions
        --current_step;
    } while (!hasTurnType(*current_step) && !hasWaypointType(*current_step));
    return current_step;
}

RouteStepIterator findNextTurn(RouteStepIterator current_step)
{
    BOOST_ASSERT(!hasWaypointType(*current_step));
    // find the first element preceeding the current step that has an actual turn type (not
    // necessarily announced)
    do
    {
        // safety to do this loop is asserted in collapseTurnInstructions
        ++current_step;
    } while (!hasTurnType(*current_step) && !hasWaypointType(*current_step));
    return current_step;
}

inline bool hasTurnType(const RouteStep &step, const TurnType::Enum type)
{
    return type == step.maneuver.instruction.type;
}

inline std::size_t numberOfAvailableTurns(const RouteStep &step)
{
    return step.intersections.front().entry.size();
}

inline bool isTrafficLightStep(const RouteStep &step)
{
    return hasTurnType(step, TurnType::Suppressed) && numberOfAvailableTurns(step) == 2;
}

inline void setInstructionType(RouteStep &step, const TurnType::Enum type)
{
    step.maneuver.instruction.type = type;
}

inline bool haveSameMode(const RouteStep &lhs, const RouteStep &rhs)
{
    return lhs.mode == rhs.mode;
}

inline bool haveSameName(const RouteStep &lhs, const RouteStep &rhs)
{
    // make sure empty is not involved
    if (lhs.name_id == EMPTY_NAMEID || rhs.name_id == EMPTY_NAMEID)
        return false;

    // easy check to not go over the strings if not necessary
    else if (lhs.name_id == rhs.name_id)
        return true;

    // ok, bite the sour grape and check the strings already
    else
        return util::guidance::requiresNameAnnounced(
            lhs.name, lhs.ref, lhs.pronunciation, rhs.name, rhs.ref, rhs.pronunciation);
}

inline void handleSliproad(RouteStepIterator sliproad_step)
{
    auto next_step = [&sliproad_step]() {
        auto next_step = findNextTurn(sliproad_step);
        while (isTrafficLightStep(*next_step))
        {
            // in sliproad checks, we should have made sure not to include invalid modes
            BOOST_ASSERT(haveSameMode(*sliproad_step,*next_step));
            sliproad_step->ElongateBy(*next_step);
            next_step->Invalidate();
            next_step = findNextTurn(next_step);
        }
        BOOST_ASSERT(haveSameMode(*sliproad_step,*next_step));
        return next_step;
    }();

    // have we reached the end?
    if (hasWaypointType(*next_step))
    {
        setInstructionType(*sliproad_step, TurnType::Turn);
    }
    else
    {
        auto sliproad_turn_type =
            haveSameName(*sliproad_step, *next_step) ? TurnType::Continue : TurnType::Turn;
        setInstructionType(*sliproad_step, sliproad_turn_type);
        CombineRouteSteps(*sliproad_step, *next_step, KeepInTurnType, KeepOutSignage);
    }
}

} // namespace

OSRM_ATTR_WARN_UNUSED
RouteSteps CollapseTurnInstructions(RouteSteps steps)
{
    // make sure we can safely iterate over all steps (has depart/arrive with TurnType::NoTurn)
    BOOST_ASSERT(!hasTurnType(steps.front()) && !hasTurnType(steps.back()));
    BOOST_ASSERT(hasWaypointType(steps.front()) && hasWaypointType(steps.back()));

    if (steps.size() <= 2)
        return steps;

    std::cout << "Processing " << steps.size() << " steps." << std::endl;
    util::guidance::print(steps);
    // start of with no-op
    for (auto current_step = steps.begin() + 1; current_step + 1 != steps.end(); ++current_step)
    {
        std::cout << "At: " << std::distance(steps.begin(), current_step) << std::endl;
        if (entersRoundabout(current_step->maneuver.instruction))
        {
            // Skip over all instructions within the roundabout
            for (; current_step + 1 != steps.end(); ++current_step)
                if (leavesRoundabout(current_step->maneuver.instruction))
                    break;

            // are we done for good?
            if (current_step + 1 == steps.end())
                break;
            else
                continue;
        }

        // handle all situations involving the sliproad turn type
        if (hasTurnType(*current_step, TurnType::Sliproad))
        {
            handleSliproad(current_step);
        }
    }
    return steps;
}

// keep turn type
void KeepInTurnType(RouteStep &output_step,
                    const RouteStep &step_at_turn_location,
                    const RouteStep & /**/)
{
    output_step.maneuver = step_at_turn_location.maneuver;
}
void KeepOutTurnType(RouteStep &output_step,
                     const RouteStep & /**/,
                     const RouteStep &step_after_turn_location)
{
    output_step.maneuver = step_after_turn_location.maneuver;
}

// keep signage
void KeepInSignage(RouteStep &output_step,
                   const RouteStep &step_at_turn_location,
                   const RouteStep & /**/)
{
    output_step.AdaptStepSignage(step_at_turn_location);
    output_step.rotary_name = step_at_turn_location.rotary_name;
    output_step.rotary_pronunciation = step_at_turn_location.rotary_pronunciation;
}
void KeepOutSignage(RouteStep &output_step,
                    const RouteStep & /**/,
                    const RouteStep &step_after_turn_location)
{
    output_step.AdaptStepSignage(step_after_turn_location);
    output_step.rotary_name = step_after_turn_location.rotary_name;
    output_step.rotary_pronunciation = step_after_turn_location.rotary_pronunciation;
}

} // namespace guidance
} // namespace engine
} // namespace osrm
