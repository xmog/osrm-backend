#ifndef OSRM_ENGINE_GUIDANCE_COLLAPSE_HPP

#include "engine/guidance/route_step.hpp"

namespace osrm
{
namespace engine
{
namespace guidance
{

/* Multiple possible reasons can result in unnecessary/confusing instructions
 * A prime example would be a segregated intersection. Turning around at this
 * intersection would result in two instructions to turn left.
 * Collapsing such turns into a single turn instruction, we give a clearer
 * set of instructionst that is not cluttered by unnecessary turns/name changes. */
OSRM_ATTR_WARN_UNUSED
std::vector<RouteStep> CollapseTurnInstructions(std::vector<RouteStep> steps);

/* A combined turn is a set of two instructions that actually form a single turn, as far as we
 * perceive it. A u-turn consisting of two left turns is one such example. But there are also lots
 * of other items that influence how we combine turns. This function is an entry point, defining the
 * possibility to select one of multiple strategies when combining a turn with another one. */
template <typename CombinedTurnStrategy, typename SignageStrategy>
RouteStep CombineRouteSteps(const RouteStep &step_at_turn_location,
                            const RouteStep &step_after_turn_location,
                            const CombinedTurnStrategy combined_turn_stragey,
                            const SignageStrategy signage_strategy);

/* Strategies to keep/throw away */

// Keep the first/second turn type
void KeepInTurnType(RouteStep &output_step,
                    const RouteStep &step_at_turn_location,
                    const RouteStep &step_after_turn_location);
// Keep the type at the turn location, but compute a combined turn angle
void KeepInTurnTypeWithCombinedAngle(RouteStep &output_step,
                                     const RouteStep &step_at_turn_location,
                                     const RouteStep &step_after_turn_location);
void KeepOutTurnType(RouteStep &output_step,
                     const RouteStep &step_at_turn_location,
                     const RouteStep &step_after_turn_location);

// Signage Strategies
void KeepInSignage(RouteStep &output_step,
                   const RouteStep &step_at_turn_location,
                   const RouteStep &step_after_turn_location);
void KeepOutSignage(RouteStep &output_step,
                    const RouteStep &step_at_turn_location,
                    const RouteStep &step_after_turn_location);

// Signage Strategies
void KeepInLanes(RouteStep &output_step,
                 const RouteStep &step_at_turn_location,
                 const RouteStep &step_after_turn_location);
void KeepOutLanes(RouteStep &output_step,
                  const RouteStep &step_at_turn_location,
                  const RouteStep &step_after_turn_location);

/* IMPLEMENTATIONS */
template <typename CombinedTurnStrategy, typename SignageStrategy, typename LaneStrategy>
void CombineRouteSteps(RouteStep &step_at_turn_location,
                       RouteStep &step_after_turn_location,
                       CombinedTurnStrategy combined_turn_stragey,
                       SignageStrategy signage_strategy,
                       LaneStrategy lane_strategy)
{
    // assign the combined turn type
    combined_turn_stragey(step_at_turn_location, step_at_turn_location, step_after_turn_location);

    // assign the combind signage
    signage_strategy(step_at_turn_location, step_at_turn_location, step_after_turn_location);

    // assign the desired turn lanes
    lane_strategy(step_at_turn_location, step_at_turn_location, step_after_turn_location);

    // further stuff should happen here as well
    step_at_turn_location.ElongateBy(step_after_turn_location);
    step_after_turn_location.Invalidate();
}

} /* namespace guidance */
} /* namespace osrm */
} /* namespace osrm */

#endif /* OSRM_ENGINE_GUIDANCE_COLLAPSE_HPP_ */
