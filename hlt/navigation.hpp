#pragma once

#include "collision.hpp"
#include "map.hpp"
#include "move.hpp"
#include "util.hpp"

namespace hlt {
    namespace navigation {
        static void check_and_add_entity_between(
                std::vector<const Entity *>& entities_found,
                const Location& start,
                const Location& target,
                const Entity& entity_to_check)
        {
            const Location &location = entity_to_check.location;
            if (location == start || location == target) {
                return;
            }
            if (collision::segment_circle_intersect(start, target, entity_to_check, constants::FORECAST_FUDGE_FACTOR)) {
                entities_found.push_back(&entity_to_check);
            }
        }

        static std::vector<const Entity *> objects_between(const Map& map, const Location& start, const Location& target) {
            std::vector<const Entity *> entities_found;

            for (const Planet& planet : map.planets) {
                check_and_add_entity_between(entities_found, start, target, planet);
            }

            for (const auto& player_ship : map.ships) {
                for (const Ship& ship : player_ship.second) {
                    check_and_add_entity_between(entities_found, start, target, ship);
                }
            }

            return entities_found;
        }

        static Move navigate_ship_towards_target(
                const Map& map,
                const Ship& ship,
                const Location& target,
                const int max_thrust,
                const bool avoid_obstacles,
                const int max_corrections,
                const double angular_step_rad)
        {
            if (max_corrections <= 0) {
                return Move::noop();
            }

            const double distance = ship.location.get_distance_to(target);
            const double angle_rad = ship.location.orient_towards_in_rad(target);

            if (avoid_obstacles && !objects_between(map, ship.location, target).empty()) {
                const double new_target_dx = cos(angle_rad + angular_step_rad) * distance;
                const double new_target_dy = sin(angle_rad + angular_step_rad) * distance;
                const Location new_target = { ship.location.pos_x + new_target_dx, ship.location.pos_y + new_target_dy };

                return navigate_ship_towards_target(
                        map, ship, new_target, max_thrust, true, (max_corrections - 1), angular_step_rad);
            }

            int thrust;
            if (distance < max_thrust) {
                // Do not round up, since overshooting might cause collision.
                thrust = (int) distance;
            } else {
                thrust = max_thrust;
            }

            const int angle_deg = util::angle_rad_to_deg_clipped(angle_rad);

            return Move::thrust(ship.entity_id, thrust, angle_deg);
        }

        static std::pair<Move,Move> navigate_ship_to_dock(
                const Map& map,
                const Ship& ship,
                const Entity& dock_target,
                const int max_thrust)
        {
            const int max_corrections = constants::MAX_NAVIGATION_CORRECTIONS;
            const bool avoid_obstacles = true;
            const double angular_step_rad = M_PI / 180.0;
            const Location& target = ship.location.get_closest_point(dock_target.location, dock_target.radius);

            Move move_first = navigate_ship_towards_target(
                    map, ship, target, max_thrust, avoid_obstacles, max_corrections, angular_step_rad);
            Move move_second = navigate_ship_towards_target(
                    map, ship, target, max_thrust, avoid_obstacles, max_corrections, -angular_step_rad);
            // If only one Move is valid, it must be the first one
            if (move_first.type == hlt::MoveType::Noop)
                return { move_second, move_first };
            if (move_second.type == hlt::MoveType::Noop)
                return { move_first, move_second };
            // If both thrust Moves of the ship_id are identical, only the first one is left valid
            if ((move_first.move_thrust == move_second.move_thrust) && (move_first.move_angle_deg == move_second.move_angle_deg))
                return { move_first, Move::noop() };
            // Provide as move_first the nearest one with the target direction
            const double target_direction_rad = ship.location.orient_towards_in_rad(target);
            const int target_direction_deg = util::angle_rad_to_deg_clipped(target_direction_rad);
            int diff_1 = (target_direction_deg < move_first.move_angle_deg) ? move_first.move_angle_deg-target_direction_deg : target_direction_deg-move_first.move_angle_deg;
            int diff_2 = (target_direction_deg < move_second.move_angle_deg) ? move_second.move_angle_deg-target_direction_deg : target_direction_deg-move_second.move_angle_deg;
            if (diff_2 < diff_1)
                return { move_second, move_first };
            // Default case
            return { move_first, move_second };
        }
    }
}
