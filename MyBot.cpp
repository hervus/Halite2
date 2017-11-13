#include "hlt/hlt.hpp"
#include "hlt/navigation.hpp"

int main() {
    const hlt::Metadata metadata = hlt::initialize("hervus3");
    const hlt::PlayerId player_id = metadata.player_id;

    const hlt::Map& initial_map = metadata.initial_map;

    // We now have 1 full minute to analyse the initial map.
    std::ostringstream initial_map_intelligence;
    initial_map_intelligence
            << "width: " << initial_map.map_width
            << "; height: " << initial_map.map_height
            << "; players: " << initial_map.ship_map.size()
            << "; my ships: " << initial_map.ship_map.at(player_id).size()
            << "; planets: " << initial_map.planets.size();
    hlt::Log::log(initial_map_intelligence.str());

    std::vector<hlt::Move> moves;
    std::vector<std::pair<hlt::Move,hlt::Move>> thrust_moves;
    for (;;) {
        moves.clear();
        thrust_moves.clear();
        const hlt::Map map = hlt::in::get_map();

        for (const hlt::Ship& ship : map.ships.at(player_id)) {
            if (ship.docking_status != hlt::ShipDockingStatus::Undocked) {
                continue;
            }

            for (const hlt::Planet& planet : map.planets) {
                if (planet.owned) {
                    if (planet.owner_id != player_id)
                        continue;
                    if (planet.docked_ships.size() == planet.docking_spots)
                        continue;
                }

                if (ship.can_dock(planet)) {
                    moves.push_back(hlt::Move::dock(ship.entity_id, planet.entity_id));
                    break;
                }

                const std::pair<hlt::Move,hlt::Move> move =
                        hlt::navigation::navigate_ship_to_dock(map, ship, planet, hlt::constants::MAX_SPEED);
                if (move.first.type != hlt::MoveType::Noop) {
                    thrust_moves.push_back(move);
                }
                break;
            }
        }

        // Solve the thrust moves conflicts
        for (std::pair<hlt::Move,hlt::Move> move : thrust_moves) {
            moves.push_back(move.first);  // TODO
        }

        // Send the moves (without conflicts)
        if (!hlt::out::send_moves(moves)) {
            hlt::Log::log("send_moves failed; exiting");
            break;
        }
    }
}
