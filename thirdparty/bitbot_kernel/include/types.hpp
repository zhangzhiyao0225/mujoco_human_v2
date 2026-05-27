#ifndef TYPES_H_INCLUDED
#define TYPES_H_INCLUDED

#include <stdint.h>
#include <variant>
#include <unordered_map>
#include <set>
#include <functional>

namespace bitbot
{
  using Number = std::variant<uint8_t, uint16_t, uint32_t, int64_t, uint64_t, double>;

  using EventId = uint32_t;
  using EventValue = int64_t;
  using StateId = uint32_t;

  using can_do_next = std::function<bool(EventId)>;
  struct bitbot_init_param
  {
    std::unordered_map<EventValue, EventId> *key_2_evts{};
    std::set<StateId /*,std::set<EventId>*/> states_2_evts{};
    can_do_next can_do{};
  };

  enum class KeyboardEvent : EventValue
  {
    Down = 1,
    Up = 2
  };

}

#endif
