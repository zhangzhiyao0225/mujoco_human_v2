#pragma once

#include <string>
#include <unordered_map>

#include "types.hpp"
#include "utils/ctstring.hpp"
#include "utils/logger.h"

namespace bitbot {
template <CTString... CTSArray>
class ExtraDataImpl {
 public:
  ExtraDataImpl() : logger_(Logger().ConsoleLogger()) {}
  ~ExtraDataImpl() = default;

  const std::array<Number, sizeof...(CTSArray)>& Data() { return data_; }

  Number& operator[](size_t index) {
    static Number null_number;
    if (index < headers_.Size()) [[likely]]
      return data_[index];
    else
      return null_number;
  }

  template <size_t I>
  void Set(const Number& number) {
    static_assert(I < headers_.Size());
    data_[I] = number;
  }

  template <size_t I>
  void Set(Number&& number) {
    static_assert(I < headers_.Size());
    data_[I] = number;
  }

  template <CTString CTS>
  void Set(const Number& number) {
    static_assert(((headers_.template Index<CTS>()) < headers_.Size()));
    data_[headers_.template Index<CTS>()] = number;
  }

  template <CTString CTS>
  void Set(Number&& number) {
    static_assert((headers_.template Index<CTS>() < headers_.Size()));
    data_[headers_.template Index<CTS>()] = number;
  }

  constexpr std::array<std::string_view, sizeof...(CTSArray)> Headers() {
    return headers_.StrArray();
  }

  constexpr static size_t Size() { return sizeof...(CTSArray); }

 private:
  constexpr static CTSTuple<CTSArray...> headers_{};
  Logger::Console logger_;
  std::array<Number, sizeof...(CTSArray)> data_;
};
}  // namespace bitbot
