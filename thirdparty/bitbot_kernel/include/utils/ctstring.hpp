#include <algorithm>
#include <type_traits>
#include <tuple>
#include <utility>

namespace bitbot
{
  // compile time string
  template <size_t N>
  struct CTString {
    constexpr CTString(const char (&str)[N])
    {
      std::copy_n(str, N, value);
    }
    char value[N]{};
  };

  template <CTString CTS>
  struct CTSWrapper {
    static constexpr CTString str{CTS};
  };

  template <CTString... CTSArray>
  struct CTSTuple
  {
  public:
    constexpr CTSTuple() {}

    constexpr size_t Size() const
    {
      return sizeof...(CTSArray);
    }

    template<CTString CTS>
    constexpr size_t Index() const
    {
      return IndexCTS<CTS>(tuple_);
    }

    constexpr std::array<std::string_view, sizeof...(CTSArray)> StrArray() const
    {
      return string_array_;
    }

  private:
    template <size_t... Indices, CTString... CTS>
    constexpr static auto MakeTupleImpl(std::index_sequence<Indices...>, CTSWrapper<CTS>...) {
      return std::make_tuple(std::pair<CTSWrapper<CTS>, size_t>{{},Indices}...);
    }

    template <CTString... CTS>
    constexpr static auto MakeTuple()
    {
      return MakeTupleImpl(std::make_index_sequence<sizeof...(CTS)>{}, CTSWrapper<CTS>{}...);
    }

    template <typename T, typename Tuple, size_t... I>
    constexpr static size_t IndexCTSImpl(T&&, Tuple&& tuple, std::index_sequence<I...>) {
      return std::min({(std::is_same_v<T, decltype(std::get<I>(tuple).first)> ? I : sizeof...(I))...});
    }

    template <CTString CTS, typename T>
    constexpr static size_t IndexCTS(T&& tuple) {
      return IndexCTSImpl(CTSWrapper<CTS>{}, std::forward<T>(tuple), std::make_index_sequence<std::tuple_size_v<std::remove_reference_t<T>>>{});
    }

    static inline constexpr auto tuple_{MakeTuple<CTSArray...>()};
    static inline constexpr std::array<std::string_view, sizeof...(CTSArray)> string_array_{CTSArray.value...};
  };

}