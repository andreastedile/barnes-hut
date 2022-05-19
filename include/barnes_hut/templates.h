#ifndef BARNES_HUT_TEMPLATES_H
#define BARNES_HUT_TEMPLATES_H

// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

#endif  // BARNES_HUT_TEMPLATES_H
