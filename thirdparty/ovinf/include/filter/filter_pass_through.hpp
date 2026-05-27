#ifndef FILTER_PASS_THROUGH_HPP
#define FILTER_PASS_THROUGH_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <algorithm>

#include "filter/filter_base.hpp"

namespace ovinf {

template <typename T = float>
class PassThroughFilter : public FilterBase<T> {
 public:
  PassThroughFilter() = delete;
  PassThroughFilter(YAML::Node const &config) : FilterBase<T>(config) {
    lower_bound_ = this->ReadYamlParam(config["lower_bound"]);
    upper_bound_ = this->ReadYamlParam(config["upper_bound"]);

    if constexpr (is_eigen_vector_v<T>) {
      this->dimension_ = lower_bound_.rows();
      this->last_input_ = T(this->dimension_).setZero();

      if (lower_bound_.size() != this->dimension_ ||
          upper_bound_.size() != this->dimension_) {
        throw std::runtime_error(
            "PassThroughFilter: lower_bound or upper_bound size doesn't match "
            "the vector dimension. "
            "Expected dimension: " +
            std::to_string(this->dimension_));
      }
    }
  }

  virtual T Filter(T const &input) final {
    if constexpr (is_eigen_vector_v<T>) {
      return this->NanHandle(input)
          .cwiseMin(upper_bound_)
          .cwiseMax(lower_bound_);
    } else {
      return std::max(lower_bound_,
                      std::min(upper_bound_, this->NanHandle(input)));
    }
  }

  virtual void Reset() final {
    if constexpr (is_eigen_vector_v<T>) {
      this->last_input_.setZero();
    } else {
      this->last_input_ = 0;
    }
  };

 private:
  T lower_bound_;
  T upper_bound_;
};

}  // namespace ovinf

#endif  // !FILTER_PASS_THROUGH_HPP
