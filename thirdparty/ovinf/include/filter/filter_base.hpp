#ifndef FILTER_BASE_HPP
#define FILTER_BASE_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <iostream>

#include "utils/traits.h"

namespace ovinf {

/**
 * @brief Filter base class.
 */
template <typename T = Eigen::Matrix<float, Eigen::Dynamic, 1>>
class FilterBase {
 public:
  using Ptr = std::shared_ptr<FilterBase<T>>;

  FilterBase() = delete;
  constexpr FilterBase(YAML::Node const &config) {
    if constexpr (is_eigen_vector_v<T>) {
      // dimension_ is read from the YAML file
      dimension_ = -1;
      first_input_ = true;
    } else {
      this->dimension_ = 1;
      last_input_ = 0;
    }
  }

  /**
   * @brief Handle input, return filterd value
   *
   * @param[in] input input
   * @return Filtered value
   */
  virtual T Filter(T const &input) {
    if constexpr (is_eigen_vector_v<T>) {
      if (first_input_) [[unlikely]] {
        first_input_ = false;
        this->last_input_ = T(this->dimension_).setZero();
      }
    }
    return NanHandle(input);
  }

  /**
   * @brief Reset the filter
   *
   */
  virtual void Reset() {
    if constexpr (is_eigen_vector_v<T>) {
      last_input_.setZero();
    } else {
      last_input_ = 0;
    }
  };

 protected:
  /**
   * @brief Read from YAML vector or scalar
   *
   * @param[in] node Yaml node
   * @return value
   */
  static T ReadYamlParam(YAML::Node const &node) {
    if constexpr (is_eigen_vector_v<T>) {
      return Eigen::Map<T>(node.as<std::vector<typename T::Scalar>>().data(),
                           node.size());
    } else {
      return node.as<T>();
    }
  }

  /**
   * @brief NaN handle, update last_input_ elementwise
   *
   * @return Updated last_input_
   */
  T &NanHandle(T const &input) {
    if constexpr (is_eigen_vector_v<T>) {
      if (input.hasNaN()) {
        // std::cout << "Nan detected" << std::endl;
        for (size_t i = 0; i < input.size(); ++i) {
          if (!std::isnan(input[i])) {
            last_input_[i] = input[i];
          }
        }
      } else {
        last_input_ = input;
      }
    } else {
      if (!std::isnan(input)) {
        last_input_ = input;
      }
    }
    return last_input_;
  }

 protected:
  size_t dimension_ = 0;
  T last_input_;

 private:
  bool first_input_ = true;
};

}  // namespace ovinf

#endif  // !FILTER_BASE_HPP
