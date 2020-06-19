// Copyright 2020 Intermodalics BVBA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OROCOS__RTT_ROS2__GETTER_SETTER_DATASOURCE_HPP_
#define OROCOS__RTT_ROS2__GETTER_SETTER_DATASOURCE_HPP_

#include <map>
#include <utility>

#include "boost/function.hpp"
#include "rtt/internal/DataSources.hpp"
#include "rtt/internal/NA.hpp"

namespace rtt_ros2
{

/**
 * A data source which accesses a field of a struct-like object by invoking
 * the given getter and setter member functions.
 *
 * @todo Move to core RTT?
 */
template<typename ObjectT, typename FieldT>
class GetterSetterDataSource : public RTT::internal::AssignableDataSource<FieldT>
{
public:
  using typename RTT::internal::AssignableDataSource<FieldT>::value_t;
  using typename RTT::internal::AssignableDataSource<FieldT>::result_t;
  using typename RTT::internal::AssignableDataSource<FieldT>::const_reference_t;
  using typename RTT::internal::AssignableDataSource<FieldT>::param_t;
  using typename RTT::internal::AssignableDataSource<FieldT>::reference_t;

  /**
   * Use this type to store a pointer to a GetterSetterDataSource.
   */
  typedef boost::intrusive_ptr<GetterSetterDataSource> shared_ptr;
  typedef typename boost::intrusive_ptr<const GetterSetterDataSource> const_ptr;

  typedef boost::function<value_t(const ObjectT & obj)> getter_func_t;
  typedef boost::function<void (ObjectT & obj, param_t value)> setter_func_t;

  /**
   * Construct a GetterSetterDataSource from an AssignableDataSource of type ObjectT and
   * functors for getting and setting the value of a specific field of type FieldT, with
   * a custom default value.
   */
  GetterSetterDataSource(
    typename RTT::internal::AssignableDataSource<ObjectT>::shared_ptr object,
    getter_func_t getter,
    setter_func_t setter,
    param_t default_value = RTT::internal::NA<FieldT>::na())
  : object_(std::move(object)),
    getter_(std::move(getter)),
    setter_(std::move(setter)),
    value_(std::move(default_value))
  {}

  ~GetterSetterDataSource()
  {}

  /**
   * Return the data as type \a FieldT.
   */
  virtual result_t get() const
  {
    value_ = getter_(object_->get());
    return value_;
  }

  /**
   * Return the result of the last \a evaluate() function.
   * You must call evaluate() prior to calling this function in order to get
   * the most recent value of this attribute.
   */
  virtual result_t value() const
  {
    return value_;
  }

  /**
   * Get a const reference to the value of this DataSource.
   * You must call evaluate() prior to calling this function in order to get
   * the most recent value of this attribute.
   * @note Getting a reference to an internal data structure is not thread-safe.
   */
  virtual const_reference_t rvalue() const
  {
    return value_;
  }

  /**
   * Set this DataSource with a value.
   */
  virtual void set(param_t t)
  {
    value_ = std::move(t);
    updated();
  }

  /**
   * Get a reference to the value of this DataSource.
   * Getting a reference to an internal data structure is not thread-safe.
   */
  virtual reference_t set()
  {
    return value_;
  }

  virtual bool evaluate() const
  {
    if (!object_->evaluate()) {return false;}
    value_ = getter_(object_->rvalue());
    return true;
  }

  /**
   * In case the internal::DataSource returns a 'reference' type,
   * call this method to notify it that the data was updated
   * in the course of an invocation of set().
   */
  virtual void updated()
  {
    setter_(object_->set(), value_);
    object_->updated();
  }

  virtual GetterSetterDataSource * clone() const
  {
    return new GetterSetterDataSource(object_, getter_, setter_);
  }

  virtual GetterSetterDataSource * copy(
    std::map<
      const RTT::base::DataSourceBase *,
      RTT::base::DataSourceBase *> & replace) const
  {
    // if somehow a copy exists, return the copy, otherwise return this (see Attribute copy)
    if (replace[this] != 0) {
      assert(
        dynamic_cast<GetterSetterDataSource *>(replace[this]) ==
        static_cast<GetterSetterDataSource *>(replace[this]));
      return static_cast<GetterSetterDataSource *>(replace[this]);
    }
    typename RTT::internal::AssignableDataSource<ObjectT>::shared_ptr object_copy =
      RTT::internal::AssignableDataSource<ObjectT>::narrow(object_->copy(replace));
    replace[this] = new GetterSetterDataSource(object_copy, getter_, setter_);
    // returns copy
    return static_cast<GetterSetterDataSource *>(replace[this]);
  }

  /**
   * This method narrows a base::DataSourceBase to a typeded AssignableDataSource,
   * possibly returning a new object.
   */
  static GetterSetterDataSource * narrow(RTT::base::DataSourceBase * db)
  {
    return dynamic_cast<GetterSetterDataSource>(db);
  }

protected:
  typename RTT::internal::AssignableDataSource<ObjectT>::shared_ptr object_;
  getter_func_t getter_;
  setter_func_t setter_;

  mutable value_t value_;
};

}  // namespace rtt_ros2

#endif  // OROCOS__RTT_ROS2__GETTER_SETTER_DATASOURCE_HPP_
