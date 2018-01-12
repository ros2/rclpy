// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <pybind11/pybind11.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;


namespace rclpy
{
class WaitSet
{
public:
  void add_subscription(py::object subscription);
  void add_guard_condition(py::object guard_condition);
  void add_timer(py::object timer);
  void add_client(py::object client);
  void add_service(py::object service);
  void wait(int timeout);
  bool is_subscription_ready(py::object subscription);
  bool is_guard_condition_ready(py::object guard_condition);
  bool is_timer_ready(py::object timer);
  bool is_client_ready(py::object client);
  bool is_service_ready(py::object service);

private:
  std::vector<rcl_subscription_t *> pysubs;
  std::vector<rcl_guard_condition_t *> pygcs;
  std::vector<rcl_timer_t *> pytmrs;
  std::vector<rcl_client_t *> pyclis;
  std::vector<rcl_service_t *> pysrvs;
};

template<typename RCL_TYPE>
RCL_TYPE * get_handle(py::object entity, const std::string & type)
{
  py::capsule capsule = py::getattr(entity, (type + std::string("_handle")).c_str());

  void * handle = PyCapsule_GetPointer(
    capsule.ptr(), (std::string("rcl_") + type + std::string("_t")).c_str());
  if (handle == NULL) {
    throw pybind11::error_already_set();
  }
  return static_cast<RCL_TYPE *>(handle);
}

void WaitSet::add_subscription(py::object subscription)
{
  pysubs.push_back(get_handle<rcl_subscription_t>(subscription, "subscription"));
}

void WaitSet::add_guard_condition(py::object guard_condition)
{
  pygcs.push_back(get_handle<rcl_guard_condition_t>(guard_condition, "guard_condition"));
}

void WaitSet::add_timer(py::object timer)
{
  pytmrs.push_back(get_handle<rcl_timer_t>(timer, "timer"));
}

void WaitSet::add_client(py::object client)
{
  pyclis.push_back(get_handle<rcl_client_t>(client, "client"));
}

void WaitSet::add_service(py::object service)
{
  pysrvs.push_back(get_handle<rcl_service_t>(service, "service"));
}

void WaitSet::wait(int timeout)
{
  rcl_ret_t ret;

  // Initialize the wait set
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(
    &wait_set, pysubs.size(), pygcs.size(), pytmrs.size(), pyclis.size(), pysrvs.size(),
    rcl_get_default_allocator());

  // Populate the wait set
  for (std::size_t i = 0; i < pysubs.size(); ++i) {
    wait_set.subscriptions[i] = pysubs[i];
  }
  for (std::size_t i = 0; i < pygcs.size(); ++i) {
    wait_set.guard_conditions[i] = pygcs[i];
  }
  for (std::size_t i = 0; i < pytmrs.size(); ++i) {
    wait_set.timers[i] = pytmrs[i];
  }
  for (std::size_t i = 0; i < pyclis.size(); ++i) {
    wait_set.clients[i] = pyclis[i];
  }
  for (std::size_t i = 0; i < pysrvs.size(); ++i) {
    wait_set.services[i] = pysrvs[i];
  }

  { // Release the GIL and wait
    py::gil_scoped_release release;
    ret = rcl_wait(&wait_set, timeout);
  }
  if (ret != RCL_RET_OK) {
    // becomes RuntimeError in python
    auto e = std::runtime_error(
      std::string("Failed to wait: ") + std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throw e;
  }

  // Get the result from the wait set
  std::vector<rcl_subscription_t *> ready_subscriptions;
  std::vector<rcl_guard_condition_t *> ready_guard_conditions;
  std::vector<rcl_timer_t *> ready_timers;
  std::vector<rcl_client_t *> ready_clients;
  std::vector<rcl_service_t *> ready_services;
  if (ret != RCL_RET_TIMEOUT) {
    for (std::size_t e = 0; e < pysubs.size(); ++e) {
      for (std::size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
        if (wait_set.subscriptions[i] == pysubs[e]) {
          ready_subscriptions.push_back(pysubs[e]);
          break;
        }
      }
    }
    for (std::size_t e = 0; e < pygcs.size(); ++e) {
      for (std::size_t i = 0; i < wait_set.size_of_guard_conditions; ++i) {
        if (wait_set.guard_conditions[i] == pygcs[e]) {
          ready_guard_conditions.push_back(pygcs[e]);
          break;
        }
      }
    }
    for (std::size_t e = 0; e < pytmrs.size(); ++e) {
      for (std::size_t i = 0; i < wait_set.size_of_timers; ++i) {
        if (wait_set.timers[i] == pytmrs[e]) {
          ready_timers.push_back(pytmrs[e]);
          break;
        }
      }
    }
    for (std::size_t e = 0; e < pyclis.size(); ++e) {
      for (std::size_t i = 0; i < wait_set.size_of_clients; ++i) {
        if (wait_set.clients[i] == pyclis[e]) {
          ready_clients.push_back(pyclis[e]);
          break;
        }
      }
    }
    for (std::size_t e = 0; e < pysrvs.size(); ++e) {
      for (std::size_t i = 0; i < wait_set.size_of_services; ++i) {
        if (wait_set.services[i] == pysrvs[e]) {
          ready_services.push_back(pysrvs[e]);
          break;
        }
      }
    }
  }
  pysubs = std::move(ready_subscriptions);
  pygcs = std::move(ready_guard_conditions);
  pytmrs = std::move(ready_timers);
  pyclis = std::move(ready_clients);
  pysrvs = std::move(ready_services);
}

bool WaitSet::is_subscription_ready(py::object subscription)
{
  return std::find(pysubs.begin(), pysubs.end(),
           get_handle<rcl_subscription_t>(subscription, "subscription")) != pysubs.end();
}

bool WaitSet::is_guard_condition_ready(py::object guard_condition)
{
  return std::find(pygcs.begin(), pygcs.end(),
           get_handle<rcl_guard_condition_t>(guard_condition, "guard_condition")) != pygcs.end();
}

bool WaitSet::is_timer_ready(py::object timer)
{
  return std::find(pytmrs.begin(), pytmrs.end(),
           get_handle<rcl_timer_t>(timer, "timer")) != pytmrs.end();
}

bool WaitSet::is_client_ready(py::object client)
{
  return std::find(pyclis.begin(), pyclis.end(),
           get_handle<rcl_client_t>(client, "client")) != pyclis.end();
}

bool WaitSet::is_service_ready(py::object service)
{
  return std::find(pysrvs.begin(), pysrvs.end(),
           get_handle<rcl_service_t>(service, "service")) != pysrvs.end();
}
}  // namespace rclpy


PYBIND11_MODULE(_rclpy_wait_set, m) {
  m.doc() =
    R"pbdoc(
      _rclpy_wait_set
      -----------------------
      Implements code to interface python and rcl.
  )pbdoc";

  py::class_<rclpy::WaitSet>(m, "WaitSet")
  .def(py::init<>())
  .def("add_subscription", &rclpy::WaitSet::add_subscription)
  .def("add_guard_condition", &rclpy::WaitSet::add_guard_condition)
  .def("add_timer", &rclpy::WaitSet::add_timer)
  .def("add_client", &rclpy::WaitSet::add_client)
  .def("add_service", &rclpy::WaitSet::add_service)
  .def("wait", &rclpy::WaitSet::wait, "this is a doc string", py::arg("timeout") = -1)
  .def("is_subscription_ready", &rclpy::WaitSet::is_subscription_ready)
  .def("is_guard_condition_ready", &rclpy::WaitSet::is_guard_condition_ready)
  .def("is_timer_ready", &rclpy::WaitSet::is_timer_ready)
  .def("is_client_ready", &rclpy::WaitSet::is_client_ready)
  .def("is_service_ready", &rclpy::WaitSet::is_service_ready);

  // TODO(sloretz) target_compile_definisions VERSION_INFO
#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}
