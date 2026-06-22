#pragma once
#include <string>
#include <map>
#include <flexiv/rdk/utility.hpp>

namespace flexiv_executor::common {

/**
 * @brief A generic structure returned by parsers, utilizing Flexiv's native variant types.
 */
struct PrimitiveRequest {
    std::string primitiveName;
    std::map<std::string, flexiv::rdk::FlexivDataTypes> parameters;
};

} // namespace flexiv_executor::common