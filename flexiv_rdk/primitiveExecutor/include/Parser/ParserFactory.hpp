#pragma once
#include "Parser/PrimitiveParser.hpp"
#include <memory>
#include <string>

namespace flexiv_executor::parser {

/**
 * @brief Factory for instantiating the correct parser. 
 * OCP compliant: Add new primitives here without touching core executor.
 */
class ParserFactory {
public:
    static std::unique_ptr<PrimitiveParser> Create(const std::string& primitiveName);
    static void PrintGeneralHelp();
};

} // namespace flexiv_executor::parser