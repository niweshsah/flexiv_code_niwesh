#include "Parser/ParserFactory.hpp"
#include "Parser/MoveLParser.hpp"
#include "Parser/MovePTPParser.hpp"
#include "Parser/MoveJParser.hpp"
#include "Parser/ForceCompParser.hpp"
#include "Parser/AlignContactParser.hpp"
#include "Parser/ForceHybridParser.hpp" 
#include "Parser/ForceTrajParser.hpp"   
#include "Parser/VisualServoingParser.hpp"
#include "Common/Logger.hpp"
#include <stdexcept>

namespace flexiv_executor::parser {

std::unique_ptr<PrimitiveParser> ParserFactory::Create(const std::string& primitiveName)
{
    if (primitiveName == "MoveL")
        return std::make_unique<MoveLParser>();
    if (primitiveName == "MovePTP")
        return std::make_unique<MovePTPParser>();
    if (primitiveName == "MoveJ")
        return std::make_unique<MoveJParser>();
    if (primitiveName == "ForceComp")
        return std::make_unique<ForceCompParser>();
    if (primitiveName == "AlignContact")
        return std::make_unique<AlignContactParser>();

    if(primitiveName == "ForceHybrid")
        return std::make_unique<ForceHybridParser>();

    if(primitiveName == "ForceTraj")
        return std::make_unique<ForceTrajParser>();

    if(primitiveName == "HPImageBasedVS")
        return std::make_unique<VisualServoingParser>();

    throw std::invalid_argument("Unsupported primitive: " + primitiveName);
}

void ParserFactory::PrintGeneralHelp()
{
    LOG_INFO("=================================================");
    LOG_INFO(" Flexiv Generic Primitive Executor");
    LOG_INFO("=================================================");
    LOG_INFO("Usage: ./PrimitiveExecutor <global_config.yaml> <recipe.yaml>");
    LOG_INFO("Available Primitives: MoveL, MovePTP, MoveJ, ForceComp, AlignContact"); 
}

} // namespace flexiv_executor::parser