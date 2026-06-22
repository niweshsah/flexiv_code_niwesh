#include "Common/Logger.hpp"
#include "Common/SignalHandler.hpp"
#include "Parser/ParserFactory.hpp"
#include "Robot/RobotExecutor.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace flexiv_executor;

int main(int argc, char** argv) {
    common::Logger::Init();
    common::SignalHandler::Init();

    if (argc < 3) {
        LOG_ERROR("Usage: ./PrimitiveExecutor <global_config.yaml> <primitive_recipe.yaml>");
        return -1;
    }

    try {
        LOG_INFO("Loading YAML configurations...");
        YAML::Node globalConfig = YAML::LoadFile(argv[1]);
        YAML::Node recipeConfig = YAML::LoadFile(argv[2]);

        std::string robotSN = globalConfig["robot_sn"].as<std::string>();
        std::string primitiveName = recipeConfig["primitive"].as<std::string>();

        common::MotionDefaults motionParams;
        if (recipeConfig["motion_params"]) {
            auto mp = recipeConfig["motion_params"];
            if (mp["vel"])  motionParams.vel = mp["vel"].as<double>();
            if (mp["acc"])  motionParams.acc = mp["acc"].as<double>();
            if (mp["hold"]) motionParams.hold = mp["hold"].as<bool>();
        }

        auto primitiveParser = parser::ParserFactory::Create(primitiveName);
        common::PrimitiveRequest request = primitiveParser->ParseYAML(recipeConfig, motionParams);

        robot::RobotExecutor executor(robotSN);
        executor.ConnectAndInitialize();

        if (primitiveParser->RequiresFTSensorZeroing()) {
            executor.ZeroFTSensor();
        }

        executor.Execute(request);
        executor.WaitUntilFinished(motionParams.hold);

    } catch (const YAML::Exception& e) {
        LOG_ERROR("YAML Parsing Error: {}", e.what());
        return -1;
    } catch (const std::exception& e) {
        LOG_ERROR("Runtime Exception: {}", e.what());
        return -1;
    }

    return 0;
}