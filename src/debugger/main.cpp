#include <core/logger.hpp>
#include <core/wall_clock.hpp>
#include <debug/repl.hpp>

int main(int argc, char **argv)
{
  hyped::core::WallClock time;
  hyped::core::Logger logger("Debugger", hyped::core::LogLevel::kDebug, time);
  hyped::debug::Repl repl(logger);
  repl.run();
}