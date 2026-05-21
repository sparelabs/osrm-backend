#include "util/exception.hpp"

#include <boost/assert.hpp>

#include <sstream>
#include <string>

namespace
{
// Throw a catchable osrm::util::exception on assertion violation, rather than
// std::terminate-ing the process.  Combined with the Node binding's
// per-request `catch (const std::exception &)` in src/nodejs/node_osrm.cpp
// (Worker::Execute) this turns assertion violations into JS-side errors
// instead of process crashes.
//
// Triggered by INC-296 (prod-ca OSRM segfault outage, 2026-05-16): a corrupt
// CH graph was producing data that downstream code accessed unsafely.  Many
// of those unsafe accesses are guarded by BOOST_ASSERT_MSG calls but those
// are compiled out unless BOOST_ENABLE_ASSERT_HANDLER is set (see
// CMakeLists.txt → ENABLE_ASSERTIONS).  This handler is what makes
// BOOST_ENABLE_ASSERT_HANDLER useful: rather than hard-aborting, we surface
// the violation up the call stack as an exception the caller can handle.
[[noreturn]] void assertion_failed_msg_helper(
    char const *expr, char const *msg, char const *function, char const *file, long line)
{
    std::ostringstream oss;
    oss << "OSRM assertion failed: ";
    if (msg != nullptr && msg[0] != '\0')
    {
        oss << msg << " (" << expr << ")";
    }
    else
    {
        oss << expr;
    }
    oss << " in " << function << " at " << file << ":" << line;
    throw osrm::util::exception(oss.str());
}
} // namespace

// Boost.Assert only declares the following two functions and let's us define them here.
namespace boost
{
void assertion_failed(char const *expr, char const *function, char const *file, long line)
{
    ::assertion_failed_msg_helper(expr, "", function, file, line);
}
void assertion_failed_msg(
    char const *expr, char const *msg, char const *function, char const *file, long line)
{
    ::assertion_failed_msg_helper(expr, msg, function, file, line);
}
} // namespace boost
