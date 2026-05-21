#include "engine/routing_algorithms/routing_base_ch.hpp"
#include "mocks/mock_datafacade.hpp"
#include "util/exception.hpp"
#include "util/typedefs.hpp"

#include <boost/test/unit_test.hpp>

#include <utility>
#include <vector>

// Regression suite for the CH path-unpacking hardening.
//
// Background: in release builds, BOOST_ASSERT_MSG is compiled out.  Before the
// hardening, a packed path whose edges could not be resolved (e.g. from a
// corrupt CH graph) would fall through to facade.GetEdgeData(SPECIAL_EDGEID)
// and segfault the process.  The fix throws util::exception so the Node binding
// can surface a JS-side error instead.
//
// See INC-296 (2026-05-16, prod-ca OSRM segfault outage).
BOOST_AUTO_TEST_SUITE(routing_base_ch_unpack)

BOOST_AUTO_TEST_CASE(unpackPath_throws_on_invalid_edge_instead_of_segfaulting)
{
    using namespace osrm::engine::routing_algorithms::ch;

    // MockAlgorithmDataFacade<CH>::FindSmallestEdge always returns SPECIAL_EDGEID,
    // which is exactly the condition that previously segfaulted via GetEdgeData.
    const osrm::test::MockDataFacade<osrm::engine::routing_algorithms::ch::Algorithm> facade{};

    // Two adjacent NodeIDs are the minimum to exercise the unpack loop.
    const std::vector<NodeID> packed_path{0u, 1u};

    BOOST_CHECK_THROW(unpackPath(facade,
                                 packed_path.begin(),
                                 packed_path.end(),
                                 [](std::pair<NodeID, NodeID>, const EdgeID &) {}),
                      osrm::util::exception);
}

BOOST_AUTO_TEST_CASE(unpackPath_empty_path_does_not_throw)
{
    using namespace osrm::engine::routing_algorithms::ch;

    const osrm::test::MockDataFacade<osrm::engine::routing_algorithms::ch::Algorithm> facade{};

    const std::vector<NodeID> packed_path{};

    // Sanity: an empty path is a legitimate input and must short-circuit.
    BOOST_CHECK_NO_THROW(unpackPath(facade,
                                    packed_path.begin(),
                                    packed_path.end(),
                                    [](std::pair<NodeID, NodeID>, const EdgeID &) {}));
}

BOOST_AUTO_TEST_SUITE_END()
