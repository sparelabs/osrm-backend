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

namespace
{
// The default MockDataFacade<CH> returns 0 from both GetNumberOfNodes and
// GetNumberOfEdges, which means the in-range node check at the top of
// unpackPath would fire for any non-SPECIAL node ID before we ever reach
// FindSmallestEdge.  To exercise each defensive branch independently we need
// to control those two values per-test.
class ConfigurableMockFacade
    : public osrm::test::MockBaseDataFacade,
      public osrm::test::MockAlgorithmDataFacade<osrm::engine::datafacade::CH>
{
  public:
    ConfigurableMockFacade(unsigned num_nodes, unsigned num_edges)
        : num_nodes_{num_nodes}, num_edges_{num_edges}
    {
    }

    unsigned GetNumberOfNodes() const override { return num_nodes_; }
    unsigned GetNumberOfEdges() const override { return num_edges_; }

  private:
    unsigned num_nodes_;
    unsigned num_edges_;
};
} // namespace

BOOST_AUTO_TEST_CASE(unpackPath_throws_when_node_id_out_of_range)
{
    using namespace osrm::engine::routing_algorithms::ch;

    // Two nodes, but the packed path references node ID 5 which is past the end.
    // This exercises the bound check at the top of the unpack loop.
    const ConfigurableMockFacade facade{/*num_nodes=*/2, /*num_edges=*/10};
    const std::vector<NodeID> packed_path{0u, 5u};

    BOOST_CHECK_THROW(unpackPath(facade,
                                 packed_path.begin(),
                                 packed_path.end(),
                                 [](std::pair<NodeID, NodeID>, const EdgeID &) {}),
                      osrm::util::exception);
}

BOOST_AUTO_TEST_CASE(unpackPath_throws_when_FindSmallestEdge_returns_SPECIAL_EDGEID)
{
    using namespace osrm::engine::routing_algorithms::ch;

    // Nodes are in range, so the node bound check passes.  MockAlgorithmDataFacade<CH>::
    // FindSmallestEdge always returns SPECIAL_EDGEID; this is exactly the
    // condition that previously segfaulted via GetEdgeData(SPECIAL_EDGEID).
    const ConfigurableMockFacade facade{/*num_nodes=*/10, /*num_edges=*/10};
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

    const ConfigurableMockFacade facade{/*num_nodes=*/0, /*num_edges=*/0};
    const std::vector<NodeID> packed_path{};

    // Sanity: an empty path is a legitimate input and must short-circuit.
    BOOST_CHECK_NO_THROW(unpackPath(facade,
                                    packed_path.begin(),
                                    packed_path.end(),
                                    [](std::pair<NodeID, NodeID>, const EdgeID &) {}));
}

BOOST_AUTO_TEST_SUITE_END()
