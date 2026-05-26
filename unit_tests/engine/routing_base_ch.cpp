#include "contractor/query_edge.hpp"
#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"
#include "engine/search_engine_data.hpp"
#include "mocks/mock_datafacade.hpp"
#include "util/exception.hpp"
#include "util/typedefs.hpp"

#include <boost/test/unit_test.hpp>

#include <unordered_map>
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

// ---------------------------------------------------------------------------
// Regression tests for endpointsFromCandidates — the assertion that previously
// segfaulted production for INC-296.
//
// Root cause (verified empirically against the production Winnipeg .osrm at
// the bug-triggering Osama coordinates):
//   1. The CH search hits a node X reachable from both source and target
//      heaps where forward_heap.GetKey(X) + reverse_heap.GetKey(X) < 0.
//      This happens when the source phantom has a large forward-weight offset
//      (e.g. snapping to a road where bus.lua produces an unusual offset).
//   2. `routingStep` (routing_base_ch.hpp) enters its "self-loop branch" on
//      `new_weight < 0` *alone*, without checking that source and target are
//      actually on the same segment.
//   3. X happens to have a self-loop shortcut edge (X→X via some contracted-
//      out dead-end node).  The algorithm picks it as the answer.
//   4. The resulting packed_leg = [X, X] is unpacked to a closed loop through
//      original-graph nodes — none of which are the source or target phantoms.
//   5. extractRoute → endpointsFromCandidates tries to match the path's
//      first/last node to the candidates, finds neither, and dereferences
//      end() of the candidates vector → segfault in release builds.
//
// This regression suite locks down the failure mode at the endpointsFromCandidates
// boundary: when the path doesn't match the candidates, we get a *catchable*
// util::exception (via BOOST_ENABLE_ASSERT_HANDLER), not undefined behavior.
//
// See INC-296 for full investigation.

BOOST_AUTO_TEST_SUITE(endpoints_from_candidates_self_loop_regression)

namespace
{
osrm::engine::PhantomNode makePhantom(NodeID forward_seg_id, NodeID reverse_seg_id)
{
    osrm::engine::PhantomNode p{};
    p.forward_segment_id = ::SegmentID{forward_seg_id, forward_seg_id != SPECIAL_SEGMENTID};
    p.reverse_segment_id = ::SegmentID{reverse_seg_id, reverse_seg_id != SPECIAL_SEGMENTID};
    return p;
}
} // namespace

// Sanity case: when the path's endpoints DO match candidates, the function
// returns normally.  Anchors the positive baseline so the failure-mode tests
// below are unambiguous.
BOOST_AUTO_TEST_CASE(endpointsFromCandidates_matching_path_succeeds)
{
    using namespace osrm::engine;
    using namespace osrm::engine::routing_algorithms;

    const PhantomNodeCandidates source_phantoms{makePhantom(/*fwd=*/100, /*rev=*/101)};
    const PhantomNodeCandidates target_phantoms{makePhantom(/*fwd=*/200, /*rev=*/201)};
    const PhantomEndpointCandidates candidates{source_phantoms, target_phantoms};

    const std::vector<NodeID> path{100u, 150u, 200u}; // source.fwd → … → target.fwd

    BOOST_CHECK_NO_THROW(endpointsFromCandidates(candidates, path));
}

// INC-296 exact reproduction.  The packed_leg returned by the CH search for
// the production-triggering route was [4042, 5176, M, 1917, 4042]: a closed
// loop at non-phantom node 4042, produced by unpacking the self-loop shortcut
// the algorithm spuriously selected as the answer.  The candidates that were
// actually requested have source-forward-segment 6716 and target-forward 1798
// / target-reverse 2679 — node 4042 is in neither.
//
// Before the assertion handler was installed, the *end() dereference at
// candidates.source_phantoms.end() segfaulted the OSRM node process via an
// out-of-bounds index.  With the handler installed, the BOOST_ASSERT throws
// util::exception, which the Node binding catches per-request.  This test
// uses the exact NodeIDs and phantom segment IDs captured from the failing
// production .osrm via instrumented OSRM build (see V1-V3 in the INC-296
// investigation write-up).
BOOST_AUTO_TEST_CASE(endpointsFromCandidates_throws_on_self_loop_path_INC296)
{
    using namespace osrm::engine;
    using namespace osrm::engine::routing_algorithms;

    // Production source phantom for Osama's start coord (-97.133251, 49.948735):
    // forward-only (the source road's bus-direction goes one way), with the
    // reverse segment marked SPECIAL_SEGMENTID (disabled).
    const PhantomNodeCandidates source_phantoms{makePhantom(/*fwd=*/6716, /*rev=*/SPECIAL_SEGMENTID)};
    // Production target phantom for Osama's end coord (-97.129906, 49.941653):
    // bidirectional.
    const PhantomNodeCandidates target_phantoms{makePhantom(/*fwd=*/1798, /*rev=*/2679)};
    const PhantomEndpointCandidates candidates{source_phantoms, target_phantoms};

    // The exact packed_leg → unpacked_nodes the production .osrm produced for
    // this route.  4042 is the node where the self-loop shortcut was applied.
    const std::vector<NodeID> path{4042u, 5176u, 9999u, 1917u, 4042u};

    BOOST_CHECK_THROW(endpointsFromCandidates(candidates, path), osrm::util::exception);
}

// Source-side mismatch only: path starts at non-phantom, but the path's end
// happens to coincide with a target phantom.  Locks down that we still throw —
// we don't silently accept a partial match.
BOOST_AUTO_TEST_CASE(endpointsFromCandidates_throws_when_only_source_unmatched)
{
    using namespace osrm::engine;
    using namespace osrm::engine::routing_algorithms;

    const PhantomNodeCandidates source_phantoms{makePhantom(/*fwd=*/100, /*rev=*/101)};
    const PhantomNodeCandidates target_phantoms{makePhantom(/*fwd=*/200, /*rev=*/201)};
    const PhantomEndpointCandidates candidates{source_phantoms, target_phantoms};

    const std::vector<NodeID> path{999u /*not a source*/, 200u /*matches target.fwd*/};

    BOOST_CHECK_THROW(endpointsFromCandidates(candidates, path), osrm::util::exception);
}

// Symmetric: source matches but target doesn't.
BOOST_AUTO_TEST_CASE(endpointsFromCandidates_throws_when_only_target_unmatched)
{
    using namespace osrm::engine;
    using namespace osrm::engine::routing_algorithms;

    const PhantomNodeCandidates source_phantoms{makePhantom(/*fwd=*/100, /*rev=*/101)};
    const PhantomNodeCandidates target_phantoms{makePhantom(/*fwd=*/200, /*rev=*/201)};
    const PhantomEndpointCandidates candidates{source_phantoms, target_phantoms};

    const std::vector<NodeID> path{100u /*matches source.fwd*/, 999u /*not a target*/};

    BOOST_CHECK_THROW(endpointsFromCandidates(candidates, path), osrm::util::exception);
}

BOOST_AUTO_TEST_SUITE_END()

// ---------------------------------------------------------------------------
// End-to-end CH search reproducer for INC-296 (V5).
//
// Drives `routingStep` directly against a minimal CH graph built in-test, with
// the specific pathology that triggers the production bug:
//
//   * Source phantom at node S=0 with a large forward-weight offset (700).
//     The forward heap starts with weight -700, so the heap key stays
//     negative across many nodes as the search expands.
//   * Target phantom at node T=2.
//   * Real forward edge S -> C (weight 50).
//   * Real backward edge B -> C (weight 50), where B=T (target adjacency for
//     the reverse search to reach C from T).
//   * A SELF-LOOP shortcut edge at node C=1, weight 1000, shortcut=true,
//     turn_id=any.  This kind of edge legitimately appears in a contracted
//     graph when a dead-end node was contracted out (its only neighbour was C,
//     so going to the dead-end and coming back is a valid -- if useless --
//     path through C).
//
// At node C the search sees
// forward_heap.GetKey(C) + reverse_heap.GetKey(C) = (-700+50) + 50 = -600.
//
// Pre-fix outcome (the bug): `routingStep`'s self-loop branch fired on
// `new_weight < 0` alone, found the self-loop edge at C with loop_weight =
// -600 + 1000 = 400, and set middle_node_id = C with upper_bound = 400.
// The resulting packed_leg = [C, C] unpacked to a closed loop whose
// endpoints did not match either phantom, segfaulting downstream.
//
// Post-fix outcome: with `new_weight < 0` removed from the self-loop branch
// condition and no force-loop signal set by the caller, the algorithm
// falls into `else if (new_weight >= 0)` -- which doesn't execute here
// (-600 < 0).  middle stays SPECIAL_NODEID and the search keeps exploring.
// This test asserts the post-fix behavior.

BOOST_AUTO_TEST_SUITE(routing_base_ch_search_self_loop_INC296)

namespace
{

// EdgeData type the CH facade is expected to return.  Mirrors
// contractor::QueryEdge::EdgeData layout that the production DataFacade
// returns from GetEdgeData(EdgeID).
using EdgeData = osrm::contractor::QueryEdge::EdgeData;

struct MinimalEdge
{
    NodeID source;
    NodeID target;
    EdgeData data;
};

// A facade exposing just enough of the BaseDataFacade interface for
// routingStep to operate on.  Returns hard-coded adjacency for a three-node
// graph with a self-loop shortcut at node 1.
//
// Doesn't inherit from DataFacade<ch::Algorithm> -- routingStep is now
// templated on FacadeT so type deduction lets us pass any class that
// supplies GetAdjacentEdgeRange / GetEdgeData / GetTarget.  Production
// callsites still pass DataFacade<ch::Algorithm> so behaviour there is
// unchanged.
class MinimalCHFacade
{
  public:
    MinimalCHFacade()
    {
        // Edge 0: S=0 -> C=1, weight 50, forward-only (forward search uses this)
        edges_.push_back({/*source=*/0, /*target=*/1,
                          EdgeData{/*turn_id=*/999,
                                   /*shortcut=*/false,
                                   /*weight=*/50,
                                   /*duration=*/50,
                                   /*distance=*/50.0f,
                                   /*forward=*/true,
                                   /*backward=*/false}});
        // Edge 1: T=2 -> C=1, weight 50, backward-only (reverse search uses this:
        // the reverse search expands edges at heapNode=T with `data.backward`,
        // reaching C).
        edges_.push_back({/*source=*/2, /*target=*/1,
                          EdgeData{/*turn_id=*/999,
                                   /*shortcut=*/false,
                                   /*weight=*/50,
                                   /*duration=*/50,
                                   /*distance=*/50.0f,
                                   /*forward=*/false,
                                   /*backward=*/true}});
        // Edge 2: SELF-LOOP at C=1, shortcut, weight 1000, bidirectional.
        // turn_id = some intermediate "contracted dead-end" node.
        edges_.push_back({/*source=*/1, /*target=*/1,
                          EdgeData{/*turn_id=*/42,
                                   /*shortcut=*/true,
                                   /*weight=*/1000,
                                   /*duration=*/1000,
                                   /*distance=*/1.0f,
                                   /*forward=*/true,
                                   /*backward=*/true}});

        // Build per-node adjacency.
        adjacency_[0] = {0};       // node 0 (S) owns edge 0
        adjacency_[2] = {1};       // node 2 (T) owns edge 1
        adjacency_[1] = {2};       // node 1 (C) owns edge 2 (the self-loop)
    }

    unsigned GetNumberOfNodes() const { return 3; }
    unsigned GetNumberOfEdges() const { return static_cast<unsigned>(edges_.size()); }

    const std::vector<EdgeID> &GetAdjacentEdgeRange(NodeID node) const
    {
        auto it = adjacency_.find(node);
        if (it == adjacency_.end())
        {
            static const std::vector<EdgeID> empty;
            return empty;
        }
        return it->second;
    }

    const EdgeData &GetEdgeData(EdgeID edge_id) const { return edges_.at(edge_id).data; }

    NodeID GetTarget(EdgeID edge_id) const { return edges_.at(edge_id).target; }

  private:
    std::vector<MinimalEdge> edges_;
    std::unordered_map<NodeID, std::vector<EdgeID>> adjacency_;
};

} // namespace

BOOST_AUTO_TEST_CASE(routingStep_does_not_pick_self_loop_when_new_weight_negative_INC296)
{
    using namespace osrm;
    using namespace osrm::engine;
    using namespace osrm::engine::routing_algorithms;

    MinimalCHFacade facade;

    // Build heaps the same way insertSourceInForwardHeap /
    // insertTargetInReverseHeap would have for our phantoms.
    // Source phantom on segment S=0 with large fwd offset (700) -> heap inserts
    // 0 with weight -700, parent=0 (self).
    // Target phantom on segment T=2 -> heap inserts 2 with weight 0,
    // parent=2 (self).
    using QueryHeap = SearchEngineData<ch::Algorithm>::QueryHeap;
    QueryHeap forward_heap{3 /*number_of_nodes hint*/};
    QueryHeap reverse_heap{3};

    forward_heap.Insert(/*node=*/0, /*weight=*/-700, /*data=*/HeapData{0});
    reverse_heap.Insert(/*node=*/2, /*weight=*/0, /*data=*/HeapData{2});

    NodeID middle = SPECIAL_NODEID;
    EdgeWeight upper_bound = INVALID_EDGE_WEIGHT;
    const EdgeWeight min_edge_offset = std::min(EdgeWeight{0}, forward_heap.MinKey());
    BOOST_CHECK_EQUAL(min_edge_offset, -700);

    // Drive the search step by step.  After each routingStep, we observe what
    // routingStep decided about middle / upper_bound.
    const std::vector<NodeID> no_force_loop;

    // Step 1: forward.  Pops node 0 (S).  Reverse heap doesn't have 0.  No
    // meeting.  Relaxes outgoing forward edges of 0: inserts node 1 (C) into
    // forward_heap with weight -700+50 = -650, parent=0.
    ch::routingStep<FORWARD_DIRECTION, ch::DISABLE_STALLING>(
        facade, forward_heap, reverse_heap, middle, upper_bound, min_edge_offset,
        no_force_loop, no_force_loop);
    BOOST_CHECK_EQUAL(middle, SPECIAL_NODEID);
    BOOST_CHECK(forward_heap.WasInserted(1));
    BOOST_CHECK_EQUAL(forward_heap.GetKey(1), -650);

    // Step 2: reverse.  Pops node 2 (T).  Forward heap doesn't have 2.  No
    // meeting.  Relaxes outgoing backward edges of 2: inserts node 1 (C) into
    // reverse_heap with weight 0+50 = 50, parent=2.
    ch::routingStep<REVERSE_DIRECTION, ch::DISABLE_STALLING>(
        facade, reverse_heap, forward_heap, middle, upper_bound, min_edge_offset,
        no_force_loop, no_force_loop);
    BOOST_CHECK_EQUAL(middle, SPECIAL_NODEID);
    BOOST_CHECK(reverse_heap.WasInserted(1));
    BOOST_CHECK_EQUAL(reverse_heap.GetKey(1), 50);

    // Step 3: forward.  Pops node 1 (C) with weight -650.  Reverse heap HAS 1
    // with weight 50.  new_weight = -650 + 50 = -600.
    //
    // Pre-fix this entered the self-loop branch and locked onto C.  Post-fix
    // the self-loop branch only fires when force_loop_*_nodes signal it (it
    // doesn't here -- no_force_loop), and the else-if (new_weight >= 0)
    // doesn't execute because -600 < 0.  So middle stays SPECIAL_NODEID and
    // the search continues exploring on subsequent steps.
    ch::routingStep<FORWARD_DIRECTION, ch::DISABLE_STALLING>(
        facade, forward_heap, reverse_heap, middle, upper_bound, min_edge_offset,
        no_force_loop, no_force_loop);

    BOOST_CHECK_EQUAL(middle, SPECIAL_NODEID);
    BOOST_CHECK_EQUAL(upper_bound, INVALID_EDGE_WEIGHT);
}

BOOST_AUTO_TEST_SUITE_END()

// ---------------------------------------------------------------------------
// Case-1 regression: same-segment force-loop.
//
// The Spec 3 fix removes the `new_weight < 0` fallback from routingStep's
// self-loop branch.  The legitimate use case the fallback was incidentally
// covering is "source and target on the same segment, source-behind-target":
// the route has to drive past the destination, loop around, and come back.
// callers must now explicitly populate force_loop_*_nodes for this case
// (directShortestPathSearch and getNetworkDistance do so via
// getForwardLoopNodes / requiresForwardLoop).
//
// These tests assert routingStep still picks the self-loop at the shared
// segment node when force_loop_forward_nodes contains that node, and that
// without the signal the algorithm no longer locks onto an unrelated self-
// loop (the INC-296 case-2 path).

BOOST_AUTO_TEST_SUITE(routing_base_ch_search_case1_force_loop)

namespace
{
using EdgeData = osrm::contractor::QueryEdge::EdgeData;

// One node with a self-loop edge representing the U-turn cost at the shared
// segment.  Source and target are both inserted at this node.
class SameSegmentUTurnFacade
{
  public:
    SameSegmentUTurnFacade()
    {
        edges_.push_back({/*source=*/0, /*target=*/0,
                          EdgeData{/*turn_id=*/0,
                                   /*shortcut=*/false,
                                   /*weight=*/1000,
                                   /*duration=*/1000,
                                   /*distance=*/1.0f,
                                   /*forward=*/true,
                                   /*backward=*/true}});
        adjacency_[0] = {0};
    }

    unsigned GetNumberOfNodes() const { return 1; }
    unsigned GetNumberOfEdges() const { return static_cast<unsigned>(edges_.size()); }

    const std::vector<EdgeID> &GetAdjacentEdgeRange(NodeID node) const
    {
        auto it = adjacency_.find(node);
        if (it == adjacency_.end())
        {
            static const std::vector<EdgeID> empty;
            return empty;
        }
        return it->second;
    }

    const EdgeData &GetEdgeData(EdgeID edge_id) const { return edges_.at(edge_id).data; }
    NodeID GetTarget(EdgeID edge_id) const { return edges_.at(edge_id).target; }

  private:
    struct MinimalEdge
    {
        NodeID source;
        NodeID target;
        EdgeData data;
    };
    std::vector<MinimalEdge> edges_;
    std::unordered_map<NodeID, std::vector<EdgeID>> adjacency_;
};
} // namespace

// With the force-loop signal set, the algorithm picks the self-loop at the
// shared node and produces a U-turn answer.  This is what directShortestPathSearch
// and getNetworkDistance now drive into routingStep after the Spec 3 caller fix.
BOOST_AUTO_TEST_CASE(routingStep_picks_self_loop_when_force_loop_forward_signal_set)
{
    using namespace osrm;
    using namespace osrm::engine;
    using namespace osrm::engine::routing_algorithms;

    SameSegmentUTurnFacade facade;

    using QueryHeap = SearchEngineData<ch::Algorithm>::QueryHeap;
    QueryHeap forward_heap{1};
    QueryHeap reverse_heap{1};

    // Source and target both on segment 0; source-behind-target means the
    // source phantom's forward weight offset > target's, so forward_heap's
    // key for node 0 is more negative than reverse_heap's.
    forward_heap.Insert(/*node=*/0, /*weight=*/-700, /*data=*/HeapData{0});
    reverse_heap.Insert(/*node=*/0, /*weight=*/0, /*data=*/HeapData{0});

    NodeID middle = SPECIAL_NODEID;
    EdgeWeight upper_bound = INVALID_EDGE_WEIGHT;
    const EdgeWeight min_edge_offset = std::min(EdgeWeight{0}, forward_heap.MinKey());

    // The caller (directShortestPathSearch / getNetworkDistance) has determined
    // source and target are on the same forward segment and require a forward
    // loop: force_loop_forward = {segment_node_id}.
    const std::vector<NodeID> force_loop_forward{0};
    const std::vector<NodeID> no_force_loop;

    // Pop node 0 (parent==0).  force_loop returns true.  Self-loop branch
    // finds the self-loop edge with weight 1000.  loop_weight = -700 + 1000 = 300.
    // middle = 0, upper_bound = 300.
    ch::routingStep<FORWARD_DIRECTION, ch::DISABLE_STALLING>(
        facade, forward_heap, reverse_heap, middle, upper_bound, min_edge_offset,
        force_loop_forward, no_force_loop);

    BOOST_CHECK_EQUAL(middle, NodeID{0});
    BOOST_CHECK_EQUAL(upper_bound, 300);
}

// Counter-test: same scenario WITHOUT the force-loop signal.  Pre-fix, the
// `new_weight < 0` fallback would have entered the self-loop branch anyway
// and produced the same answer.  Post-fix, the algorithm correctly does not
// lock onto the self-loop -- proving the fallback is gone, and demonstrating
// why the caller fix (#2 in Spec 3) is required for case 1.
BOOST_AUTO_TEST_CASE(routingStep_does_not_pick_self_loop_without_force_loop_signal)
{
    using namespace osrm;
    using namespace osrm::engine;
    using namespace osrm::engine::routing_algorithms;

    SameSegmentUTurnFacade facade;

    using QueryHeap = SearchEngineData<ch::Algorithm>::QueryHeap;
    QueryHeap forward_heap{1};
    QueryHeap reverse_heap{1};

    forward_heap.Insert(/*node=*/0, /*weight=*/-700, /*data=*/HeapData{0});
    reverse_heap.Insert(/*node=*/0, /*weight=*/0, /*data=*/HeapData{0});

    NodeID middle = SPECIAL_NODEID;
    EdgeWeight upper_bound = INVALID_EDGE_WEIGHT;
    const EdgeWeight min_edge_offset = std::min(EdgeWeight{0}, forward_heap.MinKey());

    const std::vector<NodeID> no_force_loop;

    ch::routingStep<FORWARD_DIRECTION, ch::DISABLE_STALLING>(
        facade, forward_heap, reverse_heap, middle, upper_bound, min_edge_offset,
        no_force_loop, no_force_loop);

    BOOST_CHECK_EQUAL(middle, SPECIAL_NODEID);
    BOOST_CHECK_EQUAL(upper_bound, INVALID_EDGE_WEIGHT);
}

BOOST_AUTO_TEST_SUITE_END()

// Note: requiresForwardLoop / requiresBackwardLoop are exercised end-to-end by
// the engine + cucumber integration tests via getForwardLoopNodes /
// getBackwardLoopNodes (which call them).  Targeted unit tests here are
// awkward because PhantomNode's is_valid_*_source/target bitfields are
// private and only settable through the heavy public constructor.  The Layer 2
// regression diff against real prod geographies is where these predicates'
// post-fix behaviour gets the most meaningful coverage.
