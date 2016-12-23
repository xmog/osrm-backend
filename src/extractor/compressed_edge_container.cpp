#include "extractor/compressed_edge_container.hpp"
#include "util/log.hpp"

#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <limits>
#include <string>

#include <iostream>

namespace osrm
{
namespace extractor
{

CompressedEdgeContainer::CompressedEdgeContainer()
{
    m_free_list.reserve(100);
    IncreaseFreeList();
}

void CompressedEdgeContainer::IncreaseFreeList()
{
    m_compressed_oneway_geometries.resize(m_compressed_oneway_geometries.size() + 100);
    for (unsigned i = 100; i > 0; --i)
    {
        m_free_list.emplace_back(free_list_maximum);
        ++free_list_maximum;
    }
}

bool CompressedEdgeContainer::HasEntryForID(const EdgeID edge_id) const
{
    auto iter = m_edge_id_to_list_index_map.find(edge_id);
    return iter != m_edge_id_to_list_index_map.end();
}

bool CompressedEdgeContainer::HasZippedEntryForForwardID(const EdgeID edge_id) const
{
    auto iter = m_forward_edge_id_to_zipped_index_map.find(edge_id);
    return iter != m_forward_edge_id_to_zipped_index_map.end();
}

bool CompressedEdgeContainer::HasZippedEntryForReverseID(const EdgeID edge_id) const
{
    auto iter = m_reverse_edge_id_to_zipped_index_map.find(edge_id);
    return iter != m_reverse_edge_id_to_zipped_index_map.end();
}

unsigned CompressedEdgeContainer::GetPositionForID(const EdgeID edge_id) const
{
    auto map_iterator = m_edge_id_to_list_index_map.find(edge_id);
    BOOST_ASSERT(map_iterator != m_edge_id_to_list_index_map.end());
    BOOST_ASSERT(map_iterator->second < m_compressed_oneway_geometries.size());
    return map_iterator->second;
}

unsigned CompressedEdgeContainer::GetZippedPositionForForwardID(const EdgeID edge_id) const
{
    auto map_iterator = m_forward_edge_id_to_zipped_index_map.find(edge_id);
    BOOST_ASSERT(map_iterator != m_forward_edge_id_to_zipped_index_map.end());
    BOOST_ASSERT(map_iterator->second < m_compressed_geometry_nodes.size());
    return map_iterator->second;
}

unsigned CompressedEdgeContainer::GetZippedPositionForReverseID(const EdgeID edge_id) const
{
    auto map_iterator = m_reverse_edge_id_to_zipped_index_map.find(edge_id);
    BOOST_ASSERT(map_iterator != m_reverse_edge_id_to_zipped_index_map.end());
    BOOST_ASSERT(map_iterator->second < m_compressed_geometry_nodes.size());
    return map_iterator->second;
}

void CompressedEdgeContainer::SerializeInternalVector(const std::string &path) const
{
    boost::filesystem::fstream geometry_out_stream(path, std::ios::binary | std::ios::out);
    const unsigned compressed_geometry_indices = m_compressed_geometry_index.size() + 1;
    const unsigned compressed_geometries = m_compressed_geometry_nodes.size();
    BOOST_ASSERT(std::numeric_limits<unsigned>::max() != compressed_geometry_indices);
    geometry_out_stream.write((char *)&compressed_geometry_indices, sizeof(unsigned));

    geometry_out_stream.write((char *)(m_compressed_geometry_index.data()),
                              sizeof(unsigned) * m_compressed_geometry_index.size());

    // sentinel element
    geometry_out_stream.write((char *)&(compressed_geometries), sizeof(unsigned));

    // number of geometry entries to follow
    geometry_out_stream.write((char *)&(compressed_geometries), sizeof(unsigned));

    // write compressed geometry node id's
    geometry_out_stream.write((char *)(m_compressed_geometry_nodes.data()),
                              sizeof(NodeID) * m_compressed_geometry_nodes.size());

    // write compressed geometry forward weights
    geometry_out_stream.write((char *)(m_compressed_geometry_fwd_weights.data()),
                              sizeof(EdgeWeight) * m_compressed_geometry_fwd_weights.size());

    // write compressed geometry reverse weights
    geometry_out_stream.write((char *)(m_compressed_geometry_rev_weights.data()),
                              sizeof(EdgeWeight) * m_compressed_geometry_rev_weights.size());

    // write compressed geometry forward durations
    geometry_out_stream.write((char *)(m_compressed_geometry_fwd_durations.data()),
                              sizeof(EdgeWeight) * m_compressed_geometry_fwd_durations.size());

    // write compressed geometry reverse durations
    geometry_out_stream.write((char *)(m_compressed_geometry_rev_durations.data()),
                              sizeof(EdgeWeight) * m_compressed_geometry_rev_durations.size());
}

// Adds info for a compressed edge to the container.   edge_id_2
// has been removed from the graph, so we have to save These edges/nodes
// have already been trimmed from the graph, this function just stores
// the original data for unpacking later.
//
//     edge_id_1               edge_id_2
//   ----------> via_node_id -----------> target_node_id
//     weight_1                weight_2
//     duration_1              duration_2
void CompressedEdgeContainer::CompressEdge(const EdgeID edge_id_1,
                                           const EdgeID edge_id_2,
                                           const NodeID via_node_id,
                                           const NodeID target_node_id,
                                           const EdgeWeight weight1,
                                           const EdgeWeight weight2,
                                           const EdgeWeight duration1,
                                           const EdgeWeight duration2)
{
    // remove super-trivial geometries
    BOOST_ASSERT(SPECIAL_EDGEID != edge_id_1);
    BOOST_ASSERT(SPECIAL_EDGEID != edge_id_2);
    BOOST_ASSERT(SPECIAL_NODEID != via_node_id);
    BOOST_ASSERT(SPECIAL_NODEID != target_node_id);
    BOOST_ASSERT(INVALID_EDGE_WEIGHT != weight1);
    BOOST_ASSERT(INVALID_EDGE_WEIGHT != weight2);

    // append list of removed edge_id plus via node to surviving edge id:
    // <surv_1, .. , surv_n, via_node_id, rem_1, .. rem_n
    //
    // General scheme:
    // 1. append via node id to list of edge_id_1
    // 2. find list for edge_id_2, if yes add all elements and delete it

    // Add via node id. List is created if it does not exist
    if (!HasEntryForID(edge_id_1))
    {
        // create a new entry in the map
        if (0 == m_free_list.size())
        {
            // make sure there is a place to put the entries
            IncreaseFreeList();
        }
        BOOST_ASSERT(!m_free_list.empty());
        m_edge_id_to_list_index_map[edge_id_1] = m_free_list.back();
        m_free_list.pop_back();
    }

    // find bucket index
    const auto iter = m_edge_id_to_list_index_map.find(edge_id_1);
    BOOST_ASSERT(iter != m_edge_id_to_list_index_map.end());
    const unsigned edge_bucket_id1 = iter->second;
    BOOST_ASSERT(edge_bucket_id1 == GetPositionForID(edge_id_1));
    BOOST_ASSERT(edge_bucket_id1 < m_compressed_oneway_geometries.size());

    std::vector<OnewayCompressedEdge> &edge_bucket_list1 =
        m_compressed_oneway_geometries[edge_bucket_id1];

    // note we don't save the start coordinate: it is implicitly given by edge 1
    // weight1 is the distance to the (currently) last coordinate in the bucket
    if (edge_bucket_list1.empty())
    {
        edge_bucket_list1.emplace_back(OnewayCompressedEdge{via_node_id, weight1, duration1});
    }

    BOOST_ASSERT(0 < edge_bucket_list1.size());
    BOOST_ASSERT(!edge_bucket_list1.empty());

    if (HasEntryForID(edge_id_2))
    {
        // second edge is not atomic anymore
        const unsigned list_to_remove_index = GetPositionForID(edge_id_2);
        BOOST_ASSERT(list_to_remove_index < m_compressed_oneway_geometries.size());

        std::vector<OnewayCompressedEdge> &edge_bucket_list2 =
            m_compressed_oneway_geometries[list_to_remove_index];

        // found an existing list, append it to the list of edge_id_1
        edge_bucket_list1.insert(
            edge_bucket_list1.end(), edge_bucket_list2.begin(), edge_bucket_list2.end());

        // remove the list of edge_id_2
        m_edge_id_to_list_index_map.erase(edge_id_2);
        BOOST_ASSERT(m_edge_id_to_list_index_map.end() ==
                     m_edge_id_to_list_index_map.find(edge_id_2));
        edge_bucket_list2.clear();
        BOOST_ASSERT(0 == edge_bucket_list2.size());
        m_free_list.emplace_back(list_to_remove_index);
        BOOST_ASSERT(list_to_remove_index == m_free_list.back());
    }
    else
    {
        // we are certain that the second edge is atomic.
        edge_bucket_list1.emplace_back(OnewayCompressedEdge{target_node_id, weight2, duration2});
    }
}

void CompressedEdgeContainer::AddUncompressedEdge(const EdgeID edge_id,
                                                  const NodeID target_node_id,
                                                  const EdgeWeight weight,
                                                  const EdgeWeight duration)
{
    // remove super-trivial geometries
    BOOST_ASSERT(SPECIAL_EDGEID != edge_id);
    BOOST_ASSERT(SPECIAL_NODEID != target_node_id);
    BOOST_ASSERT(INVALID_EDGE_WEIGHT != weight);

    // Add via node id. List is created if it does not exist
    if (!HasEntryForID(edge_id))
    {
        // create a new entry in the map
        if (0 == m_free_list.size())
        {
            // make sure there is a place to put the entries
            IncreaseFreeList();
        }
        BOOST_ASSERT(!m_free_list.empty());
        m_edge_id_to_list_index_map[edge_id] = m_free_list.back();
        m_free_list.pop_back();
    }

    // find bucket index
    const auto iter = m_edge_id_to_list_index_map.find(edge_id);
    BOOST_ASSERT(iter != m_edge_id_to_list_index_map.end());
    const unsigned edge_bucket_id = iter->second;
    BOOST_ASSERT(edge_bucket_id == GetPositionForID(edge_id));
    BOOST_ASSERT(edge_bucket_id < m_compressed_oneway_geometries.size());

    std::vector<OnewayCompressedEdge> &edge_bucket_list =
        m_compressed_oneway_geometries[edge_bucket_id];

    // note we don't save the start coordinate: it is implicitly given by edge_id
    // weight is the distance to the (currently) last coordinate in the bucket
    // Don't re-add this if it's already in there.
    if (edge_bucket_list.empty())
    {
        edge_bucket_list.emplace_back(OnewayCompressedEdge{target_node_id, weight, duration});
    }
}

void CompressedEdgeContainer::InitializeBothwayVector()
{
    m_compressed_geometry_index.reserve(m_compressed_oneway_geometries.size());
    m_compressed_geometry_nodes.reserve(m_compressed_oneway_geometries.size());
    m_compressed_geometry_fwd_weights.reserve(m_compressed_oneway_geometries.size());
    m_compressed_geometry_rev_weights.reserve(m_compressed_oneway_geometries.size());
    m_compressed_geometry_fwd_durations.reserve(m_compressed_oneway_geometries.size());
    m_compressed_geometry_rev_durations.reserve(m_compressed_oneway_geometries.size());
}

unsigned CompressedEdgeContainer::ZipEdges(const EdgeID f_edge_id, const EdgeID r_edge_id)
{
    const auto &forward_bucket = GetBucketReference(f_edge_id);
    const auto &reverse_bucket = GetBucketReference(r_edge_id);

    BOOST_ASSERT(forward_bucket.size() == reverse_bucket.size());

    const unsigned zipped_geometry_id = m_compressed_geometry_index.size();
    m_forward_edge_id_to_zipped_index_map[f_edge_id] = zipped_geometry_id;
    m_reverse_edge_id_to_zipped_index_map[r_edge_id] = zipped_geometry_id;

    m_compressed_geometry_index.emplace_back(m_compressed_geometry_nodes.size());

    const auto &first_node = reverse_bucket.back();

    m_compressed_geometry_nodes.emplace_back(first_node.node_id);
    m_compressed_geometry_fwd_weights.emplace_back(INVALID_EDGE_WEIGHT);
    m_compressed_geometry_rev_weights.emplace_back(first_node.weight);
    m_compressed_geometry_fwd_durations.emplace_back(INVALID_EDGE_WEIGHT);
    m_compressed_geometry_rev_durations.emplace_back(first_node.duration);

    for (std::size_t i = 0; i < forward_bucket.size() - 1; ++i)
    {
        const auto &fwd_node = forward_bucket.at(i);
        const auto &rev_node = reverse_bucket.at(reverse_bucket.size() - 2 - i);

        BOOST_ASSERT(fwd_node.node_id == rev_node.node_id);

        m_compressed_geometry_nodes.emplace_back(fwd_node.node_id);
        m_compressed_geometry_fwd_weights.emplace_back(fwd_node.weight);
        m_compressed_geometry_rev_weights.emplace_back(rev_node.weight);
        m_compressed_geometry_fwd_durations.emplace_back(fwd_node.duration);
        m_compressed_geometry_rev_durations.emplace_back(rev_node.duration);
    }

    const auto &last_node = forward_bucket.back();

    m_compressed_geometry_nodes.emplace_back(last_node.node_id);
    m_compressed_geometry_fwd_weights.emplace_back(last_node.weight);
    m_compressed_geometry_rev_weights.emplace_back(INVALID_EDGE_WEIGHT);
    m_compressed_geometry_fwd_durations.emplace_back(last_node.duration);
    m_compressed_geometry_rev_durations.emplace_back(INVALID_EDGE_WEIGHT);

    return zipped_geometry_id;
}

void CompressedEdgeContainer::PrintStatistics() const
{
    const uint64_t compressed_edges = m_compressed_oneway_geometries.size();
    BOOST_ASSERT(0 == compressed_edges % 2);
    BOOST_ASSERT(m_compressed_oneway_geometries.size() + m_free_list.size() > 0);

    uint64_t compressed_geometries = 0;
    uint64_t longest_chain_length = 0;
    for (const std::vector<OnewayCompressedEdge> &current_vector : m_compressed_oneway_geometries)
    {
        compressed_geometries += current_vector.size();
        longest_chain_length = std::max(longest_chain_length, (uint64_t)current_vector.size());
    }

    util::Log() << "Geometry successfully removed:"
                   "\n  compressed edges: "
                << compressed_edges << "\n  compressed geometries: " << compressed_geometries
                << "\n  longest chain length: " << longest_chain_length << "\n  cmpr ratio: "
                << ((float)compressed_edges / std::max(compressed_geometries, (uint64_t)1))
                << "\n  avg chain length: "
                << (float)compressed_geometries / std::max((uint64_t)1, compressed_edges);
}

const CompressedEdgeContainer::OnewayEdgeBucket &
CompressedEdgeContainer::GetBucketReference(const EdgeID edge_id) const
{
    const unsigned index = m_edge_id_to_list_index_map.at(edge_id);
    return m_compressed_oneway_geometries.at(index);
}

// Since all edges are technically in the compressed geometry container,
// regardless of whether a compressed edge actually contains multiple
// original segments, we use 'Trivial' here to describe compressed edges
// that only contain one original segment
bool CompressedEdgeContainer::IsTrivial(const EdgeID edge_id) const
{
    const auto &bucket = GetBucketReference(edge_id);
    return bucket.size() == 1;
}

NodeID CompressedEdgeContainer::GetFirstEdgeTargetID(const EdgeID edge_id) const
{
    const auto &bucket = GetBucketReference(edge_id);
    BOOST_ASSERT(bucket.size() >= 1);
    return bucket.front().node_id;
}
NodeID CompressedEdgeContainer::GetLastEdgeTargetID(const EdgeID edge_id) const
{
    const auto &bucket = GetBucketReference(edge_id);
    BOOST_ASSERT(bucket.size() >= 1);
    return bucket.back().node_id;
}
NodeID CompressedEdgeContainer::GetLastEdgeSourceID(const EdgeID edge_id) const
{
    const auto &bucket = GetBucketReference(edge_id);
    BOOST_ASSERT(bucket.size() >= 2);
    return bucket[bucket.size() - 2].node_id;
}
}
}
