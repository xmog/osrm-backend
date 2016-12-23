#include "contractor/contractor.hpp"
#include "contractor/crc32_processor.hpp"
#include "contractor/graph_contractor.hpp"

#include "extractor/compressed_edge_container.hpp"
#include "extractor/edge_based_graph_factory.hpp"
#include "extractor/node_based_edge.hpp"

#include "storage/io.hpp"
#include "util/exception.hpp"
#include "util/exception_utils.hpp"
#include "util/graph_loader.hpp"
#include "util/integer_range.hpp"
#include "util/io.hpp"
#include "util/log.hpp"
#include "util/static_graph.hpp"
#include "util/static_rtree.hpp"
#include "util/string_util.hpp"
#include "util/timing_util.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/functional/hash.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/spirit/include/qi.hpp>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/parallel_invoke.h>
#include <tbb/parallel_sort.h>
#include <tbb/spin_mutex.h>

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <thread>
#include <tuple>
#include <vector>

namespace std
{

template <> struct hash<std::pair<OSMNodeID, OSMNodeID>>
{
    std::size_t operator()(const std::pair<OSMNodeID, OSMNodeID> &k) const noexcept
    {
        return static_cast<uint64_t>(k.first) ^ (static_cast<uint64_t>(k.second) << 12);
    }
};

template <> struct hash<std::tuple<OSMNodeID, OSMNodeID, OSMNodeID>>
{
    std::size_t operator()(const std::tuple<OSMNodeID, OSMNodeID, OSMNodeID> &k) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, static_cast<uint64_t>(std::get<0>(k)));
        boost::hash_combine(seed, static_cast<uint64_t>(std::get<1>(k)));
        boost::hash_combine(seed, static_cast<uint64_t>(std::get<2>(k)));
        return seed;
    }
};
}

namespace osrm
{
namespace contractor
{

// Returns duration in deci-seconds
inline EdgeWeight distanceAndSpeedToWeight(double distance_in_meters, double speed_in_kmh)
{
    BOOST_ASSERT(speed_in_kmh > 0);
    const double speed_in_ms = speed_in_kmh / 3.6;
    const double duration = distance_in_meters / speed_in_ms;
    return std::max<EdgeWeight>(1, static_cast<EdgeWeight>(std::round(duration * 10)));
}

// Returns updated edge weight
template <class IterType>
void getNewWeight(IterType speed_iter,
                  const double &segment_length,
                  const std::vector<std::string> &segment_speed_filenames,
                  const EdgeWeight current_duration,
                  const double log_edge_updates_factor,
                  EdgeWeight &new_segment_weight,
                  EdgeWeight &new_segment_duration)
{
    new_segment_duration =
        (speed_iter->speed_source.speed > 0)
            ? distanceAndSpeedToWeight(segment_length, speed_iter->speed_source.speed)
            : INVALID_EDGE_WEIGHT;
    new_segment_weight =
        (speed_iter->speed_source.weight == INVALID_EDGE_WEIGHT)
            ? new_segment_duration
            : distanceAndSpeedToWeight(
                  segment_length,
                  speed_iter->speed_source.weight); // TODO decouple weight and duration

    // the check here is enabled by the `--edge-weight-updates-over-factor` flag
    // it logs a warning if the new weight exceeds a heuristic of what a reasonable weight update is
    if (log_edge_updates_factor > 0 && current_duration != 0)
    {
        auto new_secs = new_segment_duration / 10.0;
        auto old_secs = current_duration / 10.0;
        auto approx_original_speed = (segment_length / old_secs) * 3.6;
        if (current_duration >= (new_segment_weight * log_edge_updates_factor))
        {
            auto speed_file = segment_speed_filenames.at(speed_iter->speed_source.source - 1);
            util::Log(logWARNING) << "[weight updates] Edge weight update from " << old_secs
                                  << "s to " << new_secs
                                  << "s  New speed: " << speed_iter->speed_source.speed << " kph"
                                  << ". Old speed: " << approx_original_speed << " kph"
                                  << ". Segment length: " << segment_length << " m"
                                  << ". Segment: " << speed_iter->segment.from << ","
                                  << speed_iter->segment.to << " based on " << speed_file;
        }
    }
}

int Contractor::Run()
{
    if (config.core_factor > 1.0 || config.core_factor < 0)
    {
        throw util::exception("Core factor must be between 0.0 to 1.0 (inclusive)" + SOURCE_REF);
    }

    TIMER_START(preparing);

    util::Log() << "Reading node weights.";
    std::vector<EdgeWeight> node_weights;
    std::string node_file_name = config.osrm_input_path.string() + ".enw";

    {
        storage::io::FileReader node_file(node_file_name,
                                          storage::io::FileReader::VerifyFingerprint);
        node_file.DeserializeVector(node_weights);
    }
    util::Log() << "Done reading node weights.";

    util::Log() << "Loading edge-expanded graph representation";

    util::DeallocatingVector<extractor::EdgeBasedEdge> edge_based_edge_list;

    EdgeID max_edge_id = LoadEdgeExpandedGraph(config.edge_based_graph_path,
                                               edge_based_edge_list,
                                               node_weights,
                                               config.edge_segment_lookup_path,
                                               config.turn_weight_penalties_path,
                                               config.turn_duration_penalties_path,
                                               config.turn_penalties_index_path,
                                               config.segment_speed_lookup_paths,
                                               config.turn_penalty_lookup_paths,
                                               config.node_based_graph_path,
                                               config.geometry_path,
                                               config.datasource_names_path,
                                               config.datasource_indexes_path,
                                               config.rtree_leaf_path,
                                               config.log_edge_updates_factor);

    // Contracting the edge-expanded graph

    TIMER_START(contraction);
    std::vector<bool> is_core_node;
    std::vector<float> node_levels;
    if (config.use_cached_priority)
    {
        ReadNodeLevels(node_levels);
    }

    util::DeallocatingVector<QueryEdge> contracted_edge_list;
    ContractGraph(max_edge_id,
                  edge_based_edge_list,
                  contracted_edge_list,
                  std::move(node_weights),
                  is_core_node,
                  node_levels);
    TIMER_STOP(contraction);

    util::Log() << "Contraction took " << TIMER_SEC(contraction) << " sec";

    std::size_t number_of_used_edges = WriteContractedGraph(max_edge_id, contracted_edge_list);
    WriteCoreNodeMarker(std::move(is_core_node));
    if (!config.use_cached_priority)
    {
        WriteNodeLevels(std::move(node_levels));
    }

    TIMER_STOP(preparing);

    const auto nodes_per_second =
        static_cast<std::uint64_t>((max_edge_id + 1) / TIMER_SEC(contraction));
    const auto edges_per_second =
        static_cast<std::uint64_t>(number_of_used_edges / TIMER_SEC(contraction));

    util::Log() << "Preprocessing : " << TIMER_SEC(preparing) << " seconds";
    util::Log() << "Contraction: " << nodes_per_second << " nodes/sec and " << edges_per_second
                << " edges/sec";

    util::Log() << "finished preprocessing";

    return 0;
}

// Utilities for LoadEdgeExpandedGraph to restore my sanity
namespace
{

struct Segment final
{
    OSMNodeID from, to;
    bool operator==(const Segment &other) const
    {
        return std::tie(from, to) == std::tie(other.from, other.to);
    }
};

struct SpeedSource final
{
    unsigned speed;
    unsigned weight;
    std::uint8_t source;
};

struct SegmentSpeedSource final
{
    Segment segment;
    SpeedSource speed_source;
    // < operator is overloaded here to return a > comparison to be used by the
    // std::lower_bound() call in the find() function
    bool operator<(const SegmentSpeedSource &other) const
    {
        return std::tie(segment.from, segment.to) > std::tie(other.segment.from, other.segment.to);
    }
};

struct Turn final
{
    OSMNodeID from, via, to;
    bool operator==(const Turn &other) const
    {
        return std::tie(from, via, to) == std::tie(other.from, other.via, other.to);
    }
};

struct PenaltySource final
{
    double penalty;
    std::uint8_t source;
};
struct TurnPenaltySource final
{
    Turn segment;
    PenaltySource penalty_source;
    // < operator is overloaded here to return a > comparison to be used by the
    // std::lower_bound() call in the find() function
    bool operator<(const TurnPenaltySource &other) const
    {
        return std::tie(segment.from, segment.via, segment.to) >
               std::tie(other.segment.from, other.segment.via, other.segment.to);
    }
};
using TurnPenaltySourceFlatMap = std::vector<TurnPenaltySource>;
using SegmentSpeedSourceFlatMap = std::vector<SegmentSpeedSource>;

// Find is a binary Search over a flattened key,val Segment storage
// It takes the flat map and a Segment/PenaltySource object that has an overloaded
// `==` operator, to make the std::lower_bound call work generically
template <typename FlatMap, typename SegmentKey>
auto find(const FlatMap &map, const SegmentKey &key)
{
    const auto last = end(map);
    auto it = std::lower_bound(begin(map), last, key);

    if (it != last && (it->segment == key.segment))
        return it;

    return last;
}

// Functions for parsing files and creating lookup tables

SegmentSpeedSourceFlatMap
parse_segment_lookup_from_csv_files(const std::vector<std::string> &segment_speed_filenames)
{
    // TODO: shares code with turn penalty lookup parse function

    using Mutex = tbb::spin_mutex;

    // Loaded and parsed in parallel, at the end we combine results in a flattened map-ish view
    SegmentSpeedSourceFlatMap flatten;
    Mutex flatten_mutex;

    const auto parse_segment_speed_file = [&](const std::size_t idx) {
        const auto file_id = idx + 1; // starts at one, zero means we assigned the weight
        const auto filename = segment_speed_filenames[idx];

        storage::io::FileReader segment_speed_file_reader(
            filename, storage::io::FileReader::HasNoFingerprint);

        SegmentSpeedSourceFlatMap local;

        std::size_t line_number = 0;

        std::for_each(
            segment_speed_file_reader.GetLineIteratorBegin(),
            segment_speed_file_reader.GetLineIteratorEnd(),
            [&](const std::string &line) {
                ++line_number;

                using namespace boost::spirit::qi;

                std::uint64_t from_node_id{};
                std::uint64_t to_node_id{};
                unsigned speed{};
                unsigned weight = INVALID_EDGE_WEIGHT;

                auto it = begin(line);
                const auto last = end(line);

                // The ulong_long -> uint64_t will likely break on 32bit platforms
                const auto ok = parse(it,
                                      last,
                                      (ulong_long >> ',' >> ulong_long >> ',' >> uint_ >>
                                       -(',' >> uint_) >> *(',' >> *char_)),
                                      from_node_id,
                                      to_node_id,
                                      speed,
                                      weight);

                if (!ok || it != last)
                {
                    const std::string message{"Segment speed file " + filename +
                                              " malformed on line " + std::to_string(line_number)};
                    throw util::exception(message + SOURCE_REF);
                }

                SegmentSpeedSource val{{OSMNodeID{from_node_id}, OSMNodeID{to_node_id}},
                                       {speed, weight, static_cast<std::uint8_t>(file_id)}};

                local.push_back(std::move(val));
            });

        util::Log() << "Loaded speed file " << filename << " with " << local.size() << " speeds";

        {
            Mutex::scoped_lock _{flatten_mutex};

            flatten.insert(end(flatten),
                           std::make_move_iterator(begin(local)),
                           std::make_move_iterator(end(local)));
        }
    };

    try
    {
        tbb::parallel_for(std::size_t{0}, segment_speed_filenames.size(), parse_segment_speed_file);
    }
    catch (const tbb::captured_exception &e)
    {
        throw util::exception(e.what() + SOURCE_REF);
    }

    // With flattened map-ish view of all the files, sort and unique them on from,to,source
    // The greater '>' is used here since we want to give files later on higher precedence
    const auto sort_by = [](const SegmentSpeedSource &lhs, const SegmentSpeedSource &rhs) {
        return std::tie(lhs.segment.from, lhs.segment.to, lhs.speed_source.source) >
               std::tie(rhs.segment.from, rhs.segment.to, rhs.speed_source.source);
    };

    std::stable_sort(begin(flatten), end(flatten), sort_by);

    // Unique only on from,to to take the source precedence into account and remove duplicates
    const auto unique_by = [](const SegmentSpeedSource &lhs, const SegmentSpeedSource &rhs) {
        return std::tie(lhs.segment.from, lhs.segment.to) ==
               std::tie(rhs.segment.from, rhs.segment.to);
    };

    const auto it = std::unique(begin(flatten), end(flatten), unique_by);

    flatten.erase(it, end(flatten));

    util::Log() << "In total loaded " << segment_speed_filenames.size()
                << " speed file(s) with a total of " << flatten.size() << " unique values";

    return flatten;
}

TurnPenaltySourceFlatMap
parse_turn_penalty_lookup_from_csv_files(const std::vector<std::string> &turn_penalty_filenames)
{
    using Mutex = tbb::spin_mutex;

    // TODO: shares code with turn penalty lookup parse function
    TurnPenaltySourceFlatMap map;
    Mutex flatten_mutex;

    const auto parse_turn_penalty_file = [&](const std::size_t idx) {
        const auto file_id = idx + 1; // starts at one, zero means we assigned the weight
        const auto filename = turn_penalty_filenames[idx];

        storage::io::FileReader turn_penalty_file_reader(filename,
                                                         storage::io::FileReader::HasNoFingerprint);
        TurnPenaltySourceFlatMap local;

        std::size_t line_number = 0;

        std::for_each(
            turn_penalty_file_reader.GetLineIteratorBegin(),
            turn_penalty_file_reader.GetLineIteratorEnd(),
            [&](const std::string &line) {
                ++line_number;

                using namespace boost::spirit::qi;

                std::uint64_t from_node_id{};
                std::uint64_t via_node_id{};
                std::uint64_t to_node_id{};
                double penalty{};

                auto it = begin(line);
                const auto last = end(line);

                // The ulong_long -> uint64_t will likely break on 32bit platforms
                const auto ok = parse(it,
                                      last, //
                                      (ulong_long >> ',' >> ulong_long >> ',' >> ulong_long >>
                                       ',' >> double_ >> *(',' >> *char_)), //
                                      from_node_id,
                                      via_node_id,
                                      to_node_id,
                                      penalty); //

                if (!ok || it != last)
                {
                    const std::string message{"Turn penalty file " + filename +
                                              " malformed on line " + std::to_string(line_number)};
                    throw util::exception(message + SOURCE_REF);
                }

                TurnPenaltySource val{
                    {OSMNodeID{from_node_id}, OSMNodeID{via_node_id}, OSMNodeID{to_node_id}},
                    {penalty, static_cast<std::uint8_t>(file_id)}};
                local.push_back(std::move(val));
            });

        util::Log() << "Loaded penalty file " << filename << " with " << local.size()
                    << " turn penalties";

        {
            Mutex::scoped_lock _{flatten_mutex};

            map.insert(end(map),
                       std::make_move_iterator(begin(local)),
                       std::make_move_iterator(end(local)));
        }
    };

    try
    {
        tbb::parallel_for(std::size_t{0}, turn_penalty_filenames.size(), parse_turn_penalty_file);
    }
    catch (const tbb::captured_exception &e)
    {
        throw util::exception(e.what() + SOURCE_REF);
    }

    // With flattened map-ish view of all the files, sort and unique them on from,to,source
    // The greater '>' is used here since we want to give files later on higher precedence
    const auto sort_by = [](const TurnPenaltySource &lhs, const TurnPenaltySource &rhs) {
        return std::tie(
                   lhs.segment.from, lhs.segment.via, lhs.segment.to, lhs.penalty_source.source) >
               std::tie(
                   rhs.segment.from, rhs.segment.via, rhs.segment.to, rhs.penalty_source.source);
    };

    std::stable_sort(begin(map), end(map), sort_by);

    // Unique only on from,to to take the source precedence into account and remove duplicates
    const auto unique_by = [](const TurnPenaltySource &lhs, const TurnPenaltySource &rhs) {
        return std::tie(lhs.segment.from, lhs.segment.via, lhs.segment.to) ==
               std::tie(rhs.segment.from, rhs.segment.via, rhs.segment.to);
    };

    const auto it = std::unique(begin(map), end(map), unique_by);

    map.erase(it, end(map));

    util::Log() << "In total loaded " << turn_penalty_filenames.size()
                << " turn penalty file(s) with a total of " << map.size() << " unique values";

    return map;
}
} // anon ns

EdgeID Contractor::LoadEdgeExpandedGraph(
    std::string const &edge_based_graph_filename,
    util::DeallocatingVector<extractor::EdgeBasedEdge> &edge_based_edge_list,
    std::vector<EdgeWeight> &node_weights,
    const std::string &edge_segment_lookup_filename,
    const std::string &turn_weight_penalties_filename,
    const std::string &turn_duration_penalties_filename,
    const std::string &turn_penalties_index_filename,
    const std::vector<std::string> &segment_speed_filenames,
    const std::vector<std::string> &turn_penalty_filenames,
    const std::string &nodes_filename,
    const std::string &geometry_filename,
    const std::string &datasource_names_filename,
    const std::string &datasource_indexes_filename,
    const std::string &rtree_leaf_filename,
    const double log_edge_updates_factor)
{
    if (segment_speed_filenames.size() > 255 || turn_penalty_filenames.size() > 255)
        throw util::exception("Limit of 255 segment speed and turn penalty files each reached" +
                              SOURCE_REF);

    util::Log() << "Opening " << edge_based_graph_filename;

    auto mmap_file = [](const std::string &filename, boost::interprocess::mode_t mode) {
        using boost::interprocess::file_mapping;
        using boost::interprocess::mapped_region;

        try
        {
            const file_mapping mapping{filename.c_str(), mode};
            mapped_region region{mapping, mode};
            region.advise(mapped_region::advice_sequential);
            return region;
        }
        catch (const std::exception &e)
        {
            util::Log(logERROR) << "Error while trying to mmap " + filename + ": " + e.what();
            throw;
        }
    };

    const auto edge_based_graph_region =
        mmap_file(edge_based_graph_filename, boost::interprocess::read_only);

    const bool update_edge_weights = !segment_speed_filenames.empty();
    const bool update_turn_penalties = !turn_penalty_filenames.empty();

    const auto turn_weight_penalties_region = [&] {
        if (update_edge_weights || update_turn_penalties)
        {
            return mmap_file(turn_weight_penalties_filename, boost::interprocess::read_write);
        }
        return boost::interprocess::mapped_region();
    }();
    const auto turn_duration_penalties_region = [&] {
        if (update_edge_weights || update_turn_penalties)
        {
            return mmap_file(turn_duration_penalties_filename, boost::interprocess::read_only);
        }
        return boost::interprocess::mapped_region();
    }();
    const auto turn_penalties_index_region = [&] {
        if (update_edge_weights || update_turn_penalties)
        {
            return mmap_file(turn_penalties_index_filename, boost::interprocess::read_only);
        }
        return boost::interprocess::mapped_region();
    }();

    const auto edge_segment_region = [&] {
        if (update_edge_weights || update_turn_penalties)
        {
            return mmap_file(edge_segment_lookup_filename, boost::interprocess::read_only);
        }
        return boost::interprocess::mapped_region();
    }();

// Set the struct packing to 1 byte word sizes.  This prevents any padding.  We only use
// this struct once, so any alignment penalty is trivial.  If this is *not* done, then
// the struct will be padded out by an extra 4 bytes, and sizeof() will mean we read
// too much data from the original file.
#pragma pack(push, r1, 1)
    struct EdgeBasedGraphHeader
    {
        util::FingerPrint fingerprint;
        std::uint64_t number_of_edges;
        EdgeID max_edge_id;
    };
#pragma pack(pop, r1)

    const EdgeBasedGraphHeader graph_header =
        *(reinterpret_cast<const EdgeBasedGraphHeader *>(edge_based_graph_region.get_address()));

    const util::FingerPrint fingerprint_valid = util::FingerPrint::GetValid();
    graph_header.fingerprint.TestContractor(fingerprint_valid);

    edge_based_edge_list.resize(graph_header.number_of_edges);
    util::Log() << "Reading " << graph_header.number_of_edges << " edges from the edge based graph";

    SegmentSpeedSourceFlatMap segment_speed_lookup;
    TurnPenaltySourceFlatMap turn_penalty_lookup;

    const auto parse_segment_speeds = [&] {
        if (update_edge_weights)
            segment_speed_lookup = parse_segment_lookup_from_csv_files(segment_speed_filenames);
    };

    const auto parse_turn_penalties = [&] {
        if (update_turn_penalties)
            turn_penalty_lookup = parse_turn_penalty_lookup_from_csv_files(turn_penalty_filenames);
    };

    // If we update the edge weights, this file will hold the datasource information for each
    // segment; the other files will also be conditionally filled concurrently if we make an update
    std::vector<uint8_t> m_geometry_datasource;

    std::vector<extractor::QueryNode> internal_to_external_node_map;
    std::vector<unsigned> m_geometry_indices;
    std::vector<NodeID> m_geometry_node_list;
    std::vector<EdgeWeight> m_geometry_fwd_weight_list;
    std::vector<EdgeWeight> m_geometry_rev_weight_list;
    std::vector<EdgeWeight> m_geometry_fwd_duration_list;
    std::vector<EdgeWeight> m_geometry_rev_duration_list;

    const auto maybe_load_internal_to_external_node_map = [&] {
        if (!(update_edge_weights || update_turn_penalties))
            return;

        storage::io::FileReader nodes_file(nodes_filename,
                                           storage::io::FileReader::HasNoFingerprint);

        nodes_file.DeserializeVector(internal_to_external_node_map);

    };

    const auto maybe_load_geometries = [&] {
        if (!(update_edge_weights || update_turn_penalties))
            return;

        storage::io::FileReader geometry_file(geometry_filename,
                                              storage::io::FileReader::HasNoFingerprint);
        const auto number_of_indices = geometry_file.ReadElementCount32();
        m_geometry_indices.resize(number_of_indices);
        geometry_file.ReadInto(m_geometry_indices.data(), number_of_indices);

        const auto number_of_compressed_geometries = geometry_file.ReadElementCount32();

        BOOST_ASSERT(m_geometry_indices.back() == number_of_compressed_geometries);
        m_geometry_node_list.resize(number_of_compressed_geometries);
        m_geometry_fwd_weight_list.resize(number_of_compressed_geometries);
        m_geometry_rev_weight_list.resize(number_of_compressed_geometries);
        m_geometry_fwd_duration_list.resize(number_of_compressed_geometries);
        m_geometry_rev_duration_list.resize(number_of_compressed_geometries);

        if (number_of_compressed_geometries > 0)
        {
            geometry_file.ReadInto(m_geometry_node_list.data(), number_of_compressed_geometries);
            geometry_file.ReadInto(m_geometry_fwd_weight_list.data(),
                                   number_of_compressed_geometries);
            geometry_file.ReadInto(m_geometry_rev_weight_list.data(),
                                   number_of_compressed_geometries);
            geometry_file.ReadInto(m_geometry_fwd_duration_list.data(),
                                   number_of_compressed_geometries);
            geometry_file.ReadInto(m_geometry_rev_duration_list.data(),
                                   number_of_compressed_geometries);
        }
    };

    // Folds all our actions into independently concurrently executing lambdas
    tbb::parallel_invoke(parse_segment_speeds,
                         parse_turn_penalties, //
                         maybe_load_internal_to_external_node_map,
                         maybe_load_geometries);

    if (update_edge_weights || update_turn_penalties)
    {
        // Here, we have to update the compressed geometry weights
        // First, we need the external-to-internal node lookup table

        // This is a list of the "data source id" for every segment in the compressed
        // geometry container.  We assume that everything so far has come from the
        // profile (data source 0).  Here, we replace the 0's with the index of the
        // CSV file that supplied the value that gets used for that segment, then
        // we write out this list so that it can be returned by the debugging
        // vector tiles later on.
        m_geometry_datasource.resize(m_geometry_fwd_duration_list.size(), 0);

        // Now, we iterate over all the segments stored in the StaticRTree, updating
        // the packed geometry weights in the `.geometries` file (note: we do not
        // update the RTree itself, we just use the leaf nodes to iterate over all segments)
        using LeafNode = util::StaticRTree<extractor::EdgeBasedNode>::LeafNode;

        using boost::interprocess::mapped_region;

        auto region = mmap_file(rtree_leaf_filename.c_str(), boost::interprocess::read_only);
        region.advise(mapped_region::advice_willneed);

        const auto bytes = region.get_size();
        const auto first = static_cast<const LeafNode *>(region.get_address());
        const auto last = first + (bytes / sizeof(LeafNode));

        // vector to count used speeds for logging
        // size offset by one since index 0 is used for speeds not from external file
        using counters_type = std::vector<std::size_t>;
        std::size_t num_counters = segment_speed_filenames.size() + 1;
        tbb::enumerable_thread_specific<counters_type> segment_speeds_counters(
            counters_type(num_counters, 0));
        const constexpr auto LUA_SOURCE = 0;

        tbb::parallel_for_each(first, last, [&](const LeafNode &current_node) {
            auto &counters = segment_speeds_counters.local();
            for (size_t i = 0; i < current_node.object_count; i++)
            {
                const auto &leaf_object = current_node.objects[i];
                extractor::QueryNode *u;
                extractor::QueryNode *v;

                const unsigned forward_begin =
                    m_geometry_indices.at(leaf_object.packed_geometry_id);
                const auto current_fwd_duration =
                    m_geometry_fwd_duration_list[forward_begin + leaf_object.fwd_segment_position];

                u = &(internal_to_external_node_map
                          [m_geometry_node_list[forward_begin + leaf_object.fwd_segment_position]]);
                v = &(internal_to_external_node_map
                          [m_geometry_node_list[forward_begin + leaf_object.fwd_segment_position +
                                                1]]);

                const double segment_length = util::coordinate_calculation::greatCircleDistance(
                    util::Coordinate{u->lon, u->lat}, util::Coordinate{v->lon, v->lat});

                auto forward_speed_iter = find(
                    segment_speed_lookup, SegmentSpeedSource{{u->node_id, v->node_id}, {0, 0, 0}});
                if (forward_speed_iter != segment_speed_lookup.end())
                {
                    EdgeWeight new_segment_weight, new_segment_duration;
                    getNewWeight(forward_speed_iter,
                                 segment_length,
                                 segment_speed_filenames,
                                 current_fwd_duration,
                                 log_edge_updates_factor,
                                 new_segment_weight,
                                 new_segment_duration);

                    m_geometry_fwd_weight_list[forward_begin + 1 +
                                               leaf_object.fwd_segment_position] =
                        new_segment_weight;
                    m_geometry_fwd_duration_list[forward_begin + 1 +
                                                 leaf_object.fwd_segment_position] =
                        new_segment_duration;
                    m_geometry_datasource[forward_begin + 1 + leaf_object.fwd_segment_position] =
                        forward_speed_iter->speed_source.source;

                    // count statistics for logging
                    counters[forward_speed_iter->speed_source.source] += 1;
                }
                else
                {
                    // count statistics for logging
                    counters[LUA_SOURCE] += 1;
                }

                const auto current_rev_duration =
                    m_geometry_rev_duration_list[forward_begin + leaf_object.fwd_segment_position];

                const auto reverse_speed_iter = find(
                    segment_speed_lookup, SegmentSpeedSource{{v->node_id, u->node_id}, {0, 0, 0}});

                if (reverse_speed_iter != segment_speed_lookup.end())
                {
                    EdgeWeight new_segment_weight, new_segment_duration;
                    getNewWeight(reverse_speed_iter,
                                 segment_length,
                                 segment_speed_filenames,
                                 current_rev_duration,
                                 log_edge_updates_factor,
                                 new_segment_weight,
                                 new_segment_duration);

                    m_geometry_rev_weight_list[forward_begin + leaf_object.fwd_segment_position] =
                        new_segment_weight;
                    m_geometry_rev_duration_list[forward_begin + leaf_object.fwd_segment_position] =
                        new_segment_duration;
                    m_geometry_datasource[forward_begin + leaf_object.fwd_segment_position] =
                        reverse_speed_iter->speed_source.source;

                    // count statistics for logging
                    counters[reverse_speed_iter->speed_source.source] += 1;
                }
                else
                {
                    counters[LUA_SOURCE] += 1;
                }
            }
        }); // parallel_for_each

        counters_type merged_counters(num_counters, 0);
        for (const auto &counters : segment_speeds_counters)
        {
            for (std::size_t i = 0; i < counters.size(); i++)
            {
                merged_counters[i] += counters[i];
            }
        }

        for (std::size_t i = 0; i < merged_counters.size(); i++)
        {
            if (i == LUA_SOURCE)
            {
                util::Log() << "Used " << merged_counters[LUA_SOURCE]
                            << " speeds from LUA profile or input map";
            }
            else
            {
                // segments_speeds_counters has 0 as LUA, segment_speed_filenames not, thus we need
                // to susbstract 1 to avoid off-by-one error
                util::Log() << "Used " << merged_counters[i] << " speeds from "
                            << segment_speed_filenames[i - 1];
            }
        }
    }

    const auto maybe_save_geometries = [&] {
        if (!(update_edge_weights || update_turn_penalties))
            return;

        // Now save out the updated compressed geometries
        std::ofstream geometry_stream(geometry_filename, std::ios::binary);
        if (!geometry_stream)
        {
            const std::string message{"Failed to open " + geometry_filename + " for writing"};
            throw util::exception(message + SOURCE_REF);
        }
        const unsigned number_of_indices = m_geometry_indices.size();
        const unsigned number_of_compressed_geometries = m_geometry_node_list.size();
        geometry_stream.write(reinterpret_cast<const char *>(&number_of_indices), sizeof(unsigned));
        geometry_stream.write(reinterpret_cast<char *>(&(m_geometry_indices[0])),
                              number_of_indices * sizeof(unsigned));
        geometry_stream.write(reinterpret_cast<const char *>(&number_of_compressed_geometries),
                              sizeof(unsigned));
        geometry_stream.write(reinterpret_cast<char *>(&(m_geometry_node_list[0])),
                              number_of_compressed_geometries * sizeof(NodeID));
        geometry_stream.write(reinterpret_cast<char *>(&(m_geometry_fwd_weight_list[0])),
                              number_of_compressed_geometries * sizeof(EdgeWeight));
        geometry_stream.write(reinterpret_cast<char *>(&(m_geometry_rev_weight_list[0])),
                              number_of_compressed_geometries * sizeof(EdgeWeight));
        geometry_stream.write(reinterpret_cast<char *>(&(m_geometry_fwd_duration_list[0])),
                              number_of_compressed_geometries * sizeof(EdgeWeight));
        geometry_stream.write(reinterpret_cast<char *>(&(m_geometry_rev_duration_list[0])),
                              number_of_compressed_geometries * sizeof(EdgeWeight));
    };

    const auto save_datasource_indexes = [&] {
        std::ofstream datasource_stream(datasource_indexes_filename, std::ios::binary);
        if (!datasource_stream)
        {
            const std::string message{"Failed to open " + datasource_indexes_filename +
                                      " for writing"};
            throw util::exception(message + SOURCE_REF);
        }
        std::uint64_t number_of_datasource_entries = m_geometry_datasource.size();
        datasource_stream.write(reinterpret_cast<const char *>(&number_of_datasource_entries),
                                sizeof(number_of_datasource_entries));
        if (number_of_datasource_entries > 0)
        {
            datasource_stream.write(reinterpret_cast<char *>(&(m_geometry_datasource[0])),
                                    number_of_datasource_entries * sizeof(uint8_t));
        }
    };

    const auto save_datastore_names = [&] {
        std::ofstream datasource_stream(datasource_names_filename, std::ios::binary);
        if (!datasource_stream)
        {
            const std::string message{"Failed to open " + datasource_names_filename +
                                      " for writing"};
            throw util::exception(message + SOURCE_REF);
        }
        datasource_stream << "lua profile" << std::endl;
        for (auto const &name : segment_speed_filenames)
        {
            // Only write the filename, without path or extension.
            // This prevents information leakage, and keeps names short
            // for rendering in the debug tiles.
            const boost::filesystem::path p(name);
            datasource_stream << p.stem().string() << std::endl;
        }
    };

    tbb::parallel_invoke(maybe_save_geometries, save_datasource_indexes, save_datastore_names);

    auto turn_weight_penalty_ptr = reinterpret_cast<TurnPenalty *>(
        reinterpret_cast<char *>(turn_weight_penalties_region.get_address()) +
        sizeof(extractor::lookup::TurnPenaltiesHeader));
    // TODO MKR auto turn_duration_penalty_ptr = reinterpret_cast<TurnPenalty *>(
    //     reinterpret_cast<char *>(turn_duration_penalties_region.get_address()) +
    //     sizeof(extractor::lookup:TurnPenaltiesHeader));
    auto turn_index_block_ptr = reinterpret_cast<const extractor::lookup::TurnIndexBlock *>(
        turn_penalties_index_region.get_address());
    auto edge_segment_byte_ptr = reinterpret_cast<const char *>(edge_segment_region.get_address());
    auto edge_based_edge_ptr = reinterpret_cast<extractor::EdgeBasedEdge *>(
        reinterpret_cast<char *>(edge_based_graph_region.get_address()) +
        sizeof(EdgeBasedGraphHeader));

    const auto edge_based_edge_last = reinterpret_cast<extractor::EdgeBasedEdge *>(
        reinterpret_cast<char *>(edge_based_graph_region.get_address()) +
        sizeof(EdgeBasedGraphHeader) +
        sizeof(extractor::EdgeBasedEdge) * graph_header.number_of_edges);

    while (edge_based_edge_ptr != edge_based_edge_last)
    {
        // Make a copy of the data from the memory map
        extractor::EdgeBasedEdge inbuffer = *edge_based_edge_ptr;
        edge_based_edge_ptr++;

        if (update_edge_weights || update_turn_penalties)
        {
            bool skip_this_edge = false;
            auto header = reinterpret_cast<const extractor::lookup::SegmentHeaderBlock *>(
                edge_segment_byte_ptr);
            edge_segment_byte_ptr += sizeof(extractor::lookup::SegmentHeaderBlock);

            auto previous_osm_node_id = header->previous_osm_node_id;
            EdgeWeight new_weight = 0;
            int compressed_edge_nodes = static_cast<int>(header->num_osm_nodes);

            auto segmentblocks =
                reinterpret_cast<const extractor::lookup::SegmentBlock *>(edge_segment_byte_ptr);
            edge_segment_byte_ptr +=
                sizeof(extractor::lookup::SegmentBlock) * (header->num_osm_nodes - 1);

            const auto num_segments = header->num_osm_nodes - 1;
            for (auto i : util::irange<std::size_t>(0, num_segments))
            {
                auto speed_iter =
                    find(segment_speed_lookup,
                         SegmentSpeedSource{
                             previous_osm_node_id, segmentblocks[i].this_osm_node_id, {0, 0, 0}});
                if (speed_iter != segment_speed_lookup.end())
                {
                    if (speed_iter->speed_source.speed > 0)
                    {
                        const auto new_segment_weight = distanceAndSpeedToWeight(
                            segmentblocks[i].segment_length, speed_iter->speed_source.speed);
                        new_weight += new_segment_weight;
                    }
                    else
                    {
                        // If we hit a 0-speed edge, then it's effectively not traversible.
                        // We don't want to include it in the edge_based_edge_list, so
                        // we set a flag and `continue` the parent loop as soon as we can.
                        // This would be a perfect place to use `goto`, but Patrick vetoed it.
                        skip_this_edge = true;
                        break;
                    }
                }
                else
                {
                    // If no lookup found, use the original weight value for this segment
                    new_weight += segmentblocks[i].segment_weight;
                }

                previous_osm_node_id = segmentblocks[i].this_osm_node_id;
            }

            // Update the node-weight cache.  This is the weight of the edge-based-node only,
            // it doesn't include the turn.  We may visit the same node multiple times, but
            // we should always assign the same value here.
            node_weights[inbuffer.source] = new_weight;

            // We found a zero-speed edge, so we'll skip this whole edge-based-edge which
            // effectively removes it from the routing network.
            if (skip_this_edge)
            {
                turn_index_block_ptr++;
                turn_weight_penalty_ptr++;
                // TODO MKR turn_duration_penalty_ptr++;
                continue;
            }

            auto turn_weight_penalty = *turn_weight_penalty_ptr;
            // TODO MKR auto turn_duration_penalty = *turn_duration_penalty_ptr;

            const auto turn_iter = find(turn_penalty_lookup,
                                        TurnPenaltySource{{turn_index_block_ptr->from_id,
                                                           turn_index_block_ptr->via_id,
                                                           turn_index_block_ptr->to_id},
                                                          {0, 0}});
            if (turn_iter != turn_penalty_lookup.end())
            {
                auto turn_weight_penalty_100ms = turn_iter->penalty_source.penalty * 10;
                if (turn_weight_penalty_100ms + new_weight < compressed_edge_nodes)
                {
                    util::Log(logWARNING) << "turn penalty " << turn_iter->penalty_source.penalty
                                          << " for turn " << turn_index_block_ptr->from_id << ", "
                                          << turn_index_block_ptr->via_id << ", "
                                          << turn_index_block_ptr->to_id
                                          << " is too negative: clamping turn weight to "
                                          << (compressed_edge_nodes - new_weight);

                    turn_weight_penalty =
                        boost::numeric_cast<TurnPenalty>(compressed_edge_nodes - new_weight);
                }
                else
                {
                    turn_weight_penalty =
                        boost::numeric_cast<TurnPenalty>(turn_weight_penalty_100ms);
                }
            }

            inbuffer.weight = turn_weight_penalty + new_weight;
            // FIXME update the turn penalty
            *turn_weight_penalty_ptr = turn_weight_penalty;

            // Increment pointers
            turn_index_block_ptr++;
            turn_weight_penalty_ptr++;
            // TODO MKR turn_duration_penalty_ptr++;
        }

        edge_based_edge_list.emplace_back(std::move(inbuffer));
    }

    util::Log() << "Done reading edges";
    return graph_header.max_edge_id;
}

void Contractor::ReadNodeLevels(std::vector<float> &node_levels) const
{
    storage::io::FileReader order_file(config.level_output_path,
                                       storage::io::FileReader::HasNoFingerprint);

    const auto level_size = order_file.ReadElementCount32();
    node_levels.resize(level_size);
    order_file.ReadInto(node_levels);
}

void Contractor::WriteNodeLevels(std::vector<float> &&in_node_levels) const
{
    std::vector<float> node_levels(std::move(in_node_levels));

    boost::filesystem::ofstream order_output_stream(config.level_output_path, std::ios::binary);

    unsigned level_size = node_levels.size();
    order_output_stream.write((char *)&level_size, sizeof(unsigned));
    order_output_stream.write((char *)node_levels.data(), sizeof(float) * node_levels.size());
}

void Contractor::WriteCoreNodeMarker(std::vector<bool> &&in_is_core_node) const
{
    std::vector<bool> is_core_node(std::move(in_is_core_node));
    std::vector<char> unpacked_bool_flags(std::move(is_core_node.size()));
    for (auto i = 0u; i < is_core_node.size(); ++i)
    {
        unpacked_bool_flags[i] = is_core_node[i] ? 1 : 0;
    }

    boost::filesystem::ofstream core_marker_output_stream(config.core_output_path,
                                                          std::ios::binary);
    unsigned size = unpacked_bool_flags.size();
    core_marker_output_stream.write((char *)&size, sizeof(unsigned));
    core_marker_output_stream.write((char *)unpacked_bool_flags.data(),
                                    sizeof(char) * unpacked_bool_flags.size());
}

std::size_t
Contractor::WriteContractedGraph(unsigned max_node_id,
                                 const util::DeallocatingVector<QueryEdge> &contracted_edge_list)
{
    // Sorting contracted edges in a way that the static query graph can read some in in-place.
    tbb::parallel_sort(contracted_edge_list.begin(), contracted_edge_list.end());
    const std::uint64_t contracted_edge_count = contracted_edge_list.size();
    util::Log() << "Serializing compacted graph of " << contracted_edge_count << " edges";

    const util::FingerPrint fingerprint = util::FingerPrint::GetValid();
    boost::filesystem::ofstream hsgr_output_stream(config.graph_output_path, std::ios::binary);
    hsgr_output_stream.write((char *)&fingerprint, sizeof(util::FingerPrint));
    const NodeID max_used_node_id = [&contracted_edge_list] {
        NodeID tmp_max = 0;
        for (const QueryEdge &edge : contracted_edge_list)
        {
            BOOST_ASSERT(SPECIAL_NODEID != edge.source);
            BOOST_ASSERT(SPECIAL_NODEID != edge.target);
            tmp_max = std::max(tmp_max, edge.source);
            tmp_max = std::max(tmp_max, edge.target);
        }
        return tmp_max;
    }();

    util::Log(logDEBUG) << "input graph has " << (max_node_id + 1) << " nodes";
    util::Log(logDEBUG) << "contracted graph has " << (max_used_node_id + 1) << " nodes";

    std::vector<util::StaticGraph<EdgeData>::NodeArrayEntry> node_array;
    // make sure we have at least one sentinel
    node_array.resize(max_node_id + 2);

    util::Log() << "Building node array";
    util::StaticGraph<EdgeData>::EdgeIterator edge = 0;
    util::StaticGraph<EdgeData>::EdgeIterator position = 0;
    util::StaticGraph<EdgeData>::EdgeIterator last_edge;

    // initializing 'first_edge'-field of nodes:
    for (const auto node : util::irange(0u, max_used_node_id + 1))
    {
        last_edge = edge;
        while ((edge < contracted_edge_count) && (contracted_edge_list[edge].source == node))
        {
            ++edge;
        }
        node_array[node].first_edge = position; //=edge
        position += edge - last_edge;           // remove
    }

    for (const auto sentinel_counter :
         util::irange<unsigned>(max_used_node_id + 1, node_array.size()))
    {
        // sentinel element, guarded against underflow
        node_array[sentinel_counter].first_edge = contracted_edge_count;
    }

    util::Log() << "Serializing node array";

    RangebasedCRC32 crc32_calculator;
    const unsigned edges_crc32 = crc32_calculator(contracted_edge_list);
    util::Log() << "Writing CRC32: " << edges_crc32;

    const std::uint64_t node_array_size = node_array.size();
    // serialize crc32, aka checksum
    hsgr_output_stream.write((char *)&edges_crc32, sizeof(unsigned));
    // serialize number of nodes
    hsgr_output_stream.write((char *)&node_array_size, sizeof(std::uint64_t));
    // serialize number of edges
    hsgr_output_stream.write((char *)&contracted_edge_count, sizeof(std::uint64_t));
    // serialize all nodes
    if (node_array_size > 0)
    {
        hsgr_output_stream.write((char *)&node_array[0],
                                 sizeof(util::StaticGraph<EdgeData>::NodeArrayEntry) *
                                     node_array_size);
    }

    // serialize all edges
    util::Log() << "Building edge array";
    std::size_t number_of_used_edges = 0;

    util::StaticGraph<EdgeData>::EdgeArrayEntry current_edge;
    for (const auto edge : util::irange<std::size_t>(0UL, contracted_edge_list.size()))
    {
        // some self-loops are required for oneway handling. Need to assertthat we only keep these
        // (TODO)
        // no eigen loops
        // BOOST_ASSERT(contracted_edge_list[edge].source != contracted_edge_list[edge].target ||
        // node_represents_oneway[contracted_edge_list[edge].source]);
        current_edge.target = contracted_edge_list[edge].target;
        current_edge.data = contracted_edge_list[edge].data;

        // every target needs to be valid
        BOOST_ASSERT(current_edge.target <= max_used_node_id);
#ifndef NDEBUG
        if (current_edge.data.weight <= 0)
        {
            util::Log(logWARNING) << "Edge: " << edge
                                  << ",source: " << contracted_edge_list[edge].source
                                  << ", target: " << contracted_edge_list[edge].target
                                  << ", weight: " << current_edge.data.weight;

            util::Log(logWARNING) << "Failed at adjacency list of node "
                                  << contracted_edge_list[edge].source << "/"
                                  << node_array.size() - 1;
            throw util::exception("Edge weight is <= 0" + SOURCE_REF);
        }
#endif
        hsgr_output_stream.write((char *)&current_edge,
                                 sizeof(util::StaticGraph<EdgeData>::EdgeArrayEntry));

        ++number_of_used_edges;
    }

    return number_of_used_edges;
}

/**
 \brief Build contracted graph.
 */
void Contractor::ContractGraph(
    const EdgeID max_edge_id,
    util::DeallocatingVector<extractor::EdgeBasedEdge> &edge_based_edge_list,
    util::DeallocatingVector<QueryEdge> &contracted_edge_list,
    std::vector<EdgeWeight> &&node_weights,
    std::vector<bool> &is_core_node,
    std::vector<float> &inout_node_levels) const
{
    std::vector<float> node_levels;
    node_levels.swap(inout_node_levels);

    GraphContractor graph_contractor(
        max_edge_id + 1, edge_based_edge_list, std::move(node_levels), std::move(node_weights));
    graph_contractor.Run(config.core_factor);
    graph_contractor.GetEdges(contracted_edge_list);
    graph_contractor.GetCoreMarker(is_core_node);
    graph_contractor.GetNodeLevels(inout_node_levels);
}
}
}
