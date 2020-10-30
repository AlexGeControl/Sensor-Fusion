/*
 * @Description: loop closure detection using scan context
 * @Author: Ge Yao
 * @Date: 2020-10-28 15:43:03
 */
#include <limits>

#include <math.h>
#include <cmath>
#include <memory>
#include <ostream>

#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"
#include "glog/logging.h"

namespace lidar_localization {

ScanContextManager::ScanContextManager(const YAML::Node& node) {
    //
    // parse config:
    // 
    // a. ROI definition:
    MAX_RADIUS_ = node["max_radius"].as<float>();
    MAX_THETA_ = node["max_theta"].as<float>();
    // b. resolution:
    NUM_RINGS_ = node["num_rings"].as<int>();
    NUM_SECTORS_ = node["num_sectors"].as<int>();
    DEG_PER_SECTOR_ = MAX_THETA_ / NUM_SECTORS_;
    // c. ring key indexing interval:
    INDEXING_INTERVAL_ = node["indexing_interval"].as<int>();
    // d. min. key frame sequence distance:
    MIN_KEY_FRAME_SEQ_DISTANCE_ = node["min_key_frame_seq_distance"].as<int>();
    // e. num. of nearest-neighbor candidates to check:
    NUM_CANDIDATES_ = node["num_candidates"].as<int>();
    // f. sector key fast alignment search ratio:
    FAST_ALIGNMENT_SEARCH_RATIO_ = node["fast_alignment_search_ratio"].as<float>();
    // g. scan context distance threshold:
    SCAN_CONTEXT_DISTANCE_THRESH_ = node["scan_context_distance_thresh"].as<float>();

    // prompt:
    LOG(INFO) << "Scan Context params:" << std::endl
              << "\tmax. radius: " << MAX_RADIUS_ << std::endl
              << "\tmax. theta: " << MAX_THETA_ << std::endl
              << "\tnum. rings: " << NUM_RINGS_ << std::endl
              << "\tnum. sectors: " << NUM_SECTORS_  << std::endl
              << "\tre-indexing interval: " << INDEXING_INTERVAL_ << std::endl
              << "\tmin. key frame sequence distance: " << MIN_KEY_FRAME_SEQ_DISTANCE_ << std::endl
              << "\tnearest-neighbor candidates to check: " << NUM_CANDIDATES_ << std::endl
              << "\tfast alignment search ratio: " << FAST_ALIGNMENT_SEARCH_RATIO_ << std::endl
              << "\tloop-closure scan context distance thresh: " << SCAN_CONTEXT_DISTANCE_THRESH_ << std::endl
              << std::endl;
    
    // reset state:
    state_.scan_context_.clear();
    state_.ring_key_.clear();

    state_.indexing_counter_ = 0;

    state_.ring_key_index_.reset();
    state_.ring_key_data_.clear();
}

void ScanContextManager::Update(const CloudData &scan) {
    // extract scan context and corresponding ring key:
    ScanContext scan_context = GetScanContext(scan);
    RingKey ring_key = GetRingKey(scan_context);

    // update buffer:
    state_.scan_context_.push_back(scan_context);
    state_.ring_key_.push_back(ring_key);
}

/**
 * @brief  detect loop closure for the latest key scan
 * @param  void
 * @return void
 */
void ScanContextManager::DetectLoopClosure(void) {
    // use latest key scan for query:
    const ScanContext &latest_scan_context = state_.scan_context_.back();
    const RingKey &latest_ring_key = state_.ring_key_.back();

    GetLoopClosureMatch(latest_scan_context, latest_ring_key);
}

/**
 * @brief  get scan context of given lidar scan
 * @param  scan, lidar scan of key frame
 * @return scan context as Eigen::MatrixXd
 */
ScanContextManager::ScanContext ScanContextManager::GetScanContext(const CloudData &scan) {
    // num. of point measurements in current scan:
    const size_t N = scan.cloud_ptr->points.size();
    
    // init scan context:
    const float UNKNOWN_HEIGHT = -1000.0f;
    ScanContext scan_context = UNKNOWN_HEIGHT * ScanContext::Ones(NUM_RINGS_, NUM_SECTORS_);

    // iterate through point measurements and create scan context:
    float x, y, z;
    float radius, theta;
    for (size_t i = 0; i < N; ++i) {
        // parse point measurement:
        x = scan.cloud_ptr->points.at(i).x;
        y = scan.cloud_ptr->points.at(i).y;
        z = scan.cloud_ptr->points.at(i).z + 2.0f;

        radius = hypot(x, y);
        theta = GetOrientation(x, y);

        // ROI check:
        if (radius > MAX_RADIUS_) {
            continue;
        }
        
        // get ring-sector index:
        int rid = GetIndex(radius, MAX_RADIUS_, NUM_RINGS_); 
        int sid = GetIndex(theta, MAX_THETA_, NUM_SECTORS_); 

        // update bin height:
        if (scan_context(rid, sid) < z) {
            scan_context(rid, sid) = z;
        }
    }

    // reset unknown height to 0.0 for later cosine distance calculation:
    for (int rid = 0; rid < scan_context.rows(); ++rid) {
        for (int sid = 0; sid < scan_context.cols(); ++sid) {
            if (UNKNOWN_HEIGHT == scan_context(rid, sid)) {
                scan_context(rid, sid) = 0.0;
            }
        }
    }

    return scan_context;
}

/**
 * @brief  get ring key of given scan context
 * @param  scan_context, scan context of key scan
 * @return ring key as RingKey
 */
ScanContextManager::RingKey ScanContextManager::GetRingKey(
    const ScanContext &scan_context
) {
    RingKey ring_key(scan_context.rows());

    for (int rid = 0; rid < scan_context.rows(); ++rid) {
        ring_key.at(rid) = scan_context.row(rid).mean();
    }

    return ring_key;
}

/**
 * @brief  generate random ring keys for indexing test
 * @return void
 */
void ScanContextManager::GenerateRandomRingKey(
    ScanContextManager::RingKeys &samples, 
    const int N, const int D, const float max_range
) {
	samples.clear();
    samples.resize(N);
	for (int i = 0; i < N; ++i)
	{
		samples.at(i).resize(D);
		for (int d = 0; d < D; ++d)
			samples.at(i).at(d) = max_range * (rand() % 1000) / (1000.0);
	}
}

/**
 * @brief  get sector key of given scan context
 * @param  scan_context, scan context of key scan
 * @return sector key as RingKey
 */
Eigen::MatrixXf ScanContextManager::GetSectorKey(
    const Eigen::MatrixXf &scan_context
) {
    Eigen::MatrixXf sector_key(1, scan_context.cols());

    for (int sid = 0; sid < scan_context.cols(); ++sid)
    {
        sector_key(0, sid) = scan_context.col(sid).mean();
    }

    return sector_key;
}

/**
 * @brief  get orientation of point measurement 
 * @param  x, x component of point measurement
 * @param  y, y component of point measurement
 * @return point measurement orientation, [0.0f, 360.0f)
 */
float ScanContextManager::GetOrientation(
    const float &x, 
    const float &y
) {
    float theta = 180.0f / M_PI * atan2(y, x);

    // make sure the orientation is consistent with scan context convension:
    if (theta < 0.0f) {
        theta += 360.0f;
    }

    return theta;
}

/**
 * @brief  convert floating point value to integer index 
 * @param  value, target floating point value 
 * @param  MAX_VALUE, max. floating point value
 * @param  RESOLUTION, resolution
 * @return integer index, {0, ..., RESOLUTION - 1}
 */
int ScanContextManager::GetIndex(
    const float &value, 
    const float &MAX_VALUE, 
    const int RESOLUTION
) {
    int index = std::floor(static_cast<int>(RESOLUTION*value/MAX_VALUE));

    // this ensures value at MAX_VALUE will be cast into last bin:
    index = std::min(index, RESOLUTION - 1);

    return index;
} 

/**
 * @brief  get candidate scan context indices 
 * @param  ring_key, query ring key 
 * @param  N, num. of nearest neighbor candidates
 * @param  indices, candidate indices
 * @param  distances, candidate distances
 * @return void
 */
void ScanContextManager::GetCandidateIndices(
    const RingKey &ring_key, const int N,
    std::vector<size_t> &indices,
	std::vector<float> &distances
) {    
    state_.ring_key_index_->query(
        &ring_key.at(0),
        N,
        &indices.at(0),
        &distances.at(0)
    );
}

/**
 * @brief  circular shift mat to right by shift
 * @param  mat, original matrix 
 * @param  shift, right shift amount  
 * @return shifted matrix
 */
Eigen::MatrixXf ScanContextManager::CircularShift(
    const Eigen::MatrixXf &mat, 
    int shift 
) {
    if(0 == shift)
    {
        Eigen::MatrixXf shifted_mat(mat);
        return shifted_mat; // Early return 
    }

    Eigen::MatrixXf shifted_mat = Eigen::MatrixXf::Zero( 
        mat.rows(), 
        mat.cols() 
    );
    for (int i = 0; i < mat.cols(); ++i) {
        int shifted_i = (i + shift) % mat.cols();
        shifted_mat.col(shifted_i) = mat.col(i);
    }

    return shifted_mat;
}

/**
 * @brief  get optimal shift estimation using sector key 
 * @param  target, target sector key 
 * @param  source, source sector key  
 * @return void
 */
int ScanContextManager::GetOptimalShiftUsingSectorKey(
    const Eigen::MatrixXf &target, 
    const Eigen::MatrixXf &source
) {
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();

    for (int curr_shift = 0; curr_shift < target.cols(); ++curr_shift) {
        Eigen::MatrixXf curr_target = CircularShift(
            target, curr_shift
        );

        float curr_dist = (curr_target - source).norm();

        if(curr_dist < optimal_dist)
        {
            optimal_dist = curr_dist;
            optimal_shift = curr_shift;
        }
    }

    return optimal_shift;
} 

/**
 * @brief  compute cosine distance between target and source scan context 
 * @param  target_scan_context, target scan context 
 * @param  source_scan_context, source scan context 
 * @return scan context cosine distance
 */
float ScanContextManager::GetCosineDistance(
    const ScanContext &target_scan_context, 
    const ScanContext &source_scan_context 
) {
    const int N = target_scan_context.cols();

    int num_effective_cols = 0;
    float sum_sector_similarity = 0.0f;
    for (int sid = 0; sid < N; ++sid)
    {
        const Eigen::VectorXf target_sector = target_scan_context.col(sid);
        const Eigen::VectorXf source_sector = source_scan_context.col(sid);
        
        float target_sector_norm = target_sector.norm();
        float source_sector_norm = source_sector.norm();

        if( 0.0f == target_sector_norm || 0.0f == source_sector_norm ) {
            continue;
        }
             
        float sector_similarity = target_sector.dot(source_sector) / (target_sector_norm * source_sector_norm);

        sum_sector_similarity += sector_similarity;
        ++num_effective_cols;
    }
    
    return (0 == num_effective_cols ? 1.0f : (1.0f - sum_sector_similarity / num_effective_cols));
}

/**
 * @brief  get scan context match result between target and source scan context 
 * @param  target_scan_context, target scan context 
 * @param  source_scan_context, source scan context 
 * @return scan context match result as std::pair<int, float>
 */
std::pair<int, float> ScanContextManager::GetScanContextMatch(
    const ScanContext &target_scan_context, 
    const ScanContext &source_scan_context
) {
    // first perform fast alignment using sector key:
    Eigen::MatrixXf target_sector_key = GetSectorKey(target_scan_context);
    Eigen::MatrixXf source_sector_key = GetSectorKey(source_scan_context);
    int sector_key_shift = GetOptimalShiftUsingSectorKey(
        target_sector_key, 
        source_sector_key 
    );

    // generate precise alignment proposals:
    const int N = target_scan_context.cols();
    const int SEARCH_RADIUS = round(
        0.5 * FAST_ALIGNMENT_SEARCH_RATIO_ * N
    );
    std::vector<int> candidate_shifts{ sector_key_shift };
    for (int r = 1; r < SEARCH_RADIUS + 1; ++r)
    {
        candidate_shifts.push_back(
            (sector_key_shift + r + N) % N 
        );
        candidate_shifts.push_back( 
            (sector_key_shift - r + N) % N 
        );
    }
    std::sort(candidate_shifts.begin(), candidate_shifts.end());

    // then continue to precise alignment using scan context cosine distance:
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();
    for (int curr_shift: candidate_shifts)
    {
        ScanContext target_scan_context_shifted = CircularShift(
            target_scan_context, 
            curr_shift
        );

        float curr_dist = GetCosineDistance(
            target_scan_context_shifted, 
            source_scan_context 
        );

        if(curr_dist < optimal_dist)
        {   
            optimal_shift = curr_shift;
            optimal_dist = curr_dist;
        }
    }

    return std::make_pair(optimal_shift, optimal_dist);
}

/**
 * @brief  get loop closure match result for given scan context and ring key 
 * @param  query_scan_context, query scan context 
 * @param  query_ring_key, query ring key
 * @return loop closure match result as std::pair<int, float>
 */
std::pair<int, float> ScanContextManager::GetLoopClosureMatch(
    const ScanContext &query_scan_context,
    const RingKey &query_ring_key
) {
    int match_id = -1;

    //
    // step 0: loop closure detection criteria check -- only perform loop closure detection when
    //   a. current key scan is temporally far away from previous key scan:
    // 
    if(
        state_.ring_key_.size() <= (static_cast<size_t>(MIN_KEY_FRAME_SEQ_DISTANCE_))
    ) {
        std::pair<int, float> result {match_id, 0.0};
        return result; 
    }

    //
    // step 1: update ring key index
    // 
    ++state_.indexing_counter_;
    if (
        0 == (state_.indexing_counter_ % INDEXING_INTERVAL_)
    ) {
        // fetch to-be-indexed ring keys from buffer:
        state_.ring_key_data_.clear();
        state_.ring_key_data_.insert(
            state_.ring_key_data_.end(),
            state_.ring_key_.begin(), 
            // this ensures the min. key frame seq. distance
            state_.ring_key_.end() - MIN_KEY_FRAME_SEQ_DISTANCE_
        );

        // TODO: enable in-place update of KDTree
        state_.ring_key_index_.reset(); 
        state_.ring_key_index_ = std::make_shared<RingKeyIndex>(
            NUM_RINGS_,  /* dim */
            state_.ring_key_data_,
            10           /* max leaf size */
        );
    }

    //
    // step 2: perform kNN search
    // 
    std::vector<size_t> candidate_indices(NUM_CANDIDATES_);
	std::vector<float> candidate_distances(NUM_CANDIDATES_);
    GetCandidateIndices(
        query_ring_key, NUM_CANDIDATES_,
        candidate_indices, candidate_distances
    );

    // 
    // step 3: find optimal match
    // 
    int optimal_index = 0;
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < NUM_CANDIDATES_; ++i)
    {   
        const ScanContext &candidate_scan_context = state_.scan_context_.at(
            candidate_indices.at(i)
        );

        std::pair<int, float> match_result = GetScanContextMatch(
            candidate_scan_context, query_scan_context
        ); 
        
        int candidate_shift = match_result.first;
        float candidate_dist = match_result.second;
        
        if(candidate_dist < optimal_dist)
        {
            optimal_dist = candidate_dist;
            optimal_shift = candidate_shift;
            optimal_index = candidate_indices.at(i);
        }
    }

    // 
    // step 4: loop closure threshold check:
    //
    float yaw_change_in_deg = optimal_shift * DEG_PER_SECTOR_;
    float yaw_change_in_rad = yaw_change_in_deg / 180.0f * M_PI;
    if(optimal_dist < SCAN_CONTEXT_DISTANCE_THRESH_)
    {
        match_id = optimal_index; 

        LOG(INFO) << std::endl
                  << "[Scan Context] Loop-Closure Detected " 
                  << state_.scan_context_.size() - 1 << "<-->" << optimal_index << std::endl 
                  << "\tDistance " << optimal_dist << std::endl 
                  << "\tHeading Change " << yaw_change_in_deg << " deg." << std::endl;
    }

    std::pair<int, float> result{match_id, yaw_change_in_rad};

    return result;
}

} // namespace lidar_localization