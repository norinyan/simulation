#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

AStar::~AStar() = default;

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    if ((pool_size.array() < 3).any()) throw std::invalid_argument("SCAN_INVALID_ASTAR_POOL");
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;
    nodes_.clear();
    grid_map_ = std::move(occ_map);
}

void AStar::setReference(const std::vector<Vector3d> &reference)
{
    reference_.clear();
    reference_lengths_.clear();
    for (const auto &point : reference)
    {
        if (!point.allFinite()) throw std::invalid_argument("SCAN_INVALID_ASTAR_REFERENCE");
        if (!reference_.empty() && (point-reference_.back()).norm() < 1e-8) continue;
        reference_lengths_.push_back(reference_.empty() ? 0.0 :
            reference_lengths_.back()+(point-reference_.back()).norm());
        reference_.push_back(point);
    }
    if (reference_.size() < 2) {reference_.clear(); reference_lengths_.clear();}
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    Eigen::Vector3d start_to_end = end_pt - start_pt;
    if (start_to_end.norm() < 1e-6)
        return false;
    const double path_yaw = std::atan2(start_to_end(1), start_to_end(0));
    start_to_end.normalize();

    int occ = checkOccupancy(Index2Coord(start_idx), path_yaw);
    if (occ)
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            if (grid_map_->expired()) return false;
            start_pt -= start_to_end * step_size_;
            if (!Coord2Index(start_pt, start_idx))
                return false;

            occ = checkOccupancy(Index2Coord(start_idx), path_yaw);
            if (occ == -1)
            {
                std::fprintf(stderr, "SCAN: [Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
    if (occ)
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            if (grid_map_->expired()) return false;
            end_pt += start_to_end * step_size_;
            if (!Coord2Index(end_pt, end_idx))
                return false;

            occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
            if (occ == -1)
            {
                std::fprintf(stderr, "SCAN: [Astar] End point outside the map region.");
                return false;
            }
        } while (occ);
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    const auto time_1 = std::chrono::steady_clock::now();
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator>().swap(openSet_);
    gridPath_.clear();
    nodes_.clear();
    if (grid_map_->expired()) return ASTAR_RET::SEARCH_ERR;
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    // A 3-D straight search plane cuts through steps and slopes across a landing.
    // Keep the upstream XY neighbors, but use the relevant part of the supplied
    // terrain-following reference to determine their height.
    double min_z = std::numeric_limits<double>::infinity(), max_z = -min_z;
    for (const auto &point : reference_) {min_z = std::min(min_z, point.z()); max_z = std::max(max_z, point.z());}
    const bool terrain_reference = reference_.size() > 1 && max_z-min_z > 1e-6;
    std::vector<Vector3d> search_reference;
    auto referenceAt = [&](double progress) -> Vector3d {
        auto end = std::lower_bound(reference_lengths_.begin()+1, reference_lengths_.end(), progress);
        const std::size_t i = std::min<std::size_t>(end-reference_lengths_.begin(), reference_.size()-1);
        const double ratio = std::clamp((progress-reference_lengths_[i-1]) /
            (reference_lengths_[i]-reference_lengths_[i-1]), 0.0, 1.0);
        return reference_[i-1]+ratio*(reference_[i]-reference_[i-1]);
    };
    auto closestProgress = [&](const Vector3d &point) {
        double best = std::numeric_limits<double>::infinity(), progress = 0;
        for (std::size_t i=1; i<reference_.size(); ++i)
        {
            const Vector3d delta = reference_[i]-reference_[i-1];
            const double ratio = std::clamp((point-reference_[i-1]).dot(delta)/delta.squaredNorm(), 0.0, 1.0);
            const double distance = (point-reference_[i-1]-ratio*delta).squaredNorm();
            if (distance < best)
            {
                best = distance;
                progress = reference_lengths_[i-1]+ratio*(reference_lengths_[i]-reference_lengths_[i-1]);
            }
        }
        return progress;
    };
    auto heightOnReference = [&](const Vector2d &xy) {
        double best = std::numeric_limits<double>::infinity(), z = search_reference.front().z();
        for (std::size_t i=1; i<search_reference.size(); ++i)
        {
            const Vector2d delta = (search_reference[i]-search_reference[i-1]).head<2>();
            const double ratio = delta.squaredNorm() > 1e-12 ?
                std::clamp((xy-search_reference[i-1].head<2>()).dot(delta)/delta.squaredNorm(), 0.0, 1.0) : 0.0;
            const double distance = (xy-search_reference[i-1].head<2>()-ratio*delta).squaredNorm();
            if (distance < best)
            {
                best = distance;
                z = search_reference[i-1].z()+ratio*(search_reference[i].z()-search_reference[i-1].z());
            }
        }
        return z;
    };
    auto referenceZIndex = [&](const Vector2d &xy) {
        return static_cast<int>((heightOnReference(xy)-center_.z())*inv_step_size_+0.5)+CENTER_IDX_.z();
    };
    Vector3i start_idx, end_idx;
    bool endpoints_ok;
    if (terrain_reference)
    {
        double begin = closestProgress(start_pt), end = closestProgress(end_pt);
        const double direction = end >= begin ? 1.0 : -1.0;
        auto headingAt = [&](double progress) {
            const Vector3d tangent = referenceAt(std::min(reference_lengths_.back(), progress+step_size_))-
                referenceAt(std::max(0.0, progress-step_size_));
            return std::atan2(direction*tangent.y(), direction*tangent.x());
        };
        auto clearEndpoint = [&](double &progress, double step, Vector3i &index) {
            while (progress >= 0.0 && progress <= reference_lengths_.back())
            {
                if (grid_map_->expired()) return false;
                const Vector3d point = referenceAt(progress);
                if (!Coord2Index(point, index)) return false;
                if (!checkOccupancy(Index2Coord(index), headingAt(progress))) return true;
                progress += step;
            }
            return false;
        };
        endpoints_ok = clearEndpoint(begin, -direction*step_size_, start_idx) &&
            clearEndpoint(end, direction*step_size_, end_idx);
        if (endpoints_ok)
        {
            const double low = std::min(begin,end), high = std::max(begin,end);
            search_reference.push_back(referenceAt(low));
            for (std::size_t i=1; i<reference_.size(); ++i)
                if (reference_lengths_[i] > low && reference_lengths_[i] < high) search_reference.push_back(reference_[i]);
            search_reference.push_back(referenceAt(high));
            start_idx.z() = referenceZIndex(Index2Coord(start_idx).head<2>());
            end_idx.z() = referenceZIndex(Index2Coord(end_idx).head<2>());
            endpoints_ok = start_idx.z() >= 0 && start_idx.z() < POOL_SIZE_.z() &&
                end_idx.z() >= 0 && end_idx.z() < POOL_SIZE_.z() &&
                !checkOccupancy(Index2Coord(start_idx), headingAt(begin)) &&
                !checkOccupancy(Index2Coord(end_idx), headingAt(end));
        }
    }
    else endpoints_ok = ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx);
    if (!endpoints_ok)
    {
        std::fprintf(stderr, "SCAN: Unable to handle the initial or end point, force return!");
        return ASTAR_RET::INIT_ERR;
    }

    const Eigen::Vector3d search_start = Index2Coord(start_idx);
    const Eigen::Vector3d search_end = Index2Coord(end_idx);
    const Eigen::Vector2d search_start_xy = search_start.head<2>();
    const Eigen::Vector2d search_xy_delta = search_end.head<2>() - search_start_xy;
    const double search_xy_len2 = search_xy_delta.squaredNorm();

    auto interpolateZIndexOnSearchPlane = [&](const int x_idx, const int y_idx) -> int {
        if (terrain_reference)
            return referenceZIndex(Index2Coord(Vector3i(x_idx,y_idx,start_idx.z())).head<2>());
        if (search_xy_len2 < 1e-8)
            return start_idx(2);

        Eigen::Vector3i sample_idx(x_idx, y_idx, start_idx(2));
        const Eigen::Vector2d sample_xy = Index2Coord(sample_idx).head<2>();
        double ratio = (sample_xy - search_start_xy).dot(search_xy_delta) / search_xy_len2;
        ratio = std::max(0.0, std::min(1.0, ratio));

        const double z = search_start(2) + ratio * (search_end(2) - search_start(2));
        return static_cast<int>((z - center_(2)) * inv_step_size_ + 0.5) + CENTER_IDX_(2);
    };

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = nodeAt(start_idx);
    GridNodePtr endPtr = nodeAt(end_idx);

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        if (grid_map_->expired()) return ASTAR_RET::SEARCH_ERR;
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // auto time_2 = std::chrono::steady_clock::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, std::chrono::duration<double>(time_2 - time_1).count()*1000);
            // if(std::chrono::duration<double>(time_2 - time_1).count() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", std::chrono::duration<double>(time_2 - time_1).count() );
            gridPath_ = retrievePath(current);
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = interpolateZIndexOnSearchPlane(neighborIdx(0), neighborIdx(1));

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }

                neighborPtr = nodeAt(neighborIdx);
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue; //in closed set.
                }

                neighborPtr->rounds = rounds_;

                const double neighbor_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
                if (checkOccupancy(Index2Coord(neighborPtr->index), neighbor_yaw))
                {
                    continue;
                }

                const int dz = neighborIdx(2) - current->index(2);
                double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored)
                {
                    //discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push(neighborPtr); //put neighbor in open set and record it.
                }
                else if (tentative_gScore < neighborPtr->gScore)
                { //in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    std::vector<GridNodePtr> active;
                    while (!openSet_.empty()) {active.push_back(openSet_.top()); openSet_.pop();}
                    for (auto *node : active) openSet_.push(node);
                }
            }
        auto time_2 = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(time_2 - time_1).count() > 0.2)
        {
            std::fprintf(stderr, "SCAN: Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    auto time_2 = std::chrono::steady_clock::now();

    if (std::chrono::duration<double>(time_2 - time_1).count() > 0.1)
        std::fprintf(stderr, "SCAN: Time consume in A star path finding is %.3fs, iter=%d", std::chrono::duration<double>(time_2 - time_1).count(), num_iter);

    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
