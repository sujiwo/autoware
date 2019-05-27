#include "Trajectory.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <utility>


class TrajectoryLoop : public Trajectory
{
public:
	void push_back(const PoseStamped &);

protected:
	pcl::KdTreeFLANN<pcl::PointXYZ> positionLookups;
	std::pair<uint64_t, uint64_t> loops;
};
