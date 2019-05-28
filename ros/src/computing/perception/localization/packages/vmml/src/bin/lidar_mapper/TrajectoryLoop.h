#include "Trajectory.h"

#include <utility>

#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>


struct PointXYZid
{
	PCL_ADD_POINT4D;
	uint32_t idx;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZid,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(uint32_t, idx, idx)
);


class TrajectoryLoop : public Trajectory
{
public:
	void push_back(const PoseStamped &);

protected:
	pcl::KdTreeFLANN<PointXYZid> positionLookups;
	std::pair<uint64_t, uint64_t> loops;
};
