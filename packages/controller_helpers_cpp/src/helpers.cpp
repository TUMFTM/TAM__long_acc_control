#include "controller_helpers_cpp/helpers.hpp"
namespace tam::helpers::control
{
tam::types::control::ControlConstraintPoint find_constraint_point(
  const tam::types::control::ControlConstraints & constraints, const float idx)
{
  tam::types::control::ControlConstraintPoint point =
    tam::helpers::numerical::interp_from_idx(constraints.points, idx);

  return point;
}
tam::types::control::TrajectoryPoint find_trajectory_point(
  const tam::types::control::Trajectory & trajectory, const float idx)
{
  tam::types::control::TrajectoryPoint point =
    tam::helpers::numerical::interp_from_idx(trajectory.points, idx);

  return point;
}
tam::types::control::AdditionalInfoPoint find_additional_info_point(
  const tam::types::control::AdditionalTrajectoryInfos & additional_info, const float idx)
{
  tam::types::control::AdditionalInfoPoint point =
    tam::helpers::numerical::interp_from_idx(additional_info.points, idx);

  return point;
}
double find_heading(const tam::types::control::Trajectory & trajectory, const float idx)
{
  std::vector<double> heading;
  std::transform(
    trajectory.points.begin(), trajectory.points.end(), std::back_inserter(heading),
    [](tam::types::control::TrajectoryPoint pt) { return pt.orientation_rad.z; });
  return tam::helpers::numerical::interp_from_idx(heading, idx);
}
}  // namespace tam::helpers::control
