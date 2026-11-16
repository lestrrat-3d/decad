package booleaneval

import (
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// displacedWall is rule DW of agent-docs/carrier-record-design.md §8.7: a mesh
// Boolean's source tag on a displaced wall (the walls internal/displacedface
// reads once per operand) is declined. The recorded Plane or Cylinder of such a
// wall lies only within its loop's bound of the wall the body denotes, offset
// and tilted, and facetedrecord publishes a tag's normal with no bound, so the
// face keeps no surface, as brep-modify §2.2's faceted view keeps none. Every
// other face keeps its tag.
func displacedWall(group facetedrecord.Group, f *topo.Face, walls map[*topo.Face]struct{}) facetedrecord.Group {
	if _, ok := walls[f]; !ok {
		return group
	}
	group.Surface, group.Retained, group.Reversed, group.NormalBound = nil, false, false, 0
	group.Denoted, group.CapRecord, group.AxialDelta, group.HasAxialDelta = nil, nil, 0, false
	return group
}
