package meshthick

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Undercuts returns, in mesh order, the triangles of a closed outward-wound mesh
// that the pull proves undercut (agent-docs/mesh-wall-survey-design.md §6). A
// triangle whose outward normal has a component along the pull must be released
// along it, and one with a component against the pull along its reverse. The
// ray from the triangle's centre in that direction leaves the surface into the
// open; a nearer triangle that it crosses from the open into material, at least
// clearance out, blocks the release, and the triangle is undercut. A triangle
// seen edge on to the pull, within edgeOn of square to it, casts no ray, and a
// blocking triangle must be crossed more squarely than edgeOn too. pull must be
// a unit vector.
//
// The test finds blocked triangles and cannot show a triangle free: only the
// triangle's centre is sampled. clearance should cover the mesh's displacement
// bound twice, so a hit is not the mesh's own approximation of the triangle's
// neighbours.
func Undercuts(budget *proofbound.WorkBudget, t *Tree, pull r3.Vec, clearance float64) ([]int, error) {
	if t.Empty() {
		return nil, nil
	}
	var blocked []int
	for i, tri := range t.tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		a, b, c := t.verts[tri[0]], t.verts[tri[1]], t.verts[tri[2]]
		normal, ok := b.Sub(a).Cross(c.Sub(a)).Normalize()
		if !ok {
			continue
		}
		along := normal.Dot(pull)
		if along < edgeOn && along > -edgeOn {
			continue
		}
		dir := pull
		if along < 0 {
			dir = pull.Scale(-1)
		}
		origin := a.Add(b).Add(c).Scale(1.0 / 3)
		if _, _, ok := t.nearestHit(origin, dir, i, clearance, math.Inf(1), Entering); ok {
			blocked = append(blocked, i)
		}
	}
	return blocked, nil
}
