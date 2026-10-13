package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// revolveSectorSamples caches the per-station cone samples shared by every
// plan of one construction attempt.
type revolveSectorSamples struct {
	up, down []pointcone.Sample
	filled   bool
}

// buildRevolveSectorMesh closes an admitted sector against the root face's
// two ordered section rings. The section walk, rather than a profile role,
// selects the rings shared with the blank mesh.
func buildRevolveSectorMesh(mesh *joinedMesh, ring revolvedSectionRing,
	section revolveSectionCertificate, nodes, original []sectorPathNode, caps [][3]int,
	edges []sectorCapEdge, rootNode []int, record *pointConeTrimRecord,
	motion r3.Transform, placedGroups []facetGroup, samples *revolveSectorSamples) (float64, float64, error) {
	if mesh == nil || record == nil || record.base.pointSection == nil ||
		len(nodes) != len(rootNode) || len(nodes) != len(original) || len(nodes) < 3 ||
		section.rootSpan < 0 || section.rootSpan >= len(ring.rings) {
		return 0, 0, fmt.Errorf("%w: sector cap has invalid source records", ErrUnsupported)
	}
	root := section.spans[section.rootSpan]
	lowSpan, highSpan := section.rootSpan, (section.rootSpan+1)%len(section.spans)
	if root.start.U > root.end.U {
		lowSpan, highSpan = highSpan, lowSpan
	}
	baseGroup := len(mesh.groups)
	if placedGroups != nil && len(placedGroups) != len(record.base.groups)+2 {
		return 0, 0, fmt.Errorf("%w: placed sector lost its source faces", ErrUnsupported)
	}
	for i, group := range record.base.groups {
		if i == len(record.base.groups)-1 {
			break
		}
		group.surface = NURBSSurface{}
		if placedGroups != nil {
			group = placedGroups[i]
		}
		mesh.groups = append(mesh.groups, group)
	}
	upperGroup := len(mesh.groups)
	upperFace := pointConeFacetGroup(record.upper, false)
	if placedGroups != nil {
		upperFace = placedGroups[len(record.base.groups)]
	}
	mesh.groups = append(mesh.groups, upperFace)
	lowerGroup := len(mesh.groups)
	lowerFace := pointConeFacetGroup(record.lower, true)
	if placedGroups != nil {
		lowerFace = placedGroups[len(record.base.groups)+1]
	}
	mesh.groups = append(mesh.groups, lowerFace)
	upper, lower := make([]int, len(nodes)), make([]int, len(nodes))
	maxRound, maxSnap := 0.0, 0.0
	// The cone samples depend only on the unplaced cap station and
	// the cone, never on the plan's motion, so the first plan's samples are
	// reused by every later plan. They are filled in node order inside the
	// first plan's loop, so a refusal is raised at the same node and in the
	// same order as before.
	if samples.up == nil {
		samples.up = make([]pointcone.Sample, len(nodes))
		samples.down = make([]pointcone.Sample, len(nodes))
	}
	for i := range nodes {
		far := original[i].p
		var up, down pointcone.Sample
		if samples.filled {
			up, down = samples.up[i], samples.down[i]
		} else {
			var err error
			up, err = pointConeFactor(far, record.upper)
			if err != nil {
				return 0, 0, err
			}
			down, err = pointConeFactor(far, record.lower)
			if err != nil {
				return 0, 0, err
			}
			samples.up[i], samples.down[i] = up, down
		}
		if down.Q-down.QError <= 1 {
			return 0, 0, fmt.Errorf("%w: sector cap reaches its source plane", ErrUnsupported)
		}
		upperPoint, upperRound, err := sectorUpperCapNode(far, up, false)
		if err != nil {
			return 0, 0, err
		}
		lowerPoint := far.Scale(down.Alpha)
		if motion != r3.Identity() {
			maxInput := max(math.Abs(upperPoint.X), math.Abs(upperPoint.Y), math.Abs(upperPoint.Z),
				math.Abs(lowerPoint.X), math.Abs(lowerPoint.Y), math.Abs(lowerPoint.Z))
			maxRound = max(maxRound, proofbound.AbsSumUpper(upperRound, down.PointError,
				proofbound.RigidRoundAllow(maxInput, 0)))
			upperPoint, lowerPoint = motion.Apply(upperPoint), motion.Apply(lowerPoint)
		} else {
			maxRound = max(maxRound, upperRound, down.PointError)
		}
		if !proofbound.FiniteVec(upperPoint) || !proofbound.FiniteVec(lowerPoint) {
			return 0, 0, fmt.Errorf("%w: sector cap station is non-finite", ErrUnsupported)
		}
		if rootNode[i] >= 0 {
			j := rootNode[i]
			if j >= len(ring.angles) {
				return 0, 0, fmt.Errorf("%w: sector root station is missing", ErrUnsupported)
			}
			upper[i], lower[i] = ring.vertex(highSpan, j), ring.vertex(lowSpan, j)
			maxSnap = max(maxSnap,
				proofbound.DvLenUpper(proofbound.HeldDelta(upperPoint, mesh.verts[upper[i]])),
				proofbound.DvLenUpper(proofbound.HeldDelta(lowerPoint, mesh.verts[lower[i]])))
			continue
		}
		upper[i] = len(mesh.verts)
		mesh.verts = append(mesh.verts, upperPoint)
		lower[i] = len(mesh.verts)
		mesh.verts = append(mesh.verts, lowerPoint)
	}
	samples.filled = true
	for _, tri := range caps {
		mesh.add([3]int{upper[tri[0]], upper[tri[1]], upper[tri[2]]}, upperGroup)
		mesh.add([3]int{lower[tri[2]], lower[tri[1]], lower[tri[0]]}, lowerGroup)
	}
	for _, edge := range edges {
		if edge.group < 0 {
			continue
		}
		if edge.group >= len(record.base.groups)-1 {
			return 0, 0, fmt.Errorf("%w: sector face role is missing", ErrUnsupported)
		}
		u, v := edge.u, edge.v
		mesh.add([3]int{upper[u], lower[u], lower[v]}, baseGroup+edge.group)
		mesh.add([3]int{upper[u], lower[v], upper[v]}, baseGroup+edge.group)
	}
	return maxRound, maxSnap, nil
}
