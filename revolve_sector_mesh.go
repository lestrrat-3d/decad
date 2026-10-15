package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
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
	section revolveSectionCertificate, nodes, original []pointcone.SectorNode, caps [][3]int,
	edges []pointcone.SectorEdge, rootNode []int, record *pointConeTrimRecord,
	motion r3.Transform, placedGroups []facetGroup, samples *revolveSectorSamples) (float64, float64, error) {
	if mesh == nil || record == nil || record.Base.PointSection == nil ||
		len(nodes) != len(rootNode) || len(nodes) != len(original) || len(nodes) < 3 ||
		section.RootSpan < 0 || section.RootSpan >= len(ring.Rings) {
		return 0, 0, fmt.Errorf("%w: sector cap has invalid source records", ErrUnsupported)
	}
	root := section.Spans[section.RootSpan]
	lowSpan, highSpan := section.RootSpan, (section.RootSpan+1)%len(section.Spans)
	if root.Start.U > root.End.U {
		lowSpan, highSpan = highSpan, lowSpan
	}
	baseGroup := len(mesh.Groups)
	if placedGroups != nil && len(placedGroups) != len(record.Base.Groups)+2 {
		return 0, 0, fmt.Errorf("%w: placed sector lost its source faces", ErrUnsupported)
	}
	for i, group := range record.Base.Groups {
		if i == len(record.Base.Groups)-1 {
			break
		}
		group.Surface = surfacegeom.NURBSSurface{}
		if placedGroups != nil {
			group = placedGroups[i]
		}
		mesh.Groups = append(mesh.Groups, group)
	}
	upperGroup := len(mesh.Groups)
	upperFace := pointConeFacetGroup(record.Upper, false)
	if placedGroups != nil {
		upperFace = placedGroups[len(record.Base.Groups)]
	}
	mesh.Groups = append(mesh.Groups, upperFace)
	lowerGroup := len(mesh.Groups)
	lowerFace := pointConeFacetGroup(record.Lower, true)
	if placedGroups != nil {
		lowerFace = placedGroups[len(record.Base.Groups)+1]
	}
	mesh.Groups = append(mesh.Groups, lowerFace)
	upper, lower := make([]int, len(nodes)), make([]int, len(nodes))
	maxRound, maxSnap := 0.0, 0.0
	// The cone samples depend only on the unplaced cap station and
	// the cone, never on the plan's motion, so the first plan's samples are
	// reused by every later plan. They are formed for every node before the
	// first plan's loop and their errors are read in node order inside it, so a
	// refusal is raised at the same node and in the same order as a node-by-node
	// computation.
	var upErrs, downErrs []error
	if !samples.filled {
		points := make([]r3.Vec, len(original))
		for i := range original {
			points[i] = original[i].P
		}
		w := orderedwork.EachWorkers(len(points), pointConeFactorParallelMin)
		samples.up, upErrs = pointConeFactors(points, record.Upper, w)
		samples.down, downErrs = pointConeFactors(points, record.Lower, w)
	}
	for i := range nodes {
		far := original[i].P
		var up, down pointcone.Sample
		if samples.filled {
			up, down = samples.up[i], samples.down[i]
		} else {
			if upErrs[i] != nil {
				return 0, 0, upErrs[i]
			}
			if downErrs[i] != nil {
				return 0, 0, downErrs[i]
			}
			up, down = samples.up[i], samples.down[i]
		}
		if down.Q-down.QError <= 1 {
			return 0, 0, fmt.Errorf("%w: sector cap reaches its source plane", ErrUnsupported)
		}
		upperPoint, upperRound, err := pointcone.UpperCapNode(far, up, false)
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
			if j >= len(ring.Angles) {
				return 0, 0, fmt.Errorf("%w: sector root station is missing", ErrUnsupported)
			}
			upper[i], lower[i] = ring.Vertex(highSpan, j), ring.Vertex(lowSpan, j)
			maxSnap = max(maxSnap,
				proofbound.DvLenUpper(proofbound.HeldDelta(upperPoint, mesh.Verts[upper[i]])),
				proofbound.DvLenUpper(proofbound.HeldDelta(lowerPoint, mesh.Verts[lower[i]])))
			continue
		}
		upper[i] = len(mesh.Verts)
		mesh.Verts = append(mesh.Verts, upperPoint)
		lower[i] = len(mesh.Verts)
		mesh.Verts = append(mesh.Verts, lowerPoint)
	}
	samples.filled = true
	for _, tri := range caps {
		mesh.Add([3]int{upper[tri[0]], upper[tri[1]], upper[tri[2]]}, upperGroup)
		mesh.Add([3]int{lower[tri[2]], lower[tri[1]], lower[tri[0]]}, lowerGroup)
	}
	for _, edge := range edges {
		if edge.Group < 0 {
			continue
		}
		if edge.Group >= len(record.Base.Groups)-1 {
			return 0, 0, fmt.Errorf("%w: sector face role is missing", ErrUnsupported)
		}
		u, v := edge.U, edge.V
		mesh.Add([3]int{upper[u], lower[u], lower[v]}, baseGroup+edge.Group)
		mesh.Add([3]int{upper[u], lower[v], upper[v]}, baseGroup+edge.Group)
	}
	return maxRound, maxSnap, nil
}
