package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const bevelBoreSideRole = "side(0,0)"

// tryBevelPostJoinBoreCut cuts a coaxial, spanning circle from the complete
// eight-tooth structural join. Its source record identifies both axis-cap
// fans, so this path cuts them without asking the broad joined mesh to resolve
// a second local Boolean contact.
func tryBevelPostJoinBoreCut(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpCut {
		return nil, false, nil
	}
	joined, ok := target.payload.(facetedPayload)
	if !ok || joined.bevelJoin == nil || joined.bevelJoin.capRing == nil ||
		len(joined.bevelJoin.placed) != 7 || joined.xform != r3.Identity() ||
		!joined.contactAudited {
		return nil, false, nil
	}
	blank, ok := exactBevelBlank(joined.bevelJoin.blank)
	if !ok {
		return nil, false, nil
	}
	prism, ok := tool.payload.(prismPayload)
	if !ok || prism.surfaceResult || prism.xform != r3.Identity() ||
		prism.sectionDelta != 0 || prism.z0Delta != 0 || prism.z1Delta != 0 ||
		prism.frame.U() != (r3.Vec{Y: 1}) || prism.frame.V() != (r3.Vec{Z: 1}) ||
		prism.frame.N() != (r3.Vec{X: 1}) ||
		prism.frame.Origin().Y != 0 || prism.frame.Origin().Z != 0 ||
		len(prism.profile.Holes) != 0 || len(prism.profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	circle, ok := prism.profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center != (Point2{}) ||
		!((circle.TStart == 0 && circle.TEnd == 1) ||
			(circle.TStart == 1 && circle.TEnd == 0)) {
		return nil, false, nil
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil {
		return nil, false, err
	}
	origin := prism.frame.Origin()
	if !proofbound.FiniteVec(origin) ||
		!proofbound.FiniteVec(r3.Vec{X: prism.z0, Y: prism.z1}) {
		return nil, false, nil
	}
	// Compare the held tool ends as exact sums. Rounding origin.X+z could
	// otherwise put an end just outside a cap that it actually touches.
	start := new(big.Rat).Add(proofarith.FloatRat(origin.X), proofarith.FloatRat(prism.z0))
	end := new(big.Rat).Add(proofarith.FloatRat(origin.X), proofarith.FloatRat(prism.z1))
	if !proofbound.FiniteVec(r3.Vec{X: radius}) || radius <= 0 ||
		start.Cmp(proofarith.FloatRat(blank.toeAxis.U)) >= 0 ||
		end.Cmp(proofarith.FloatRat(blank.heelAxis.U)) <= 0 ||
		radius >= min(blank.heelOuter.V, blank.ded.V, blank.toeRoot.V, blank.toeOuter.V) {
		return nil, false, nil
	}
	var wall *Face
	for _, face := range tool.Faces() {
		for _, origin := range face.Origins() {
			if origin.Role != bevelBoreSideRole {
				continue
			}
			if wall != nil {
				return nil, false, nil
			}
			if _, ok := face.Surface().(Cylinder); !ok {
				return nil, false, nil
			}
			wall = face
		}
	}
	if wall == nil {
		return nil, false, nil
	}
	result, err := buildBevelPostJoinBore(ctx, d, ref, joined, blank,
		joined.bevelJoin.capRing, radius, wall)
	return result, true, err
}

// bevelBoreCapFan identifies one complete triangle of the original axis fan.
// A different use of the axis vertex is an invariant failure in the retained
// structural record, not a cap triangle this rewrite may silently discard.
func bevelBoreCapFan(t [3]int, src, axis, group int, positions map[int]int, n int) (int, bool, error) {
	if t[0] != axis && t[1] != axis && t[2] != axis {
		return 0, false, nil
	}
	other := [2]int{}
	k := 0
	for _, v := range t {
		if v == axis {
			continue
		}
		other[k] = v
		k++
	}
	if k != 2 || src != group {
		return 0, false, fmt.Errorf("%w: bevel bore axis belongs to another face", ErrUnsupported)
	}
	a, aOK := positions[other[0]]
	b, bOK := positions[other[1]]
	if !aOK || !bOK {
		return 0, false, fmt.Errorf("%w: bevel bore cap fan misses its outer ring", ErrUnsupported)
	}
	if (a+1)%n == b {
		return a, true, nil
	}
	if (b+1)%n == a {
		return b, true, nil
	}
	return 0, false, fmt.Errorf("%w: bevel bore cap fan crosses angular stations", ErrUnsupported)
}

func bevelBoreTriangleClear(t [3]int, verts []r3.Vec, clearance float64) bool {
	a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
	loY, hiY := min(a.Y, b.Y, c.Y), max(a.Y, b.Y, c.Y)
	loZ, hiZ := min(a.Z, b.Z, c.Z), max(a.Z, b.Z, c.Z)
	return loY > clearance || hiY < -clearance || loZ > clearance || hiZ < -clearance
}

func buildBevelPostJoinBore(ctx context.Context, d *Document, ref producerID,
	joined facetedPayload, blank *bevelBlankSource, ring *bevelCapRingRecord,
	radius float64, wall *Face) (*Body, error) {
	n := len(ring.angles)
	if n < 16 || len(ring.heel) != n || len(ring.toe) != n ||
		len(joined.tris) != len(joined.src) || len(joined.verts) != len(joined.vertexBound) ||
		ring.heelAxis == ring.toeAxis {
		return nil, fmt.Errorf("%w: bevel bore has no complete cap-ring record", ErrUnsupported)
	}
	heelPos, toePos := make(map[int]int, n), make(map[int]int, n)
	for i := range n {
		if ring.heel[i] < 0 || ring.heel[i] >= len(joined.verts) ||
			ring.toe[i] < 0 || ring.toe[i] >= len(joined.verts) {
			return nil, fmt.Errorf("%w: bevel bore cap ring leaves the joined mesh", ErrUnsupported)
		}
		heelPos[ring.heel[i]] = i
		toePos[ring.toe[i]] = i
	}
	if ring.heelAxis < 0 || ring.heelAxis >= len(joined.verts) ||
		ring.toeAxis < 0 || ring.toeAxis >= len(joined.verts) ||
		len(heelPos) != n || len(toePos) != n {
		return nil, fmt.Errorf("%w: bevel bore cap ring repeats a vertex", ErrUnsupported)
	}
	// The same ordered angular stations built the blank's complete outer cap
	// rings. Interval trigonometry charges each new inner vertex; the step
	// bound charges the exact circular wall between those vertices.
	innerCos, innerSin := make([]float64, n), make([]float64, n)
	maxRound, maxStep := 0.0, 0.0
	ratRadius := proofarith.FloatRat(radius)
	for i, angle := range ring.angles {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
		if !ok {
			return nil, fmt.Errorf("%w: bevel bore ring angle has no trigonometric enclosure", ErrUnsupported)
		}
		innerCos[i], innerSin[i] = radius*math.Cos(angle), radius*math.Sin(angle)
		yErr := proofbound.IntervalFloatError(proofbound.IntervalScale(cosIv, ratRadius), innerCos[i])
		zErr := proofbound.IntervalFloatError(proofbound.IntervalScale(sinIv, ratRadius), innerSin[i])
		maxRound = max(maxRound, proofbound.Radius3D(max(yErr, zErr)))
		j := (i + 1) % n
		step := new(big.Rat)
		if j == 0 {
			step.Add(proofbound.TwoPiInterval().Hi,
				new(big.Rat).Sub(proofarith.FloatRat(ring.angles[0]), proofarith.FloatRat(angle)))
		} else {
			step.Sub(proofarith.FloatRat(ring.angles[j]), proofarith.FloatRat(angle))
		}
		stepUp := proofbound.RatFloatUp(step)
		if step.Sign() <= 0 || proofbound.IsNonFinite(stepUp) || stepUp > 0.025 {
			return nil, fmt.Errorf("%w: bevel bore cap angles are not ordered", ErrUnsupported)
		}
		maxStep = max(maxStep, stepUp)
	}
	sag := proofbound.ProductUpper(radius, proofbound.ProductUpper(maxStep, maxStep)/8)
	boreBound := proofbound.AbsSumUpper(sag, maxRound)
	if proofbound.IsNonFinite(boreBound) || boreBound >= radius {
		return nil, fmt.Errorf("%w: bevel bore circle has no radial bound", ErrUnsupported)
	}
	clearance := proofbound.AbsSumUpper(radius, joined.meshBound, boreBound)
	heelSeen, toeSeen := make([]bool, n), make([]bool, n)
	mesh := &bevelJoinedMesh{
		verts:  append([]r3.Vec(nil), joined.verts...),
		groups: append([]facetGroup(nil), joined.groups...),
	}
	for i, t := range joined.tris {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sector, fan, err := bevelBoreCapFan(t, joined.src[i], ring.heelAxis, 1, heelPos, n)
		if err != nil {
			return nil, err
		}
		if fan {
			if heelSeen[sector] {
				return nil, fmt.Errorf("%w: bevel bore heel fan repeats a sector", ErrUnsupported)
			}
			heelSeen[sector] = true
			continue
		}
		sector, fan, err = bevelBoreCapFan(t, joined.src[i], ring.toeAxis, 5, toePos, n)
		if err != nil {
			return nil, err
		}
		if fan {
			if toeSeen[sector] {
				return nil, fmt.Errorf("%w: bevel bore toe fan repeats a sector", ErrUnsupported)
			}
			toeSeen[sector] = true
			continue
		}
		if !bevelBoreTriangleClear(t, joined.verts, clearance) {
			return nil, fmt.Errorf("%w: a joined tooth or side face may enter the bore", ErrUnsupported)
		}
		mesh.add(t, joined.src[i])
	}
	for i := range n {
		if !heelSeen[i] || !toeSeen[i] {
			return nil, fmt.Errorf("%w: bevel bore cap fan misses an angular sector", ErrUnsupported)
		}
	}
	wallGroup := len(mesh.groups)
	mesh.groups = append(mesh.groups, facetGroup{origins: wall.Origins(),
		surface: wall.Surface(), normalBound: wall.normalBound,
		reversed: !wall.reversed})
	vertexBound := append([]float64(nil), joined.vertexBound...)
	innerHeel, innerToe := make([]int, n), make([]int, n)
	for i := range n {
		heelPoint := r3.Vec{X: blank.heelAxis.U, Y: innerCos[i], Z: innerSin[i]}
		toePoint := r3.Vec{X: blank.toeAxis.U, Y: innerCos[i], Z: innerSin[i]}
		if i == 0 {
			innerHeel[i], innerToe[i] = ring.heelAxis, ring.toeAxis
			mesh.verts[ring.heelAxis], mesh.verts[ring.toeAxis] = heelPoint, toePoint
			vertexBound[ring.heelAxis], vertexBound[ring.toeAxis] = boreBound, boreBound
			continue
		}
		innerHeel[i] = len(mesh.verts)
		mesh.verts = append(mesh.verts, heelPoint)
		vertexBound = append(vertexBound, boreBound)
		innerToe[i] = len(mesh.verts)
		mesh.verts = append(mesh.verts, toePoint)
		vertexBound = append(vertexBound, boreBound)
	}
	for i := range n {
		j := (i + 1) % n
		mesh.add([3]int{innerHeel[i], ring.heel[i], ring.heel[j]}, 1)
		mesh.add([3]int{innerHeel[i], ring.heel[j], innerHeel[j]}, 1)
		mesh.add([3]int{ring.toe[i], innerToe[i], innerToe[j]}, 5)
		mesh.add([3]int{ring.toe[i], innerToe[j], ring.toe[j]}, 5)
		mesh.add([3]int{innerToe[i], innerHeel[i], innerHeel[j]}, wallGroup)
		mesh.add([3]int{innerToe[i], innerHeel[j], innerToe[j]}, wallGroup)
	}
	if err := bevelOrientMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.verts), mesh.tris); err != nil {
		return nil, err
	}
	if n < 16 || n > 1_536 {
		return nil, fmt.Errorf("%w: bevel bore requires at most 1536 ring stations", ErrUnsupported)
	}
	// The annular strips and inward wall add four net triangles per station
	// after the old axis fans are removed.
	const maxPatternAuditTriangles = 44_480 + 22_000*6
	maxAuditTriangles := maxPatternAuditTriangles + 4*n
	if err := loftmesh.LoftCrossingAuditBounded(
		proofbound.NewWorkBudget(ctx), mesh.verts, mesh.tris,
		maxAuditTriangles, 128_000_000); err != nil {
		return nil, err
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(blank.heelAxis.U), proofarith.FloatRat(blank.toeAxis.U)))
	if height <= 0 || proofbound.IsNonFinite(height) {
		return nil, fmt.Errorf("%w: bevel bore has no positive axial span", ErrUnsupported)
	}
	// Subtracting one exact cylinder from the true and held joined solids
	// cannot enlarge their symmetric difference. The held polygonal bore may
	// differ from that cylinder only in the radial shell of thickness
	// boreBound, over this exact blank cap-to-cap span.
	shellArea := proofbound.ProductUpper(
		proofbound.ProductUpper(2, proofbound.TwoPiUpper()),
		proofbound.ProductUpper(radius, boreBound))
	volumeGap := proofbound.AbsSumUpper(joined.volSymDiff,
		proofbound.ProductUpper(shellArea, height))
	bound := max(joined.meshBound, boreBound)
	heldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, bound)
	if err != nil {
		return nil, err
	}
	oldArea, err := joined.bevelJoin.blank.Area()
	if err != nil {
		return nil, err
	}
	// The source body's stored areaSlack already bounds its true/held area
	// gap. This broad addition covers either cap disc removed by the Cut and
	// the newly exposed cylinder without relying on cancellation of areas.
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.areaSlack, heldArea,
		oldArea.Value.Base(), oldArea.Bound.Base(), wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: bevel bore has no finite measure proof", ErrUnsupported)
	}
	result := joined
	result.verts, result.tris, result.src, result.groups = mesh.verts, mesh.tris, mesh.src, mesh.groups
	result.vertexBound, result.meshBound = vertexBound, bound
	result.volSymDiff, result.areaSlack = volumeGap, areaSlack
	result.bevelJoin = nil
	result.contactAudited = true
	return buildFacetedBody(ctx, d, ref, result)
}
