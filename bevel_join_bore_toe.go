package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// bevelSortedTriangle ignores winding only while checking the retained blank
// band's exact cell census. orientJoinedMesh checks winding after reconstruction.
func bevelSortedTriangle(t [3]int) [3]int {
	if t[0] > t[1] {
		t[0], t[1] = t[1], t[0]
	}
	if t[1] > t[2] {
		t[1], t[2] = t[2], t[1]
	}
	if t[0] > t[1] {
		t[0], t[1] = t[1], t[0]
	}
	return t
}

type bevelToeBoreCross struct {
	x, round, slopeLo, slopeHi float64
}

// bevelToeBoreIntersection solves the recorded straight flank against the
// exact held circle radius, then encloses the coordinate written to the mesh.
func bevelToeBoreIntersection(root, toe Point2, radius float64) (bevelToeBoreCross, bool) {
	if !proofbound.FiniteVec(r3.Vec{X: root.U, Y: root.V, Z: radius}) ||
		!proofbound.FiniteVec(r3.Vec{X: toe.U, Y: toe.V}) {
		return bevelToeBoreCross{}, false
	}
	rootX, rootR := proofarith.FloatRat(root.U), proofarith.FloatRat(root.V)
	toeX, toeR := proofarith.FloatRat(toe.U), proofarith.FloatRat(toe.V)
	rad := proofarith.FloatRat(radius)
	dx := new(big.Rat).Sub(toeX, rootX)
	dr := new(big.Rat).Sub(rootR, toeR)
	if dx.Sign() <= 0 || dr.Sign() <= 0 || rad.Cmp(toeR) <= 0 || rad.Cmp(rootR) >= 0 {
		return bevelToeBoreCross{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(rootR, rad), dr)
	crossRat := new(big.Rat).Add(rootX, new(big.Rat).Mul(dx, fraction))
	if crossRat.Cmp(rootX) <= 0 || crossRat.Cmp(toeX) >= 0 {
		return bevelToeBoreCross{}, false
	}
	crossX, _ := crossRat.Float64()
	crossGap := new(big.Rat).Sub(crossRat, proofarith.FloatRat(crossX))
	crossGap.Abs(crossGap)
	slope := new(big.Rat).Quo(dr, dx)
	cross := bevelToeBoreCross{x: crossX, round: proofbound.RatFloatUp(crossGap),
		slopeLo: proofbound.RatFloatDown(slope), slopeHi: proofbound.RatFloatUp(slope)}
	if !proofbound.FiniteVec(r3.Vec{X: cross.x, Y: cross.slopeLo, Z: cross.slopeHi}) ||
		cross.slopeLo <= 0 || cross.round < 0 {
		return bevelToeBoreCross{}, false
	}
	return cross, true
}

// buildBevelPostJoinToeBore clips the blank's narrow toe flank where a coaxial
// bore reaches it. The certified tooth sectors remain outside the cylinder;
// only the blank's two cap fans and its final meridian band are rebuilt.
func buildBevelPostJoinToeBore(ctx context.Context, d *Document, ref producerID,
	joined facetedPayload, blank *bevelBlankSource, ring *bevelCapRingRecord,
	radius float64, wall *Face) (*Body, error) {
	n := len(ring.angles)
	if n < 16 || n > 1_536 || len(ring.heel) != n || len(ring.toeRoot) != n ||
		len(ring.toe) != n || len(joined.tris) != len(joined.src) ||
		len(joined.verts) != len(joined.vertexBound) ||
		ring.heelAxis == ring.toeAxis {
		return nil, fmt.Errorf("%w: bevel toe bore has no complete cap-ring record", ErrUnsupported)
	}
	// The recorded six-line meridian supplies the exact intersection with the
	// bore circle. It must be strictly inside the toe flank, away from both
	// source vertices and from the tooth-bearing root ring.
	cross, ok := bevelToeBoreIntersection(blank.toeRoot, blank.toeOuter, radius)
	if !ok || cross.x >= blank.heelAxis.U {
		return nil, fmt.Errorf("%w: bevel bore does not cross the toe flank once", ErrUnsupported)
	}
	crossX, crossRound, slopeLo, slopeHi := cross.x, cross.round, cross.slopeLo, cross.slopeHi

	heelPos, toePos := make(map[int]int, n), make(map[int]int, n)
	used := make(map[int]bool, 3*n+2)
	for _, axis := range [...]int{ring.heelAxis, ring.toeAxis} {
		if axis < 0 || axis >= len(joined.verts) || used[axis] {
			return nil, fmt.Errorf("%w: bevel toe bore repeats an axis vertex", ErrUnsupported)
		}
		used[axis] = true
	}
	for i := range n {
		for _, v := range [...]int{ring.heel[i], ring.toeRoot[i], ring.toe[i]} {
			if v < 0 || v >= len(joined.verts) || used[v] {
				return nil, fmt.Errorf("%w: bevel toe bore repeats a ring vertex", ErrUnsupported)
			}
			used[v] = true
		}
		heelPos[ring.heel[i]], toePos[ring.toe[i]] = i, i
	}
	// The old toe band consists of precisely two triangles per angular cell.
	// This check proves that removing group 4 drops no tooth or other blank
	// patch, and that its root ring stays paired with the new clipped band.
	toeBand := make(map[[3]int]bool, 2*n)
	for i := range n {
		j := (i + 1) % n
		for _, tri := range [...][3]int{
			{ring.toeRoot[i], ring.toe[i], ring.toe[j]},
			{ring.toeRoot[i], ring.toe[j], ring.toeRoot[j]},
		} {
			key := bevelSortedTriangle(tri)
			if _, exists := toeBand[key]; exists {
				return nil, fmt.Errorf("%w: bevel toe bore band repeats a cell", ErrUnsupported)
			}
			toeBand[key] = false
		}
	}
	cosHeld, sinHeld := make([]float64, n), make([]float64, n)
	maxRound, oldRound, maxStep := 0.0, 0.0, 0.0
	for i, angle := range ring.angles {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
		if !ok {
			return nil, fmt.Errorf("%w: bevel bore ring angle has no enclosure", ErrUnsupported)
		}
		cosHeld[i], sinHeld[i] = math.Cos(angle), math.Sin(angle)
		for _, sample := range [...]struct {
			rad   float64
			point r3.Vec
		}{
			{radius, r3.Vec{Y: radius * cosHeld[i], Z: radius * sinHeld[i]}},
			{blank.toeRoot.V, joined.verts[ring.toeRoot[i]]},
			{blank.toeOuter.V, joined.verts[ring.toe[i]]},
		} {
			exactRadius := proofarith.FloatRat(sample.rad)
			yErr := proofbound.IntervalFloatError(proofbound.IntervalScale(cosIv, exactRadius), sample.point.Y)
			zErr := proofbound.IntervalFloatError(proofbound.IntervalScale(sinIv, exactRadius), sample.point.Z)
			round := proofbound.Radius3D(max(yErr, zErr))
			if sample.rad == radius {
				maxRound = max(maxRound, round)
			} else {
				oldRound = max(oldRound, round)
			}
		}
		if joined.verts[ring.toeRoot[i]].X != blank.toeRoot.U ||
			joined.verts[ring.toe[i]].X != blank.toeOuter.U {
			return nil, fmt.Errorf("%w: bevel toe band changed its source stations", ErrUnsupported)
		}
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
			return nil, fmt.Errorf("%w: bevel toe bore cap angles are not ordered", ErrUnsupported)
		}
		maxStep = max(maxStep, stepUp)
	}
	sagFactor := proofbound.ProductUpper(maxStep, maxStep) / 8
	cutRound := proofbound.AbsSumUpper(maxRound, crossRound)
	boreBound := proofbound.AbsSumUpper(proofbound.ProductUpper(radius, sagFactor), cutRound)
	oldSideErr := proofbound.AbsSumUpper(proofbound.ProductUpper(blank.toeRoot.V, sagFactor), oldRound)
	newSideErr := proofbound.AbsSumUpper(
		proofbound.ProductUpper(max(blank.toeRoot.V, radius), sagFactor), oldRound, cutRound,
		proofbound.ProductUpper(slopeHi, crossRound))
	sideErr := max(oldSideErr, newSideErr)
	bound := max(joined.meshBound, boreBound, sideErr)
	if proofbound.IsNonFinite(bound) || bound > 0.1 || boreBound >= radius ||
		proofbound.AbsSumUpper(blank.toeOuter.V, joined.meshBound, boreBound) >= radius ||
		proofbound.AbsSumUpper(radius, joined.meshBound, boreBound) >= blank.toeRoot.V {
		return nil, fmt.Errorf("%w: bevel toe bore has no separate source bands", ErrUnsupported)
	}
	clearance := proofbound.AbsSumUpper(radius, joined.meshBound, boreBound)
	heelSeen, toeSeen := make([]bool, n), make([]bool, n)
	mesh := &joinedMesh{verts: append([]r3.Vec(nil), joined.verts...),
		groups: append([]facetGroup(nil), joined.groups...)}
	for i, tri := range joined.tris {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sector, fan, err := bevelBoreCapFan(tri, joined.src[i], ring.heelAxis, 1, heelPos, n)
		if err != nil {
			return nil, err
		}
		if fan {
			if heelSeen[sector] {
				return nil, fmt.Errorf("%w: bevel toe bore repeats a heel fan", ErrUnsupported)
			}
			heelSeen[sector] = true
			continue
		}
		sector, fan, err = bevelBoreCapFan(tri, joined.src[i], ring.toeAxis, 5, toePos, n)
		if err != nil {
			return nil, err
		}
		if fan {
			if toeSeen[sector] {
				return nil, fmt.Errorf("%w: bevel toe bore repeats a toe fan", ErrUnsupported)
			}
			toeSeen[sector] = true
			continue
		}
		if joined.src[i] == 4 {
			key := bevelSortedTriangle(tri)
			seen, expected := toeBand[key]
			if !expected || seen {
				return nil, fmt.Errorf("%w: bevel toe bore has an unexpected flank cell", ErrUnsupported)
			}
			toeBand[key] = true
			continue
		}
		if !bevelBoreTriangleClear(tri, joined.verts, clearance) {
			return nil, fmt.Errorf("%w: a tooth or untouched blank face may enter the bore", ErrUnsupported)
		}
		mesh.add(tri, joined.src[i])
	}
	for i := range n {
		if !heelSeen[i] || !toeSeen[i] {
			return nil, fmt.Errorf("%w: bevel toe bore misses a cap sector", ErrUnsupported)
		}
	}
	for _, seen := range toeBand {
		if !seen {
			return nil, fmt.Errorf("%w: bevel toe bore misses a flank cell", ErrUnsupported)
		}
	}
	wallGroup := len(mesh.groups)
	mesh.groups = append(mesh.groups, facetGroup{origins: wall.Origins(),
		surface: wall.Surface(), normalBound: wall.normalBound, reversed: !wall.reversed})
	vertexBound := append([]float64(nil), joined.vertexBound...)
	innerHeel := make([]int, n)
	for i := range n {
		cut := r3.Vec{X: crossX, Y: radius * cosHeld[i], Z: radius * sinHeld[i]}
		mesh.verts[ring.toe[i]] = cut
		vertexBound[ring.toe[i]] = max(boreBound, newSideErr)
		inner := r3.Vec{X: blank.heelAxis.U, Y: radius * cosHeld[i], Z: radius * sinHeld[i]}
		switch i {
		case 0:
			innerHeel[i] = ring.heelAxis
		case 1:
			innerHeel[i] = ring.toeAxis
		default:
			innerHeel[i] = len(mesh.verts)
			mesh.verts = append(mesh.verts, inner)
			vertexBound = append(vertexBound, boreBound)
		}
		mesh.verts[innerHeel[i]] = inner
		vertexBound[innerHeel[i]] = boreBound
	}
	for i := range n {
		j := (i + 1) % n
		mesh.add([3]int{ring.toeRoot[i], ring.toe[i], ring.toe[j]}, 4)
		mesh.add([3]int{ring.toeRoot[i], ring.toe[j], ring.toeRoot[j]}, 4)
		mesh.add([3]int{innerHeel[i], ring.heel[i], ring.heel[j]}, 1)
		mesh.add([3]int{innerHeel[i], ring.heel[j], innerHeel[j]}, 1)
		mesh.add([3]int{ring.toe[i], innerHeel[i], innerHeel[j]}, wallGroup)
		mesh.add([3]int{ring.toe[i], innerHeel[j], ring.toe[j]}, wallGroup)
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.verts), mesh.tris); err != nil {
		return nil, err
	}
	// The local rewrite removes four old blank triangles and adds six per
	// station. The joined pattern already proved its adaptive cap-cell ceiling.
	// Its 25-million-per-tooth scan budget also covers the local bore rewrite:
	// the real twelve-tooth source scans 221,081,767 pairs under 300 million.
	maxAuditTriangles := 44_480 + 22_000*(12-2) + 6*bevelCombineCapOnePassLimit*12 + 2*n
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		mesh.verts, mesh.tris, maxAuditTriangles, 25_000_000*12); err != nil {
		return nil, err
	}
	// The original joined solid and its held mesh keep their symmetric-volume
	// allowance after both are cut by the exact cylinder. On each old and new
	// toe-flank cell, circular-chord sag and held coordinate rounding bound
	// radial distance from the same recorded straight meridian by sideErr.
	// The two clipped strips can differ only in that radial shell, or in the
	// axial slab where the old chord meets the cylinder near crossRat.
	rootX := proofarith.FloatRat(blank.toeRoot.U)
	span := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(blank.toeOuter.U), rootX))
	height := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(blank.heelAxis.U), rootX))
	seamHalf := proofbound.DivUpper(sideErr, slopeLo)
	if proofbound.IsNonFinite(span) || proofbound.IsNonFinite(height) ||
		proofbound.IsNonFinite(seamHalf) || height <= 0 {
		return nil, fmt.Errorf("%w: bevel toe bore has no finite source span", ErrUnsupported)
	}
	sideShell := proofbound.ProductUpper(proofbound.ProductUpper(4, proofbound.TwoPiUpper()/2),
		proofbound.ProductUpper(proofbound.AbsSumUpper(blank.toeRoot.V, sideErr),
			proofbound.ProductUpper(sideErr, span)))
	seamRadius := proofbound.AbsSumUpper(radius, sideErr)
	seamSlab := proofbound.ProductUpper(proofbound.ProductUpper(2, seamHalf),
		proofbound.ProductUpper(proofbound.TwoPiUpper()/2,
			proofbound.ProductUpper(seamRadius, seamRadius)))
	circleShell := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(2, proofbound.TwoPiUpper()), radius),
		proofbound.ProductUpper(boreBound, height))
	volumeGap := proofbound.AbsSumUpper(joined.volSymDiff, sideShell, seamSlab, circleShell)
	oldHeldArea, err := proofbound.PerturbedAreaUpperContext(ctx,
		joined.verts, joined.tris, joined.meshBound)
	if err != nil {
		return nil, err
	}
	newHeldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, bound)
	if err != nil {
		return nil, err
	}
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.areaSlack, oldHeldArea, newHeldArea, wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: bevel toe bore has no finite measure proof", ErrUnsupported)
	}
	result := joined
	result.verts, result.tris, result.src, result.groups = mesh.verts, mesh.tris, mesh.src, mesh.groups
	result.vertexBound, result.meshBound = vertexBound, bound
	result.volSymDiff, result.areaSlack = volumeGap, areaSlack
	result.bevelJoin = nil
	result.contactAudited = true
	return buildFacetedBody(ctx, d, ref, result)
}
