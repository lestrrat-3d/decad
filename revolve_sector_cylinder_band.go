package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

type revolveSectorBandCross struct {
	x, round, slopeLo, slopeHi float64
}

// revolveSectorStraightBandCross solves a circle/straight-meridian crossing
// with exact dyadic source coordinates. Its rounded axial mesh coordinate
// carries a separate outward error and slope interval.
func revolveSectorStraightBandCross(outer, capPoint planePoint, radius float64) (revolveSectorBandCross, bool) {
	if !proofbound.FiniteVec(r3.Vec{X: outer.U, Y: outer.V, Z: radius}) ||
		!proofbound.FiniteVec(r3.Vec{X: capPoint.U, Y: capPoint.V}) {
		return revolveSectorBandCross{}, false
	}
	outerX, outerR := proofarith.FloatRat(outer.U), proofarith.FloatRat(outer.V)
	capX, capR := proofarith.FloatRat(capPoint.U), proofarith.FloatRat(capPoint.V)
	rad := proofarith.FloatRat(radius)
	dx := new(big.Rat).Sub(capX, outerX)
	dr := new(big.Rat).Sub(outerR, capR)
	if dx.Sign() <= 0 || dr.Sign() <= 0 || rad.Cmp(capR) <= 0 || rad.Cmp(outerR) >= 0 {
		return revolveSectorBandCross{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(outerR, rad), dr)
	crossRat := new(big.Rat).Add(outerX, new(big.Rat).Mul(dx, fraction))
	if crossRat.Cmp(outerX) <= 0 || crossRat.Cmp(capX) >= 0 {
		return revolveSectorBandCross{}, false
	}
	crossX, _ := crossRat.Float64()
	crossGap := new(big.Rat).Sub(crossRat, proofarith.FloatRat(crossX))
	crossGap.Abs(crossGap)
	slope := new(big.Rat).Quo(dr, dx)
	cross := revolveSectorBandCross{x: crossX, round: proofbound.RatFloatUp(crossGap),
		slopeLo: proofbound.RatFloatDown(slope), slopeHi: proofbound.RatFloatUp(slope)}
	if !proofbound.FiniteVec(r3.Vec{X: cross.x, Y: cross.slopeLo, Z: cross.slopeHi}) ||
		cross.slopeLo <= 0 || cross.round < 0 {
		return revolveSectorBandCross{}, false
	}
	return cross, true
}

func revolveSectorSortedTriangle(t [3]int) [3]int {
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

// buildRevolveSectorBandCut rewrites one complete cap-adjacent straight band.
// The full band and both cap fans are counted before any facets are removed.
func buildRevolveSectorBandCut(ctx context.Context, d *Document, ref producerID,
	joined facetedPayload, caps revolveSectorBoreCaps, radius float64, wall *Face) (*Body, bool, error) {
	record := joined.RevolveLoftJoin
	section, ring := record.Certificate.Section, record.CapRing
	n, count := len(ring.Angles), len(section.Spans)
	bandIndex := (caps.low + count - 1) % count
	band := section.Spans[bandIndex]
	if bandIndex == section.AxisSpan || bandIndex == section.RootSpan ||
		band.End != section.Spans[caps.low].Start ||
		band.Start.V <= radius || band.End.V >= radius ||
		band.Face == nil || len(band.Origins) == 0 ||
		len(ring.Rings[bandIndex]) != n ||
		ring.Groups[bandIndex] < 0 || ring.Groups[bandIndex] >= len(joined.Groups) ||
		len(joined.Tris) > revolveSectorBoreMaxAuditTriangles-2*n {
		return nil, true, fmt.Errorf("%w: bore has no complete cap-adjacent straight band", ErrUnsupported)
	}
	cross, ok := revolveSectorStraightBandCross(band.Start, band.End, radius)
	if !ok || cross.x >= caps.highX {
		return nil, true, fmt.Errorf("%w: bore does not cross one straight section band", ErrUnsupported)
	}
	maxStep, ok := revolvemesh.SectorRingMaxStep(ring.Angles)
	if !ok {
		return nil, true, fmt.Errorf("%w: section ring has no bounded angular plan", ErrUnsupported)
	}
	inner, boreBound, err := circularbounds.BoreRingAtStep(ctx, radius, ring.Angles, maxStep)
	if err != nil {
		return nil, true, err
	}
	if boreBound >= radius || proofbound.IsNonFinite(boreBound) {
		return nil, true, fmt.Errorf("%w: bore circle has no radial bound", ErrUnsupported)
	}
	lowPos, highPos := make(map[int]int, n), make(map[int]int, n)
	used := map[int]bool{caps.lowAxis: true, caps.highAxis: true}
	rootRing := ring.Rings[bandIndex]
	for i := range n {
		for _, v := range [...]int{caps.lowOuter[i], caps.highOuter[i], rootRing[i]} {
			if v < 0 || v >= len(joined.Verts) || used[v] {
				return nil, true, fmt.Errorf("%w: section band repeats a ring vertex", ErrUnsupported)
			}
			used[v] = true
		}
		lowPos[caps.lowOuter[i]], highPos[caps.highOuter[i]] = i, i
	}
	bandGroup := ring.Groups[bandIndex]
	bandCells := make(map[[3]int]bool, 2*n)
	for i := range n {
		j := (i + 1) % n
		for _, tri := range [...][3]int{
			{rootRing[i], caps.lowOuter[i], caps.lowOuter[j]},
			{rootRing[i], caps.lowOuter[j], rootRing[j]},
		} {
			key := revolveSectorSortedTriangle(tri)
			if _, exists := bandCells[key]; exists {
				return nil, true, fmt.Errorf("%w: section band repeats a mesh cell", ErrUnsupported)
			}
			bandCells[key] = false
		}
	}
	maxRound, oldRound := 0.0, 0.0
	for i, angle := range ring.Angles {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
		if !ok {
			return nil, true, fmt.Errorf("%w: section ring angle has no enclosure", ErrUnsupported)
		}
		for _, sample := range [...]struct {
			rad   float64
			point r3.Vec
		}{
			{radius, r3.Vec{Y: inner[i].U, Z: inner[i].V}},
			{band.Start.V, joined.Verts[rootRing[i]]},
			{band.End.V, joined.Verts[caps.lowOuter[i]]},
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
		if joined.Verts[rootRing[i]].X != band.Start.U ||
			joined.Verts[caps.lowOuter[i]].X != band.End.U {
			return nil, true, fmt.Errorf("%w: section band changed its source stations", ErrUnsupported)
		}
	}
	sagFactor := proofbound.ProductUpper(maxStep, maxStep) / 8
	cutRound := proofbound.AbsSumUpper(maxRound, cross.round)
	boreBound = proofbound.AbsSumUpper(proofbound.ProductUpper(radius, sagFactor), cutRound)
	oldSideErr := proofbound.AbsSumUpper(proofbound.ProductUpper(band.Start.V, sagFactor), oldRound)
	newSideErr := proofbound.AbsSumUpper(
		proofbound.ProductUpper(max(band.Start.V, radius), sagFactor), oldRound, cutRound,
		proofbound.ProductUpper(cross.slopeHi, cross.round))
	sideErr := max(oldSideErr, newSideErr)
	bound := max(joined.MeshBound, boreBound, sideErr)
	if proofbound.IsNonFinite(bound) || bound > 0.1 || boreBound >= radius ||
		proofbound.AbsSumUpper(caps.lowRadius, joined.MeshBound, boreBound) >= radius ||
		proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound) >= band.Start.V {
		return nil, true, fmt.Errorf("%w: bore has no separate source bands", ErrUnsupported)
	}
	// Keep every other section span strictly outside the cylinder. This also
	// checks shoulders and additional meridian faces without assigning roles.
	clearance := proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound)
	for i, span := range section.Spans {
		if i == section.AxisSpan || i == caps.low || i == caps.high || i == bandIndex {
			continue
		}
		if min(span.Start.V, span.End.V) <= clearance {
			return nil, true, fmt.Errorf("%w: another section span may enter the bore", ErrUnsupported)
		}
	}
	lowSeen, highSeen := make([]bool, n), make([]bool, n)
	mesh := &joinedMesh{Verts: append([]r3.Vec(nil), joined.Verts...),
		Groups: append([]facetGroup(nil), joined.Groups...)}
	for i, tri := range joined.Tris {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		cell, fan, err := revolvemesh.SectorCapFan(tri, joined.Src[i], caps.lowAxis,
			caps.lowGroup, lowPos, n)
		if err != nil {
			return nil, true, err
		}
		if fan {
			if lowSeen[cell] {
				return nil, true, fmt.Errorf("%w: low cap repeats a cell", ErrUnsupported)
			}
			lowSeen[cell] = true
			continue
		}
		cell, fan, err = revolvemesh.SectorCapFan(tri, joined.Src[i], caps.highAxis,
			caps.highGroup, highPos, n)
		if err != nil {
			return nil, true, err
		}
		if fan {
			if highSeen[cell] {
				return nil, true, fmt.Errorf("%w: high cap repeats a cell", ErrUnsupported)
			}
			highSeen[cell] = true
			continue
		}
		if joined.Src[i] == bandGroup {
			key := revolveSectorSortedTriangle(tri)
			seen, expected := bandCells[key]
			if !expected || seen {
				return nil, true, fmt.Errorf("%w: section band has an unexpected facet", ErrUnsupported)
			}
			bandCells[key] = true
			continue
		}
		if !revolveSectorTriangleClear(tri, joined.Verts, clearance) {
			return nil, true, fmt.Errorf("%w: untouched face may enter the bore", ErrUnsupported)
		}
		mesh.Add(tri, joined.Src[i])
	}
	for i := range n {
		if !lowSeen[i] || !highSeen[i] {
			return nil, true, fmt.Errorf("%w: section cap misses an angular cell", ErrUnsupported)
		}
	}
	for _, seen := range bandCells {
		if !seen {
			return nil, true, fmt.Errorf("%w: section band misses an angular cell", ErrUnsupported)
		}
	}
	wallGroup := len(mesh.Groups)
	mesh.Groups = append(mesh.Groups, facetGroup{Origins: wall.originRefs(),
		Surface: wall.n.Surface, NormalBound: wall.n.NormalBound, Reversed: !wall.n.Reversed})
	vertexBound := append([]float64(nil), joined.VertexBound...)
	innerHigh := make([]int, n)
	for i := range n {
		cut := r3.Vec{X: cross.x, Y: inner[i].U, Z: inner[i].V}
		mesh.Verts[caps.lowOuter[i]] = cut
		vertexBound[caps.lowOuter[i]] = max(boreBound, newSideErr)
		high := r3.Vec{X: caps.highX, Y: inner[i].U, Z: inner[i].V}
		switch i {
		case 0:
			innerHigh[i] = caps.highAxis
		case 1:
			innerHigh[i] = caps.lowAxis
		default:
			innerHigh[i] = len(mesh.Verts)
			mesh.Verts = append(mesh.Verts, high)
			vertexBound = append(vertexBound, boreBound)
		}
		mesh.Verts[innerHigh[i]] = high
		vertexBound[innerHigh[i]] = boreBound
	}
	oldCount := len(mesh.Tris)
	for i := range n {
		j := (i + 1) % n
		mesh.Add([3]int{rootRing[i], caps.lowOuter[i], caps.lowOuter[j]}, bandGroup)
		mesh.Add([3]int{rootRing[i], caps.lowOuter[j], rootRing[j]}, bandGroup)
		mesh.Add([3]int{innerHigh[i], caps.highOuter[i], caps.highOuter[j]}, caps.highGroup)
		mesh.Add([3]int{innerHigh[i], caps.highOuter[j], innerHigh[j]}, caps.highGroup)
		mesh.Add([3]int{caps.lowOuter[i], innerHigh[i], innerHigh[j]}, wallGroup)
		mesh.Add([3]int{caps.lowOuter[i], innerHigh[j], caps.lowOuter[j]}, wallGroup)
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, true, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return nil, true, err
	}
	if len(mesh.Tris) != len(joined.Tris)+2*n {
		return nil, true, fmt.Errorf("%w: band and wall triangle census changed", ErrUnsupported)
	}
	patches := revolvemesh.SectorBorePatches(mesh.Src, oldCount, n, true, caps.lowGroup, caps.highGroup, bandGroup, wallGroup)
	if err := loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewWorkBudget(ctx),
		mesh.Verts, mesh.Tris, patches, revolveSectorBoreMaxAuditTriangles,
		revolveSectorBoreMaxAuditScans); err != nil {
		return nil, true, err
	}
	rootX := proofarith.FloatRat(band.Start.U)
	span := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(band.End.U), rootX))
	height := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(caps.highX), rootX))
	seamHalf := proofbound.DivUpper(sideErr, cross.slopeLo)
	if proofbound.IsNonFinite(span) || proofbound.IsNonFinite(height) ||
		proofbound.IsNonFinite(seamHalf) || height <= 0 {
		return nil, true, fmt.Errorf("%w: bore has no finite section span", ErrUnsupported)
	}
	sideShell := proofbound.ProductUpper(proofbound.ProductUpper(4, proofbound.TwoPiUpper()/2),
		proofbound.ProductUpper(proofbound.AbsSumUpper(band.Start.V, sideErr),
			proofbound.ProductUpper(sideErr, span)))
	seamRadius := proofbound.AbsSumUpper(radius, sideErr)
	seamSlab := proofbound.ProductUpper(proofbound.ProductUpper(2, seamHalf),
		proofbound.ProductUpper(proofbound.TwoPiUpper()/2,
			proofbound.ProductUpper(seamRadius, seamRadius)))
	circleShell := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(2, proofbound.TwoPiUpper()), radius),
		proofbound.ProductUpper(boreBound, height))
	volumeGap := proofbound.AbsSumUpper(joined.VolSymDiff, sideShell, seamSlab, circleShell)
	oldHeldArea, err := proofbound.PerturbedAreaUpperContext(ctx,
		joined.Verts, joined.Tris, joined.MeshBound)
	if err != nil {
		return nil, true, err
	}
	newHeldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return nil, true, err
	}
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.AreaSlack, oldHeldArea, newHeldArea, wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, true, fmt.Errorf("%w: bore has no finite measure proof", ErrUnsupported)
	}
	result := joined
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = vertexBound, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, areaSlack
	result.RevolveLoftJoin = nil
	result.ContactAudited = true
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), result))
	return body, true, err
}
