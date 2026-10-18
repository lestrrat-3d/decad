package revolveshell

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

const (
	SectorBoreMaxAuditTriangles = 1_321_696
	SectorBoreMaxAuditScans     = uint64(400_000_000)
)

// BuildSectorBandCut rewrites one complete cap-adjacent straight band.
// The full band and both cap fans are counted before any facets are removed.
func BuildSectorBandCut(ctx context.Context, joined facetedrecord.Payload,
	caps SectorBoreCaps, radius float64, wall facetedrecord.Group) (facetedrecord.Payload, error) {
	record := joined.RevolveLoftJoin
	section, ring := record.Certificate.Section, record.CapRing
	n, count := len(ring.Angles), len(section.Spans)
	bandIndex := (caps.Low + count - 1) % count
	band := section.Spans[bandIndex]
	if bandIndex == section.AxisSpan || bandIndex == section.RootSpan ||
		band.End != section.Spans[caps.Low].Start ||
		band.Start.V <= radius || band.End.V >= radius ||
		band.Face == nil || len(band.Origins) == 0 ||
		len(ring.Rings[bandIndex]) != n ||
		ring.Groups[bandIndex] < 0 || ring.Groups[bandIndex] >= len(joined.Groups) ||
		len(joined.Tris) > SectorBoreMaxAuditTriangles-2*n {
		return facetedrecord.Payload{}, fmt.Errorf("%w: bore has no complete cap-adjacent straight band", decaderr.ErrUnsupported)
	}
	cross, ok := RevolveSectorStraightBandCross(band.Start, band.End, radius)
	if !ok || cross.X >= caps.HighX {
		return facetedrecord.Payload{}, fmt.Errorf("%w: bore does not cross one straight section band", decaderr.ErrUnsupported)
	}
	maxStep, ok := revolvemesh.SectorRingMaxStep(ring.Angles)
	if !ok {
		return facetedrecord.Payload{}, fmt.Errorf("%w: section ring has no bounded angular plan", decaderr.ErrUnsupported)
	}
	inner, boreBound, err := circularbounds.BoreRingAtStep(ctx, radius, ring.Angles, maxStep)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	if boreBound >= radius || proofbound.IsNonFinite(boreBound) {
		return facetedrecord.Payload{}, fmt.Errorf("%w: bore circle has no radial bound", decaderr.ErrUnsupported)
	}
	lowPos, highPos := make(map[int]int, n), make(map[int]int, n)
	used := map[int]bool{caps.LowAxis: true, caps.HighAxis: true}
	rootRing := ring.Rings[bandIndex]
	for i := range n {
		for _, v := range [...]int{caps.LowOuter[i], caps.HighOuter[i], rootRing[i]} {
			if v < 0 || v >= len(joined.Verts) || used[v] {
				return facetedrecord.Payload{}, fmt.Errorf("%w: section band repeats a ring vertex", decaderr.ErrUnsupported)
			}
			used[v] = true
		}
		lowPos[caps.LowOuter[i]], highPos[caps.HighOuter[i]] = i, i
	}
	bandGroup := ring.Groups[bandIndex]
	bandCells := make(map[[3]int]bool, 2*n)
	for i := range n {
		j := (i + 1) % n
		for _, tri := range [...][3]int{
			{rootRing[i], caps.LowOuter[i], caps.LowOuter[j]},
			{rootRing[i], caps.LowOuter[j], rootRing[j]},
		} {
			key := RevolveSectorSortedTriangle(tri)
			if _, exists := bandCells[key]; exists {
				return facetedrecord.Payload{}, fmt.Errorf("%w: section band repeats a mesh cell", decaderr.ErrUnsupported)
			}
			bandCells[key] = false
		}
	}
	maxRound, oldRound := 0.0, 0.0
	for i, angle := range ring.Angles {
		if err := ctx.Err(); err != nil {
			return facetedrecord.Payload{}, err
		}
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
		if !ok {
			return facetedrecord.Payload{}, fmt.Errorf("%w: section ring angle has no enclosure", decaderr.ErrUnsupported)
		}
		for _, sample := range [...]struct {
			rad   float64
			point r3.Vec
		}{
			{radius, r3.Vec{Y: inner[i].U, Z: inner[i].V}},
			{band.Start.V, joined.Verts[rootRing[i]]},
			{band.End.V, joined.Verts[caps.LowOuter[i]]},
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
			joined.Verts[caps.LowOuter[i]].X != band.End.U {
			return facetedrecord.Payload{}, fmt.Errorf("%w: section band changed its source stations", decaderr.ErrUnsupported)
		}
	}
	sagFactor := proofbound.ProductUpper(maxStep, maxStep) / 8
	cutRound := proofbound.AbsSumUpper(maxRound, cross.Round)
	boreBound = proofbound.AbsSumUpper(proofbound.ProductUpper(radius, sagFactor), cutRound)
	oldSideErr := proofbound.AbsSumUpper(proofbound.ProductUpper(band.Start.V, sagFactor), oldRound)
	newSideErr := proofbound.AbsSumUpper(
		proofbound.ProductUpper(max(band.Start.V, radius), sagFactor), oldRound, cutRound,
		proofbound.ProductUpper(cross.SlopeHi, cross.Round))
	sideErr := max(oldSideErr, newSideErr)
	bound := max(joined.MeshBound, boreBound, sideErr)
	if proofbound.IsNonFinite(bound) || bound > 0.1 || boreBound >= radius ||
		proofbound.AbsSumUpper(caps.LowRadius, joined.MeshBound, boreBound) >= radius ||
		proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound) >= band.Start.V {
		return facetedrecord.Payload{}, fmt.Errorf("%w: bore has no separate source bands", decaderr.ErrUnsupported)
	}
	// Keep every other section span strictly outside the cylinder. This also
	// checks shoulders and additional meridian faces without assigning roles.
	clearance := proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound)
	for i, span := range section.Spans {
		if i == section.AxisSpan || i == caps.Low || i == caps.High || i == bandIndex {
			continue
		}
		if min(span.Start.V, span.End.V) <= clearance {
			return facetedrecord.Payload{}, fmt.Errorf("%w: another section span may enter the bore", decaderr.ErrUnsupported)
		}
	}
	lowSeen, highSeen := make([]bool, n), make([]bool, n)
	mesh := &facetedrecord.JoinedMesh{Verts: append([]r3.Vec(nil), joined.Verts...),
		Groups: append([]facetedrecord.Group(nil), joined.Groups...)}
	for i, tri := range joined.Tris {
		if err := ctx.Err(); err != nil {
			return facetedrecord.Payload{}, err
		}
		cell, fan, err := revolvemesh.SectorCapFan(tri, joined.Src[i], caps.LowAxis,
			caps.LowGroup, lowPos, n)
		if err != nil {
			return facetedrecord.Payload{}, err
		}
		if fan {
			if lowSeen[cell] {
				return facetedrecord.Payload{}, fmt.Errorf("%w: low cap repeats a cell", decaderr.ErrUnsupported)
			}
			lowSeen[cell] = true
			continue
		}
		cell, fan, err = revolvemesh.SectorCapFan(tri, joined.Src[i], caps.HighAxis,
			caps.HighGroup, highPos, n)
		if err != nil {
			return facetedrecord.Payload{}, err
		}
		if fan {
			if highSeen[cell] {
				return facetedrecord.Payload{}, fmt.Errorf("%w: high cap repeats a cell", decaderr.ErrUnsupported)
			}
			highSeen[cell] = true
			continue
		}
		if joined.Src[i] == bandGroup {
			key := RevolveSectorSortedTriangle(tri)
			seen, expected := bandCells[key]
			if !expected || seen {
				return facetedrecord.Payload{}, fmt.Errorf("%w: section band has an unexpected facet", decaderr.ErrUnsupported)
			}
			bandCells[key] = true
			continue
		}
		if !RevolveSectorTriangleClear(tri, joined.Verts, clearance) {
			return facetedrecord.Payload{}, fmt.Errorf("%w: untouched face may enter the bore", decaderr.ErrUnsupported)
		}
		mesh.Add(tri, joined.Src[i])
	}
	for i := range n {
		if !lowSeen[i] || !highSeen[i] {
			return facetedrecord.Payload{}, fmt.Errorf("%w: section cap misses an angular cell", decaderr.ErrUnsupported)
		}
	}
	for _, seen := range bandCells {
		if !seen {
			return facetedrecord.Payload{}, fmt.Errorf("%w: section band misses an angular cell", decaderr.ErrUnsupported)
		}
	}
	wallGroup := len(mesh.Groups)
	mesh.Groups = append(mesh.Groups, wall)
	vertexBound := append([]float64(nil), joined.VertexBound...)
	innerHigh := make([]int, n)
	for i := range n {
		cut := r3.Vec{X: cross.X, Y: inner[i].U, Z: inner[i].V}
		mesh.Verts[caps.LowOuter[i]] = cut
		vertexBound[caps.LowOuter[i]] = max(boreBound, newSideErr)
		high := r3.Vec{X: caps.HighX, Y: inner[i].U, Z: inner[i].V}
		switch i {
		case 0:
			innerHigh[i] = caps.HighAxis
		case 1:
			innerHigh[i] = caps.LowAxis
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
		mesh.Add([3]int{rootRing[i], caps.LowOuter[i], caps.LowOuter[j]}, bandGroup)
		mesh.Add([3]int{rootRing[i], caps.LowOuter[j], rootRing[j]}, bandGroup)
		mesh.Add([3]int{innerHigh[i], caps.HighOuter[i], caps.HighOuter[j]}, caps.HighGroup)
		mesh.Add([3]int{innerHigh[i], caps.HighOuter[j], innerHigh[j]}, caps.HighGroup)
		mesh.Add([3]int{caps.LowOuter[i], innerHigh[i], innerHigh[j]}, wallGroup)
		mesh.Add([3]int{caps.LowOuter[i], innerHigh[j], caps.LowOuter[j]}, wallGroup)
	}
	if err := tessellation.OrientJoinedMesh(mesh.Verts, mesh.Tris); err != nil {
		return facetedrecord.Payload{}, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return facetedrecord.Payload{}, err
	}
	if len(mesh.Tris) != len(joined.Tris)+2*n {
		return facetedrecord.Payload{}, fmt.Errorf("%w: band and wall triangle census changed", decaderr.ErrUnsupported)
	}
	patches := revolvemesh.SectorBorePatches(mesh.Src, oldCount, n, true, caps.LowGroup, caps.HighGroup, bandGroup, wallGroup)
	if err := loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewWorkBudget(ctx),
		mesh.Verts, mesh.Tris, patches, SectorBoreMaxAuditTriangles,
		SectorBoreMaxAuditScans); err != nil {
		return facetedrecord.Payload{}, err
	}
	rootX := proofarith.FloatRat(band.Start.U)
	span := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(band.End.U), rootX))
	height := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(caps.HighX), rootX))
	seamHalf := proofbound.DivUpper(sideErr, cross.SlopeLo)
	if proofbound.IsNonFinite(span) || proofbound.IsNonFinite(height) ||
		proofbound.IsNonFinite(seamHalf) || height <= 0 {
		return facetedrecord.Payload{}, fmt.Errorf("%w: bore has no finite section span", decaderr.ErrUnsupported)
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
		return facetedrecord.Payload{}, err
	}
	newHeldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.AreaSlack, oldHeldArea, newHeldArea, wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return facetedrecord.Payload{}, fmt.Errorf("%w: bore has no finite measure proof", decaderr.ErrUnsupported)
	}
	result := joined
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = vertexBound, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, areaSlack
	result.RevolveLoftJoin = nil
	result.ContactAudited = true
	return result, nil
}
