package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// loftCapBandSource retains the fit-spline section and its already certified
// approximation. The modifier acts on the loft's held cap edges, so its new
// band is polygonal while the original spline remains available as the flank
// source and its displacement and mass allowances remain attached.
type loftCapBandSource struct {
	profile profileRecord
	proof   loftmesh.MeshProof
}

// tryLoftCapChamfer is the matching axial loft's complete outer-cap loop
// route. Its cap contour follows the loft's certified held station polygon.
func tryLoftCapChamfer(ctx context.Context, b *Body, pl loftPayload, edges []*Edge,
	d, dDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	if asym != nil || pl.surfaceResult || pl.xform != r3.Identity() ||
		!sectionrecord.IdenticalRecord(pl.profile0, pl.profile1) ||
		len(pl.profile0.Holes) != 1 || len(pl.profile0.Holes[0].Segments) != 1 ||
		pl.frame0.U() != pl.frame1.U() || pl.frame0.V() != pl.frame1.V() {
		return nil, false, nil
	}
	if _, ok := pl.profile0.Holes[0].Segments[0].(circleSeg); !ok {
		return nil, false, nil
	}
	for _, offset := range pl.alignment {
		if offset != 0 {
			return nil, false, nil
		}
	}
	end := pl.frame0.ToLocal(pl.frame1.Origin())
	if end.X != 0 || end.Y != 0 || end.Z <= 2*d {
		return nil, false, nil
	}
	var caps [2]*Face
	for _, face := range b.Faces() {
		for _, origin := range face.origins {
			if origin == CapStart(b) {
				caps[0] = face
			}
			if origin == CapEnd(b) {
				caps[1] = face
			}
		}
	}
	if caps[0] == nil || caps[1] == nil {
		return nil, false, nil
	}
	if !matchingLoftHeldCapOuters(pl.frame0, caps[0], caps[1]) {
		return nil, false, nil
	}
	selected := make(map[*Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edge] = struct{}{}
	}
	for _, cap := range caps {
		if len(cap.loops) == 0 || !cap.loops[0].outer {
			return nil, false, nil
		}
		for _, edge := range cap.loops[0].Edges() {
			if _, ok := selected[edge]; !ok {
				return nil, false, nil
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return nil, false, nil
	}
	outer := make([]curveSegment, 0, len(caps[1].loops[0].coedges))
	for _, ce := range caps[1].loops[0].coedges {
		start := pl.frame0.ToLocal(ce.Start().position)
		stop := pl.frame0.ToLocal(ce.End().position)
		outer = append(outer, lineSeg{
			Start:  sectionrecord.Point2{U: start.X, V: start.Y},
			End:    sectionrecord.Point2{U: stop.X, V: stop.Y},
			TStart: 0, TEnd: 1,
		})
	}
	profile := profileRecord{
		Outer: loopRecord{Segments: outer},
		Holes: pl.profile0.Holes,
	}
	pp := prismPayload{profile: profile, frame: pl.frame0, z0: 0, z1: end.Z, xform: r3.Identity()}
	setback := capSetback{dc: d, dcDelta: dDelta, ds: d, dsDelta: dDelta}
	source := &loftCapBandSource{profile: pl.profile0, proof: pl.proof}
	result, err := buildCapBlendWithLoftSource(ctx, b.doc, b.doc.nextProducerID(), pp, setback, setback,
		map[int]bool{0: true}, map[int]bool{0: true}, source)
	if err != nil {
		return nil, true, fmt.Errorf("loft cap chamfer: %w", err)
	}
	return result, true, nil
}

// matchingLoftHeldCapOuters requires one shared held polygon at both axial
// levels. This is stronger than equal input records: asymmetric station
// alignment or a different floating-point cap sample must not be silently
// replaced by the other end's polygon when the cap band is built.
func matchingLoftHeldCapOuters(frame r3.Frame, start, end *Face) bool {
	if len(start.loops) == 0 || len(end.loops) == 0 ||
		!start.loops[0].outer || !end.loops[0].outer ||
		len(start.loops[0].coedges) != len(end.loops[0].coedges) {
		return false
	}
	keyOf := func(ce coedge) [2]Point2 {
		a, b := frame.ToLocal(ce.Start().position), frame.ToLocal(ce.End().position)
		p := Point2{U: a.X, V: a.Y}
		q := Point2{U: b.X, V: b.Y}
		if p.U > q.U || (p.U == q.U && p.V > q.V) {
			p, q = q, p
		}
		return [2]Point2{p, q}
	}
	counts := make(map[[2]Point2]int, len(start.loops[0].coedges))
	for _, ce := range start.loops[0].coedges {
		counts[keyOf(ce)]++
	}
	for _, ce := range end.loops[0].coedges {
		key := keyOf(ce)
		if counts[key] == 0 {
			return false
		}
		counts[key]--
	}
	return true
}

// widenLoftCapBandMeasurements adds the source loft's approximation allowances
// to the polygon band's global readings. The band's own area and geometry
// proofs describe its held-polygon offset, not an offset of the fit spline.
// Source allowances are extra bounds on comparison to the unchanged spline
// flanks and the loft from which that polygon was certified.
func widenLoftCapBandMeasurements(body *Body, source loftmesh.MeshProof) error {
	if proofbound.IsNonFinite(source.FacetDeparture) || source.FacetDeparture < 0 ||
		proofbound.IsNonFinite(source.AreaSlack) || source.AreaSlack < 0 ||
		proofbound.IsNonFinite(source.VolSymDiff) || source.VolSymDiff < 0 {
		return fmt.Errorf("%w: loft cap band has no finite source proof", ErrUnsupported)
	}
	volumeBound := proofbound.AbsSumUpper(body.volume.Bound.Base(), source.VolSymDiff)
	body.volume.Bound = units.CubicMillimeters(volumeBound)
	body.volume.Exactness = exactnessOf(volumeBound)
	areaBound := proofbound.AbsSumUpper(body.area.Bound.Base(), source.AreaSlack)
	body.area.Bound = units.SquareMillimeters(areaBound)
	body.area.Exactness = exactnessOf(areaBound)
	boxBound := proofbound.AbsSumUpper(body.bounds.Bound.Base(), source.FacetDeparture)
	body.bounds.Bound = units.Millimeters(boxBound)
	body.bounds.Exactness = exactnessOf(boxBound)
	dx := proofbound.BoundedSub(proofbound.ExactScalar(body.bounds.Max.X), proofbound.ExactScalar(body.bounds.Min.X))
	dy := proofbound.BoundedSub(proofbound.ExactScalar(body.bounds.Max.Y), proofbound.ExactScalar(body.bounds.Min.Y))
	dz := proofbound.BoundedSub(proofbound.ExactScalar(body.bounds.Max.Z), proofbound.ExactScalar(body.bounds.Min.Z))
	diameter := proofbound.Radius3D(math.Max(proofbound.AbsSumUpper(dx.Value, dx.Bound),
		math.Max(proofbound.AbsSumUpper(dy.Value, dy.Bound), proofbound.AbsSumUpper(dz.Value, dz.Bound))))
	floor := math.Nextafter(body.volume.Value.Base()-volumeBound, math.Inf(-1))
	centroidBound := proofbound.AbsSumUpper(body.centroid.Bound.Base(), source.FacetDeparture,
		facetedCentroidAllowance(source.VolSymDiff, diameter, floor))
	body.centroid.Bound = units.Millimeters(centroidBound)
	body.centroid.Exactness = exactnessOf(centroidBound)
	return nil
}
