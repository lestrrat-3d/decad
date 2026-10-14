package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
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
		len(pl.profile0.Holes) > 1 ||
		pl.frame0.U() != pl.frame1.U() || pl.frame0.V() != pl.frame1.V() {
		return nil, false, nil
	}
	if len(pl.profile0.Holes) == 1 {
		if len(pl.profile0.Holes[0].Segments) != 1 {
			return nil, false, nil
		}
		if _, ok := pl.profile0.Holes[0].Segments[0].(circleSeg); !ok {
			return nil, false, nil
		}
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
		for _, origin := range face.n.Origins {
			if origin == refOf(CapStart(b)) {
				caps[0] = face
			}
			if origin == refOf(CapEnd(b)) {
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
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edgeNode(edge)] = struct{}{}
	}
	for _, cap := range caps {
		if len(cap.n.Loops) == 0 || !cap.n.Loops[0].Outer {
			return nil, false, nil
		}
		for _, ce := range cap.n.Loops[0].CoEdges {
			edge := ce.Edge
			if _, ok := selected[edge]; !ok {
				return nil, false, nil
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return nil, false, nil
	}
	outer := make([]curveSegment, 0, len(caps[1].n.Loops[0].CoEdges))
	for _, ce := range caps[1].n.Loops[0].CoEdges {
		start := pl.frame0.ToLocal(coedgeStart(ce).Position)
		stop := pl.frame0.ToLocal(coedgeEnd(ce).Position)
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
	pp := prismrecord.Payload{Profile: profile, Frame: pl.frame0, Z0: 0, Z1: end.Z, Xform: r3.Identity()}
	setback := capSetback{dc: d, dcDelta: dDelta, ds: d, dsDelta: dDelta}
	source := &loftCapBandSource{profile: pl.profile0, proof: pl.proof}
	result, err := buildCapBlendWithLoftSource(ctx, documentHandle(b.n.Doc), b.n.Doc.NextProducer, pp, setback, setback,
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
	if len(start.n.Loops) == 0 || len(end.n.Loops) == 0 ||
		!start.n.Loops[0].Outer || !end.n.Loops[0].Outer ||
		len(start.n.Loops[0].CoEdges) != len(end.n.Loops[0].CoEdges) {
		return false
	}
	keyOf := func(ce topo.CoEdge) [2]planePoint {
		a, b := frame.ToLocal(coedgeStart(ce).Position), frame.ToLocal(coedgeEnd(ce).Position)
		p := planePoint{U: a.X, V: a.Y}
		q := planePoint{U: b.X, V: b.Y}
		if p.U > q.U || (p.U == q.U && p.V > q.V) {
			p, q = q, p
		}
		return [2]planePoint{p, q}
	}
	counts := make(map[[2]planePoint]int, len(start.n.Loops[0].CoEdges))
	for _, ce := range start.n.Loops[0].CoEdges {
		counts[keyOf(ce)]++
	}
	for _, ce := range end.n.Loops[0].CoEdges {
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
	volumeBound := proofbound.AbsSumUpper(body.n.Volume.Bound.Base(), source.VolSymDiff)
	body.n.Volume.Bound = units.CubicMillimeters(volumeBound)
	body.n.Volume.Exactness = measurement.ExactnessOf(volumeBound)
	areaBound := proofbound.AbsSumUpper(body.n.Area.Bound.Base(), source.AreaSlack)
	body.n.Area.Bound = units.SquareMillimeters(areaBound)
	body.n.Area.Exactness = measurement.ExactnessOf(areaBound)
	boxBound := proofbound.AbsSumUpper(body.n.Bounds.Bound.Base(), source.FacetDeparture)
	body.n.Bounds.Bound = units.Millimeters(boxBound)
	body.n.Bounds.Exactness = measurement.ExactnessOf(boxBound)
	dx := proofbound.BoundedSub(proofbound.ExactScalar(body.n.Bounds.Max.X), proofbound.ExactScalar(body.n.Bounds.Min.X))
	dy := proofbound.BoundedSub(proofbound.ExactScalar(body.n.Bounds.Max.Y), proofbound.ExactScalar(body.n.Bounds.Min.Y))
	dz := proofbound.BoundedSub(proofbound.ExactScalar(body.n.Bounds.Max.Z), proofbound.ExactScalar(body.n.Bounds.Min.Z))
	diameter := proofbound.Radius3D(math.Max(proofbound.AbsSumUpper(dx.Value, dx.Bound),
		math.Max(proofbound.AbsSumUpper(dy.Value, dy.Bound), proofbound.AbsSumUpper(dz.Value, dz.Bound))))
	floor := math.Nextafter(body.n.Volume.Value.Base()-volumeBound, math.Inf(-1))
	centroidBound := proofbound.AbsSumUpper(body.n.Centroid.Bound.Base(), source.FacetDeparture,
		facetedCentroidAllowance(source.VolSymDiff, diameter, floor))
	body.n.Centroid.Bound = units.Millimeters(centroidBound)
	body.n.Centroid.Exactness = measurement.ExactnessOf(centroidBound)
	return nil
}
