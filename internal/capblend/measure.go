package capblend

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ReadBandMass adapts the payload and built band to the recorded readings both
// mass integrals consume. This package owns the offset contour and the
// authenticated loop areas; internal/capband owns their flux composition.
func ReadBandMass(ctx context.Context, li int, loop sectionrecord.LoopRecord, cbp Payload,
	geom []capband.Patch, capZ, matSign, delta float64, closure capband.Closure) (capband.BandMassInput, error) {
	return readBandMassWithSideArea(ctx, li, loop, cbp, geom, capZ, matSign, delta, closure, nil)
}

// readBandMassWithSideArea reuses the area already computed with a loop's slab
// moments. Other callers pass nil and read the area from the loop as before.
func readBandMassWithSideArea(ctx context.Context, li int, loop sectionrecord.LoopRecord, cbp Payload,
	geom []capband.Patch, capZ, matSign, delta float64, closure capband.Closure,
	knownSideArea *proofbound.BoundedScalar) (capband.BandMassInput, error) {
	setback := cbp.SetbackAt(matSign)
	capZB := cbp.CapBandLevel(capZ, matSign)
	sideZB := proofbound.BoundedAdd(capZB,
		proofbound.MeasuredScalar(matSign*setback.Ds, setback.DsDelta))
	signedArea, err := sectionaudit.LoopSignedArea(proofbound.NewWorkBudget(ctx), loop)
	if err != nil {
		return capband.BandMassInput{}, err
	}
	orient := 1.0
	if signedArea < 0 {
		orient = -1
	}
	var sideArea proofbound.BoundedScalar
	if knownSideArea != nil {
		if err := ctx.Err(); err != nil {
			return capband.BandMassInput{}, err
		}
		sideArea = *knownSideArea
	} else {
		sideArea, err = momentinput.LoopEnclosedArea(ctx, loop)
		if err != nil {
			return capband.BandMassInput{}, err
		}
	}
	capBoundary, err := cbp.ContourOf(ctx, li, loop, setback.Dc)
	if err != nil {
		return capband.BandMassInput{}, err
	}
	capArea, err := momentinput.LoopEnclosedArea(ctx, capBoundary)
	if err != nil {
		return capband.BandMassInput{}, err
	}
	return capband.BandMassInput{
		Loop: loop, CapBoundary: capBoundary, Patches: geom,
		CapLevel: capZB, SideLevel: sideZB, SideArea: sideArea, CapArea: capArea,
		MaterialSign: matSign, Orientation: orient, Delta: delta,
		LevelDelta: capband.BandLevelDelta(capZ, matSign, setback.Ds, setback.DsDelta), Closure: closure,
	}, nil
}

// Bounds is the placed body's axis-aligned bounding box, read
// along each world axis from the payload's OWN patch extrema
// (extentBoundedAlong, Table DX row DX5) exactly as prismrecord.Bounds reads a
// prism's — so every face of the box is a value the body attains. Min and Max
// are positions and Bound is the absolute error on them (measurement.go), so a
// box widened outward by the setback d would be a Min sitting d millimetres
// from the true extreme while claiming an error of zero; §8.4 asks for bounds
// from patch extrema for that reason, and payload.go's ExtentAlong already
// states why padding the receiver prism by d proves nothing about attainment.
//
// The Bound is the reading's own, never a fixed zero. An extreme held by a
// recorded coordinate really does have none, which is the ordinary case on an
// unplaced body whose plane is axis aligned — a placement's own arithmetic can
// round it even so, and the reading charges that; an extreme held by the COMPUTED cap
// contour is known only to that contour's proven displacement, and publishing
// it as an Exact position would assert an accuracy the offset solve never had.
// extentBoundedAlong keeps the two apart per candidate, so a contour that loses
// the extremization contributes nothing here.
func Bounds(ctx context.Context, cbp Payload, work *freeform.FreeformWork) (measurement.Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	bound := 0.0
	for i, axis := range axes {
		if err := ctx.Err(); err != nil {
			return measurement.Box{}, err
		}
		lo, hi, axisBound, err := cbp.ExtentBoundedAlong(ctx, axis, work)
		if err != nil {
			return measurement.Box{}, err
		}
		minC[i], maxC[i] = lo, hi
		bound = math.Max(bound, axisBound)
	}
	return measurement.Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// widenLoftMeasurements adds the source loft's approximation allowances
// to the polygon band's global readings. The band's own area and geometry
// proofs describe its held-polygon offset, not an offset of the fit spline.
// Source allowances are extra bounds on comparison to the unchanged spline
// flanks and the loft from which that polygon was certified.
func widenLoftMeasurements(body *topo.Body, source loftmesh.MeshProof) error {
	if proofbound.IsNonFinite(source.FacetDeparture) || source.FacetDeparture < 0 ||
		proofbound.IsNonFinite(source.AreaSlack) || source.AreaSlack < 0 ||
		proofbound.IsNonFinite(source.VolSymDiff) || source.VolSymDiff < 0 {
		return fmt.Errorf("%w: loft cap band has no finite source proof", decaderr.ErrUnsupported)
	}
	volumeBound := proofbound.AbsSumUpper(body.Volume.Bound.Base(), source.VolSymDiff)
	body.Volume.Bound = units.CubicMillimeters(volumeBound)
	body.Volume.Exactness = measurement.ExactnessOf(volumeBound)
	areaBound := proofbound.AbsSumUpper(body.Area.Bound.Base(), source.AreaSlack)
	body.Area.Bound = units.SquareMillimeters(areaBound)
	body.Area.Exactness = measurement.ExactnessOf(areaBound)
	boxBound := proofbound.AbsSumUpper(body.Bounds.Bound.Base(), source.FacetDeparture)
	body.Bounds.Bound = units.Millimeters(boxBound)
	body.Bounds.Exactness = measurement.ExactnessOf(boxBound)
	dx := proofbound.BoundedSub(proofbound.ExactScalar(body.Bounds.Max.X), proofbound.ExactScalar(body.Bounds.Min.X))
	dy := proofbound.BoundedSub(proofbound.ExactScalar(body.Bounds.Max.Y), proofbound.ExactScalar(body.Bounds.Min.Y))
	dz := proofbound.BoundedSub(proofbound.ExactScalar(body.Bounds.Max.Z), proofbound.ExactScalar(body.Bounds.Min.Z))
	diameter := proofbound.Radius3D(math.Max(proofbound.AbsSumUpper(dx.Value, dx.Bound),
		math.Max(proofbound.AbsSumUpper(dy.Value, dy.Bound), proofbound.AbsSumUpper(dz.Value, dz.Bound))))
	floor := math.Nextafter(body.Volume.Value.Base()-volumeBound, math.Inf(-1))
	centroidBound := proofbound.AbsSumUpper(body.Centroid.Bound.Base(), source.FacetDeparture,
		facetproof.FacetedCentroidAllowance(source.VolSymDiff, diameter, floor))
	body.Centroid.Bound = units.Millimeters(centroidBound)
	body.Centroid.Exactness = measurement.ExactnessOf(centroidBound)
	return nil
}
