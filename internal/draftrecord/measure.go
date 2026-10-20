package draftrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// This file measures a draft body (agent-docs/draft-design.md §8). The body is one
// closed sub-solid per loop — the near disk, the far disk and the wall band
// between them — signed positive for the outer loop and negative for each
// hole, so every reading is the cap-loop band's own closed form
// (internal/capband/band_mass.go) summed over the loops with no
// straight slab. Each composes its bounds exactly as modify-reach §8.4
// states: the far contour's displacement once per band after the flux sum
// (proofbound.SweptVolumeAllow, proofbound.SweptMomentAllow), each patch's
// own area allowances inside capband.AreaOf, the closure slivers through
// capband.Closure, and the far cap's area through
// proofbound.SectionDisplacementArea.

// measureBody stamps the body's volume, area, centroid, bounds and its
// two cap faces' areas. bands are capblend.BuildBand's results in loop order.
func measureBody(ctx context.Context, body *topo.Body, dp Payload, cbp capblend.Payload, bands []capblend.BandResult, nearCap, farCap *topo.Face, work *freeform.FreeformWork) error {
	_, _, farZ, matSign := dp.Levels()
	var nearArea, farArea, patchArea, volume proofbound.BoundedScalar
	var mu, mv, mz proofbound.BoundedScalar
	for li, loop := range cbp.Loops() {
		if err := ctx.Err(); err != nil {
			return err
		}
		sign := 1.0
		if li != 0 {
			sign = -1
		}
		signed := func(v proofbound.BoundedScalar) proofbound.BoundedScalar {
			return proofbound.MeasuredScalar(sign*v.Value, v.Bound)
		}
		band := bands[li]

		mass, err := capblend.ReadBandMass(ctx, li, loop, cbp, band.Geom, farZ, matSign, band.Delta, band.Closure)
		if err != nil {
			return err
		}
		v, err := capband.BandVolume(mass, work)
		if err != nil {
			return err
		}
		volume = proofbound.BoundedAdd(volume, signed(v))
		if err := ctx.Err(); err != nil {
			return err
		}
		bmu, bmv, bmz, err := capband.BandMoment(mass, work)
		if err != nil {
			return err
		}
		mu = proofbound.BoundedAdd(mu, signed(bmu))
		mv = proofbound.BoundedAdd(mv, signed(bmv))
		mz = proofbound.BoundedAdd(mz, signed(bmz))
		for _, g := range band.Geom {
			pa, pb := capband.AreaOf(g)
			patchArea = proofbound.BoundedAdd(patchArea, proofbound.MeasuredScalar(pa, pb))
		}

		near, err := momentinput.LoopEnclosedArea(ctx, loop)
		if err != nil {
			return err
		}
		nearArea = proofbound.BoundedAdd(nearArea, signed(near))
		boundary, err := cbp.ContourOf(ctx, li, loop, dp.D)
		if err != nil {
			return err
		}
		far, err := momentinput.LoopEnclosedArea(ctx, boundary)
		if err != nil {
			return err
		}
		// The far disk is measured on the BUILT far section, which sits within
		// the band's contour displacement of the section the taper denotes:
		// that displacement moves the disk's area by at most the tube and
		// joint terms proofbound.SectionDisplacementArea states over the far
		// rim's own held perimeter.
		far.Bound = proofbound.AbsSumUpper(far.Bound,
			proofbound.SectionDisplacementArea(band.Delta, len(loop.Segments), capblend.ContourPerimeterUpper(band.CapCo)))
		farArea = proofbound.BoundedAdd(farArea, signed(far))
	}
	nearCap.Area, nearCap.AreaBound = nearArea.Value, nearArea.Bound
	farCap.Area, farCap.AreaBound = farArea.Value, farArea.Bound

	body.Volume = measurement.Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: measurement.ExactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}
	total := proofbound.BoundedAdd(proofbound.BoundedAdd(nearArea, farArea), patchArea)
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(total.Value),
		Exactness: measurement.ExactnessOf(total.Bound),
		Bound:     units.SquareMillimeters(total.Bound),
	}

	bounds, err := capblend.Bounds(ctx, cbp, work)
	if err != nil {
		return err
	}
	pl := prismrecord.Payload{Frame: dp.Frame, Xform: dp.Xform}
	cu := proofbound.BoundedQuotient(mu.Value, mu.Bound, volume.Value, volume.Bound)
	cv := proofbound.BoundedQuotient(mv.Value, mv.Bound, volume.Value, volume.Bound)
	cz := proofbound.BoundedQuotient(mz.Value, mz.Bound, volume.Value, volume.Bound)
	centroid := pl.Point(cu.Value, cv.Value, cz.Value)
	centroidBound := math.Min(pl.PointBound(cu, cv, cz), capband.CentroidGeometryBound(centroid, bounds))
	body.Centroid = measurement.VecMeasurement{
		Value:     centroid,
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	body.Bounds = bounds
	if err := prismrecord.ChargeMap(body, dp.Frame, dp.Xform); err != nil {
		return err
	}
	return topobuild.ValidateAnalyticMeasurements(body)
}
