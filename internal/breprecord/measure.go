package breprecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file measures a Payload (docs/general-boolean-design.md §4.3): every
// reading is a sum over faces of a closed form in that face's own record. The
// divergence-theorem sums run in exact rational intervals in the reference
// frame (Embeds): a line contributes an exact rational, a circular wall or
// region its proven enclosure, and the published float is the sum rounded
// once. The terms may cancel in value, never in bound — every enclosure's
// width is carried through the sum.

// measureBody adapts the body's face records to brepgeom.IntegrateFaces
// and publishes its readings, topology bounds and final validation.
func measureBody(ctx context.Context, bp Payload, btopo *Topology, body *topo.Body, bands bandMass) error {
	restored, err := bp.FilletRestored(btopo)
	if err != nil {
		return err
	}
	readings, err := brepgeom.IntegrateFaces(ctx, len(bp.Faces), func(fi int) (brepgeom.FaceMeasure, error) {
		f, rf := bp.Faces[fi], restored[fi]
		record := brepgeom.FaceMeasure{
			Embed: btopo.Embeds[fi], Planar: f.Planar(), Outward: f.Outward,
			Z0: f.Z0, Z1: f.Z1, RestoredZ0: rf.Z0, RestoredZ1: rf.Z1,
			Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta, Delta: f.Delta,
		}
		if !record.Planar {
			record.Wall, record.Walk = f.Wall, btopo.Walls[fi]
			return record, nil
		}
		region, err := btopo.Region(ctx, bp, fi)
		if err != nil {
			return brepgeom.FaceMeasure{}, err
		}
		record.Region, record.RestoredRegion = region, region
		if rf.Region != f.Region {
			record.RestoredRegion, err = brepgeom.RestoredRegion(ctx, rf.Region, rf.Delta)
		}
		return record, err
	}, brepgeom.FaceMass{Vol3: bands.vol3, Moments: bands.moments, Area: bands.area},
		btopo.CoordUpper, bp.SectionDelta(), bp.AxialDelta())
	if err != nil {
		return err
	}
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(readings.Volume.Value),
		Exactness: measurement.ExactnessOf(readings.Volume.Bound), Bound: units.CubicMillimeters(readings.Volume.Bound)}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(readings.Area.Value),
		Exactness: measurement.ExactnessOf(readings.Area.Bound), Bound: units.SquareMillimeters(readings.Area.Bound)}
	refView := bp.RefView()
	c := readings.Centroid
	centroidBound := refView.PointBound(c[0], c[1], c[2])
	body.Centroid = measurement.VecMeasurement{Value: refView.Point(c[0].Value, c[1].Value, c[2].Value),
		Exactness: measurement.ExactnessOf(centroidBound), Bound: units.Millimeters(centroidBound)}

	box, err := Bounds(ctx, bp)
	if err != nil {
		return err
	}
	body.Bounds = box
	return topobuild.ValidateAnalyticMeasurements(body)
}

// Bounds is the union of every face's own box (§4.3): each face's
// prism view read along the three world axes, composed with the largest
// section and level displacement as prismrecord.Bounds composes a prism's. A
// fillet band's patches bulge past their directrices along an oblique axis,
// so each adds its own extents (filletBandExtent).
func Bounds(ctx context.Context, bp Payload) (measurement.Box, error) {
	minC, maxC, bound, err := brepgeom.Bounds(ctx, len(bp.Faces)+len(bp.LoopBands), bp.ExtentAt,
		bp.SectionDelta(), bp.AxialDelta())
	if err != nil {
		return measurement.Box{}, err
	}
	return measurement.Box{
		Min:       minC,
		Max:       maxC,
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// extentAt adapts one face or band to the shared BRep extent reader.
func (bp Payload) ExtentAt(ctx context.Context, item int, axis r3.Vec,
	work *freeform.FreeformWork) (float64, float64, float64, bool, error) {
	if item < len(bp.Faces) {
		lo, hi, bound, err := bp.Faces[item].View(bp.Xform).ExtentBoundedAlong(ctx, axis, work, nil)
		return lo, hi, bound, true, err
	}
	band := bp.LoopBands[item-len(bp.Faces)]
	if band.Kind != BandFillet {
		return 0, 0, 0, false, nil
	}
	lo, hi, bound, err := filletBandExtent(ctx, band, bp.Faces[band.Face], bp.Xform, axis, work)
	return lo, hi, bound, true, err
}

// ExtentAlong is the through-all stop's reading (root stops.go): the union of every
// face's own extent along g, and every fillet band's (filletBandExtent,
// docs/loop-fillet-design.md §5.4), beside the largest of their bounds. Like
// a prism's, it refuses a record carrying a section displacement, which moves
// a coordinate the interval is stated over.
func (bp Payload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	if bp.SectionDelta() != 0 {
		return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a brep body with a proven section displacement`, decaderr.ErrUnsupported)
	}
	return brepgeom.ExtentAlong(context.Background(), len(bp.Faces)+len(bp.LoopBands), g, bp.ExtentAt)
}
