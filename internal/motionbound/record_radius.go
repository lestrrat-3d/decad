package motionbound

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

func recordVecL1(v r3.Vec) float64 {
	return proofbound.AbsSumUpper(v.X, v.Y, v.Z)
}

// LinearRecordRadius bounds every record point of a straight sweep before
// placement. coordinateUpper includes the section displacement, and zUpper
// includes the axial displacement.
func LinearRecordRadius(frame r3.Frame, coordinateUpper, zUpper float64) float64 {
	return proofbound.AbsSumUpper(
		recordVecL1(frame.Origin()),
		proofbound.ProductUpper(recordVecL1(frame.U()), coordinateUpper),
		proofbound.ProductUpper(recordVecL1(frame.V()), coordinateUpper),
		proofbound.ProductUpper(recordVecL1(frame.N()), zUpper),
	)
}

// RevolveRecordRadius bounds a record point swept around the recorded axis.
// The axis coordinates and their bounds come from the resolved axis record.
func RevolveRecordRadius(frame r3.Frame, coordinateUpper, axisU, axisUBound, axisV, axisVBound float64) float64 {
	originUpper := recordVecL1(frame.Origin())
	profileUpper := proofbound.AbsSumUpper(
		originUpper,
		proofbound.ProductUpper(recordVecL1(frame.U()), coordinateUpper),
		proofbound.ProductUpper(recordVecL1(frame.V()), coordinateUpper),
	)
	axisUpper := proofbound.AbsSumUpper(
		originUpper,
		proofbound.ProductUpper(recordVecL1(frame.U()), proofbound.AbsSumUpper(axisU, axisUBound)),
		proofbound.ProductUpper(recordVecL1(frame.V()), proofbound.AbsSumUpper(axisV, axisVBound)),
	)
	return proofbound.AbsSumUpper(proofbound.ProductUpper(3, profileUpper), proofbound.ProductUpper(4, axisUpper))
}

// DraftSectionCoordinateUpper bounds max(|u|, |v|) over a recorded section.
// It takes the smallest proven bound from each segment's walk and carrier: a
// line from its ends, an arc from its extent, and a circle from its center,
// radius, and radius bound.
func DraftSectionCoordinateUpper(profile momentinput.Profile) (float64, error) {
	work := freeform.NewFreeformWork()
	upper := 0.0
	for _, loop := range append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...) {
		for _, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return 0, err
			}
			segUpper := w.CoordUpper
			if local, ok := capband.SegmentCoordinateUpper(seg); ok {
				segUpper = math.Min(segUpper, local)
			}
			if w.IsCircular() && !proofbound.IsNonFinite(w.RadiusBound) {
				segUpper = math.Min(segUpper, proofbound.AbsSumUpper(math.Max(math.Abs(w.CU), math.Abs(w.CV)), w.Radius, w.RadiusBound))
			}
			upper = math.Max(upper, segUpper)
		}
	}
	return upper, nil
}
