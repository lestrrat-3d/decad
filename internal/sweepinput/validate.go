// Package sweepinput validates recorded sections and paths before Sweep builds a body.
package sweepinput

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/r3"
)

// Tangents are the exact incoming and outgoing directions of one path span.
type Tangents struct {
	In, Out sweeparc.RatVec
}

// ValidateStart admits a path start on the recorded plane with a positive
// normal tangent. The caller supplies at least one recorded span.
func ValidateStart(start r3.Vec, plane sectionrecord.PlaneRecord, tangent sweeparc.RatVec) error {
	normal := sweeparc.FromDyadic(proofarith.DvCross(proofarith.DyVec(plane.U), proofarith.DyVec(plane.V)))
	relStart := sweeparc.Sub(sweeparc.VecOf(start), sweeparc.VecOf(plane.Origin))
	if sweeparc.Dot(relStart, normal).Sign() != 0 {
		return fmt.Errorf(`%w: the sweep path must start in the profile plane`, decaderr.ErrDegenerate)
	}
	if !sweeparc.IsZero(sweeparc.Cross(tangent, normal)) || sweeparc.Dot(tangent, normal).Sign() <= 0 {
		return fmt.Errorf(`%w: the sweep path's initial tangent must follow the profile plane's positive normal`, decaderr.ErrDegenerate)
	}
	return nil
}

// ValidateJoins admits only exactly codirectional internal tangent rays.
func ValidateJoins(tangents []Tangents) error {
	for i := 1; i < len(tangents); i++ {
		previousOut, tangentIn := tangents[i-1].Out, tangents[i].In
		if !sweeparc.IsZero(sweeparc.Cross(previousOut, tangentIn)) || sweeparc.Dot(previousOut, tangentIn).Sign() <= 0 {
			return fmt.Errorf(`%w: sweep path join %d is not tangent; WithMitredJoins admits a corner on a LineTo-only path`, decaderr.ErrUnsupported, i)
		}
	}
	return nil
}

// StraightSpan reads one admitted line's height and the error of its held
// normal displacement from the recorded exact tangent.
func StraightSpan(start, end, normal r3.Vec) (float64, float64, error) {
	tangent := proofarith.DvSub(proofarith.DyVec(end), proofarith.DyVec(start))
	delta := end.Sub(start)
	if !proofbound.FiniteVec(delta) {
		return 0, 0, fmt.Errorf(`%w: the sweep line's derived displacement is outside the representable range`, decaderr.ErrUnsupported)
	}
	height := delta.Len()
	if math.IsInf(height, 0) || math.IsNaN(height) {
		return 0, 0, fmt.Errorf(`%w: the sweep line's length is outside the representable range`, decaderr.ErrUnsupported)
	}
	// The squared length is the recorded tangent's own exact dot product.
	lengthBound := capcontour.StraightEdgeBound(height, proofarith.DvDot(tangent, tangent), true)
	heldSweep := normal.Scale(height)
	bound := proofbound.AbsSumUpper(
		lengthBound,
		proofarith.DyadicFloatError(tangent[0], heldSweep.X),
		proofarith.DyadicFloatError(tangent[1], heldSweep.Y),
		proofarith.DyadicFloatError(tangent[2], heldSweep.Z),
	)
	if math.IsInf(bound, 0) || math.IsNaN(bound) {
		return 0, 0, fmt.Errorf(`%w: the sweep line's length has no finite error bound`, decaderr.ErrUnsupported)
	}
	return height, bound, nil
}

// ValidateAnalyticProfile admits only recorded section kinds the span builders read.
func ValidateAnalyticProfile(profile momentinput.Profile) error {
	loops := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		if err := ValidateAnalyticSegments(loop.Segments, "profile"); err != nil {
			return err
		}
	}
	return nil
}

// ValidateAnalyticSegments applies Sweep's segment-kind gate to one profile
// or chain walk. kind names the source argument in its refusal.
func ValidateAnalyticSegments(segments []sectionrecord.CurveSegment, kind string) error {
	for _, raw := range segments {
		segment, err := sectionrecord.NormalizeSegment(raw)
		if err != nil {
			return err
		}
		switch segment.(type) {
		case sectionrecord.LineSeg, sectionrecord.CircleSeg, sectionrecord.ArcSeg:
		default:
			return fmt.Errorf(`%w: Sweep supports line, circle, and arc %s segments only`, decaderr.ErrUnsupported, kind)
		}
	}
	return nil
}
