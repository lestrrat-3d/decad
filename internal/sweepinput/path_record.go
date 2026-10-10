package sweepinput

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/r3"
)

// Segment is one owned public path segment expressed without public CAD types.
type Segment struct {
	End, Through r3.Vec
	Arc          bool
}

// PathRecord holds exact derived geometry of one recorded path segment.
type PathRecord struct {
	Start, End            r3.Vec
	TangentIn, TangentOut sweeparc.RatVec
	Arc                   *sweeparc.Record
	ArcPhi                float64
	ArcAngle              revolveangle.Angle
}

// RecordSegment derives one segment after the root has copied its public
// input. The caller records each segment in input order, preserving refusals.
func RecordSegment(start r3.Vec, segment Segment, index int) (PathRecord, error) {
	if !segment.Arc {
		if !proofbound.FiniteVec(segment.End) {
			return PathRecord{}, fmt.Errorf(`%w: path segment %d has a non-finite endpoint`, decaderr.ErrNotFinite, index)
		}
		if segment.End == start {
			return PathRecord{}, fmt.Errorf(`%w: path segment %d is a zero-length line`, decaderr.ErrDegenerate, index)
		}
		tangent := sweeparc.Sub(sweeparc.VecOf(segment.End), sweeparc.VecOf(start))
		return PathRecord{
			Start: start, End: segment.End,
			TangentIn: tangent, TangentOut: tangent,
		}, nil
	}
	if !proofbound.FiniteVec(segment.Through) || !proofbound.FiniteVec(segment.End) {
		return PathRecord{}, fmt.Errorf(`%w: path segment %d has a non-finite point`, decaderr.ErrNotFinite, index)
	}
	if segment.Through == start || segment.End == start || segment.Through == segment.End {
		return PathRecord{}, fmt.Errorf(`%w: path segment %d repeats an arc point`, decaderr.ErrDegenerate, index)
	}
	fromStart := proofarith.DvSub(proofarith.DyVec(segment.Through), proofarith.DyVec(start))
	toEnd := proofarith.DvSub(proofarith.DyVec(segment.End), proofarith.DyVec(start))
	if proofarith.DvIsZero(proofarith.DvCross(fromStart, toEnd)) {
		return PathRecord{}, fmt.Errorf(`%w: path segment %d has collinear arc points`, decaderr.ErrDegenerate, index)
	}
	carrier, err := sweeparc.RecordArc(start, segment.Through, segment.End)
	if err != nil {
		return PathRecord{}, fmt.Errorf(`path segment %d: %w`, index, err)
	}
	phi, angle, err := sweeparc.ArcAngle(carrier.RadiusStart, carrier.RadiusEnd, carrier.Axis)
	if err != nil {
		return PathRecord{}, fmt.Errorf(`path segment %d: %w`, index, err)
	}
	for _, coordinate := range carrier.Center {
		if _, _, ok := sweeparc.Held(coordinate); !ok {
			return PathRecord{}, fmt.Errorf(`%w: path segment %d has an unrepresentable circular carrier`, decaderr.ErrUnsupported, index)
		}
	}
	return PathRecord{
		Start: start, End: segment.End,
		TangentIn:  sweeparc.Cross(carrier.Axis, carrier.RadiusStart),
		TangentOut: sweeparc.Cross(carrier.Axis, carrier.RadiusEnd),
		Arc:        &carrier,
		ArcPhi:     phi,
		ArcAngle:   angle,
	}, nil
}
