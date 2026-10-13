package sweepinput

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ClosedCircle recognizes the two complementary arcs of one exact
// directed circle. The recorded spans already prove that each arc is
// nondegenerate and shares its endpoint with the next arc. Matching exact
// centers and codirectional axes make their angles sum to one turn; matching
// the first and final tangents proves the section frame closes at the seam.
func ClosedCircle(records []PathRecord) (*sweeparc.Record, bool) {
	if len(records) != 2 || records[0].Arc == nil || records[1].Arc == nil {
		return nil, false
	}
	first, second := records[0], records[1]
	if !samePoint(first.Start, second.End) || !samePoint(first.End, second.Start) {
		return nil, false
	}
	for i := range first.Arc.Center {
		if first.Arc.Center[i].Cmp(second.Arc.Center[i]) != 0 {
			return nil, false
		}
	}
	if !sweeparc.IsZero(sweeparc.Cross(first.Arc.Axis, second.Arc.Axis)) ||
		sweeparc.Dot(first.Arc.Axis, second.Arc.Axis).Sign() <= 0 {
		return nil, false
	}
	if !sweeparc.IsZero(sweeparc.Cross(second.TangentOut, first.TangentIn)) ||
		sweeparc.Dot(second.TangentOut, first.TangentIn).Sign() <= 0 {
		return nil, false
	}
	return first.Arc, true
}

// ClosedRacetrack admits the circular tube of sweep design §3.4 from exact
// recorded coordinates. A close sampled path or section cannot pass this gate.
func ClosedRacetrack(
	profile momentinput.Profile, plane sectionrecord.PlaneRecord, start r3.Vec, records []PathRecord,
) bool {
	if plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) ||
		plane.V != r3.NewVec(0, 1, 0) || start != (r3.Vec{}) ||
		len(records) != 4 || len(profile.Holes) != 0 || len(profile.Outer.Segments) != 1 {
		return false
	}
	circle, ok := profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || circle.Center.U != 0 || circle.Center.V != 0 ||
		!circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return false
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 {
		return false
	}
	r := new(big.Rat).SetFloat64(radius)
	if r == nil {
		return false
	}
	firstLine, upper, lastLine, lower := records[0], records[1], records[2], records[3]
	if firstLine.Arc != nil || upper.Arc == nil || lastLine.Arc != nil || lower.Arc == nil ||
		!halfTurn(upper) || !halfTurn(lower) {
		return false
	}
	height := sweeparc.VecOf(firstLine.End)[2]
	major := upper.Arc.Center[0]
	if height.Sign() <= 0 || major.Sign() <= 0 ||
		major.Cmp(r) <= 0 || height.Cmp(new(big.Rat).Mul(r, big.NewRat(2, 1))) <= 0 {
		return false
	}
	twiceMajor := new(big.Rat).Mul(major, big.NewRat(2, 1))
	return ratVecIs(firstLine.End, new(big.Rat), new(big.Rat), height) &&
		ratVecIs(upper.End, twiceMajor, new(big.Rat), height) &&
		ratVecIs(lastLine.End, twiceMajor, new(big.Rat), new(big.Rat)) &&
		ratVecIs(lower.End, new(big.Rat), new(big.Rat), new(big.Rat)) &&
		carrierIs(upper.Arc, major, height) &&
		carrierIs(lower.Arc, major, new(big.Rat))
}

// ClosedPlanarTangentLoop admits a cyclic tube only when its recorded arc
// turns compose one exact rotation about the common Y axis. Transport and
// the global contact audit supply the remaining frame and separation proofs.
func ClosedPlanarTangentLoop(
	profile momentinput.Profile, plane sectionrecord.PlaneRecord, start r3.Vec, records []PathRecord,
) bool {
	if plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) ||
		plane.V != r3.NewVec(0, 1, 0) || start != (r3.Vec{}) ||
		len(records) < 5 || len(profile.Holes) != 0 || len(profile.Outer.Segments) != 1 {
		return false
	}
	circle, ok := profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || circle.Center.U != 0 || circle.Center.V != 0 ||
		!circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return false
	}
	turns := new(big.Rat)
	arcCount := 0
	for _, record := range records {
		if record.Start.Y != 0 || record.End.Y != 0 {
			return false
		}
		if record.Arc == nil {
			continue
		}
		arcCount++
		arc := record.Arc
		angle := record.ArcAngle
		if arc.Center[1].Sign() != 0 || arc.Axis[0].Sign() != 0 ||
			arc.Axis[1].Sign() <= 0 || arc.Axis[2].Sign() != 0 ||
			angle.Turn == nil || angle.Rad == nil || angle.Rad.Sign() != 0 || angle.Span != nil ||
			(angle.Turn.Cmp(big.NewRat(1, 4)) != 0 && angle.Turn.Cmp(big.NewRat(1, 2)) != 0) {
			return false
		}
		turns.Add(turns, angle.Turn)
	}
	return arcCount >= 2 && turns.Cmp(big.NewRat(1, 1)) == 0
}

func samePoint(a, b r3.Vec) bool {
	return a.X == b.X && a.Y == b.Y && a.Z == b.Z
}

func halfTurn(record PathRecord) bool {
	angle := record.ArcAngle
	return angle.Turn != nil && angle.Turn.Cmp(big.NewRat(1, 2)) == 0 &&
		angle.Rad != nil && angle.Rad.Sign() == 0 && angle.Span == nil
}

func ratVecIs(point r3.Vec, x, y, z *big.Rat) bool {
	actual := sweeparc.VecOf(point)
	return actual[0].Cmp(x) == 0 && actual[1].Cmp(y) == 0 && actual[2].Cmp(z) == 0
}

func carrierIs(arc *sweeparc.Record, x, z *big.Rat) bool {
	return arc.Center[0].Cmp(x) == 0 && arc.Center[1].Sign() == 0 && arc.Center[2].Cmp(z) == 0 &&
		arc.Axis[0].Sign() == 0 && arc.Axis[1].Sign() > 0 && arc.Axis[2].Sign() == 0
}
