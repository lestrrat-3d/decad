package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// closedRacetrack admits the circular tube of sweep design §3.4 from exact
// recorded coordinates. A close sampled path or section cannot pass this gate.
func closedRacetrack(profile profileRecord, plane planeRecord, path *Path) bool {
	if plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) ||
		plane.V != r3.NewVec(0, 1, 0) || path.Start() != (r3.Vec{}) ||
		len(path.records) != 4 || len(profile.Holes) != 0 || len(profile.Outer.Segments) != 1 {
		return false
	}
	circle, ok := profile.Outer.Segments[0].(circleSeg)
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
	firstLine, upper, lastLine, lower := path.records[0], path.records[1], path.records[2], path.records[3]
	if firstLine.Arc != nil || upper.Arc == nil || lastLine.Arc != nil || lower.Arc == nil ||
		!closedSweepHalfTurn(upper) || !closedSweepHalfTurn(lower) {
		return false
	}
	height := sweeparc.VecOf(firstLine.End)[2]
	major := upper.Arc.Center[0]
	if height.Sign() <= 0 || major.Sign() <= 0 ||
		major.Cmp(r) <= 0 || height.Cmp(new(big.Rat).Mul(r, big.NewRat(2, 1))) <= 0 {
		return false
	}
	twiceMajor := new(big.Rat).Mul(major, big.NewRat(2, 1))
	return closedSweepRatVec(firstLine.End, new(big.Rat), new(big.Rat), height) &&
		closedSweepRatVec(upper.End, twiceMajor, new(big.Rat), height) &&
		closedSweepRatVec(lastLine.End, twiceMajor, new(big.Rat), new(big.Rat)) &&
		closedSweepRatVec(lower.End, new(big.Rat), new(big.Rat), new(big.Rat)) &&
		closedSweepCarrier(upper.Arc, major, height) &&
		closedSweepCarrier(lower.Arc, major, new(big.Rat))
}

// closedPlanarTangentLoop admits a cyclic tube only when its recorded arc
// turns compose one exact rotation about the common Y axis. Transport and
// the global contact audit supply the remaining frame and separation proofs.
func closedPlanarTangentLoop(profile profileRecord, plane planeRecord, path *Path) bool {
	if plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) ||
		plane.V != r3.NewVec(0, 1, 0) || path.Start() != (r3.Vec{}) ||
		len(path.records) < 5 || len(profile.Holes) != 0 || len(profile.Outer.Segments) != 1 {
		return false
	}
	circle, ok := profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center.U != 0 || circle.Center.V != 0 ||
		!circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return false
	}
	turns := new(big.Rat)
	arcCount := 0
	for _, record := range path.records {
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

func closedSweepHalfTurn(record sweepinput.PathRecord) bool {
	angle := record.ArcAngle
	return angle.Turn != nil && angle.Turn.Cmp(big.NewRat(1, 2)) == 0 &&
		angle.Rad != nil && angle.Rad.Sign() == 0 && angle.Span == nil
}

func closedSweepRatVec(point r3.Vec, x, y, z *big.Rat) bool {
	actual := sweeparc.VecOf(point)
	return actual[0].Cmp(x) == 0 && actual[1].Cmp(y) == 0 && actual[2].Cmp(z) == 0
}

func closedSweepCarrier(arc *sweeparc.Record, x, z *big.Rat) bool {
	return arc.Center[0].Cmp(x) == 0 && arc.Center[1].Sign() == 0 && arc.Center[2].Cmp(z) == 0 &&
		arc.Axis[0].Sign() == 0 && arc.Axis[1].Sign() > 0 && arc.Axis[2].Sign() == 0
}

func evalClosedCyclicSweep(ctx context.Context, d *Document, ref producerID,
	profile profileRecord, plane planeRecord, frame r3.Frame, path *Path,
	work *freeform.FreeformWork,
) (*Body, error) {
	if !closedRacetrack(profile, plane, path) && !closedPlanarTangentLoop(profile, plane, path) {
		return nil, fmt.Errorf(`%w: the closed sweep path is outside the admitted cyclic classes`, ErrUnsupported)
	}
	return evalCompositeSweepContextWithClosure(ctx, d, ref, profile, plane, frame, path, work, false, true)
}
