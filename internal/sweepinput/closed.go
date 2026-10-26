package sweepinput

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/r3"
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

// ClosedCardinalLoop admits a cyclic sweep path from exact recorded data.
// Every arc must be an exact quarter or half turn about an axis parallel to one
// common coordinate axis, and the whole path must lie in the one plane
// perpendicular to that axis that holds the first record. The signed turns must sum to exactly one
// whole turn. Any refusal wraps decaderr.ErrUnsupported and names the failed
// gate. The caller still transports the section frame, which must return to
// its start with zero displacement, and the global contact audit decides
// whether the result is simple. A close sampled path cannot pass this gate.
func ClosedCardinalLoop(records []PathRecord) error {
	if len(records) < 4 {
		return closedLoopRefusal("the path has %d spans, fewer than four", len(records))
	}
	axisIndex := -1
	var plane *big.Rat
	turns := new(big.Rat)
	arcCount := 0
	for i, record := range records {
		if record.Arc != nil {
			arcCount++
			index, sign, ok := cardinalAxis(record.Arc.Axis)
			if !ok {
				return closedLoopRefusal("span %d is an arc about a non-coordinate axis", i)
			}
			if axisIndex >= 0 && index != axisIndex {
				return closedLoopRefusal("span %d turns about a different coordinate axis", i)
			}
			axisIndex = index
			angle := record.ArcAngle
			if angle.Turn == nil || angle.Rad == nil || angle.Rad.Sign() != 0 || angle.Span != nil ||
				(angle.Turn.Cmp(big.NewRat(1, 4)) != 0 && angle.Turn.Cmp(big.NewRat(1, 2)) != 0) {
				return closedLoopRefusal("span %d is not an exact quarter or half turn", i)
			}
			turns.Add(turns, new(big.Rat).Mul(angle.Turn, big.NewRat(int64(sign), 1)))
		}
	}
	if arcCount < 2 || axisIndex < 0 {
		return closedLoopRefusal("the path has %d arcs, fewer than two", arcCount)
	}
	for i, record := range records {
		start, end := sweeparc.VecOf(record.Start), sweeparc.VecOf(record.End)
		if plane == nil {
			plane = start[axisIndex]
		}
		if start[axisIndex].Cmp(plane) != 0 || end[axisIndex].Cmp(plane) != 0 ||
			(record.Arc != nil && record.Arc.Center[axisIndex].Cmp(plane) != 0) {
			return closedLoopRefusal("span %d leaves the plane of the loop", i)
		}
	}
	if new(big.Rat).Abs(turns).Cmp(big.NewRat(1, 1)) != 0 {
		return closedLoopRefusal("the signed arc turns sum to %s, not one whole turn", turns.RatString())
	}
	return nil
}

func closedLoopRefusal(format string, args ...any) error {
	return fmt.Errorf("%w: the closed sweep path is outside the admitted cardinal loop class: "+format,
		append([]any{decaderr.ErrUnsupported}, args...)...)
}

// cardinalAxis reports the index and sign of an axis vector with exactly one
// nonzero component.
func cardinalAxis(axis sweeparc.RatVec) (int, int, bool) {
	index := -1
	for i, component := range axis {
		if component.Sign() == 0 {
			continue
		}
		if index >= 0 {
			return 0, 0, false
		}
		index = i
	}
	if index < 0 {
		return 0, 0, false
	}
	return index, axis[index].Sign(), true
}

func samePoint(a, b r3.Vec) bool {
	return a.X == b.X && a.Y == b.Y && a.Z == b.Z
}
