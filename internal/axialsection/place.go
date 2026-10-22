package axialsection

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/patternrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Place brings every placed operand onto the frame of the reference operand, the
// first loft that carries no placement of its own, so one private scene can hold
// all their sections. A placed operand's section is re-recorded through the exact
// in-plane map between its placed frame and the reference frame
// (patternrecord.MovePlaneRegion), and Delta is the upper bound on how far that
// record sits from the exact image. ok is false, with a nil error, when an operand
// cannot be placed exactly: its plane does not coincide with the reference plane,
// the reference frame is not exactly orthonormal, the map is not a similarity, its
// two end planes sit more than a few float steps from the reference operand's
// heights, or a segment kind has no such image. The distance an end plane does sit
// from the reference height is charged into Delta beside the section's own charge. Operands that carry no placement are returned as they are,
// so a set with none behaves as it did before placed operands were admitted.
func Place(ctx context.Context, ops []Operand) ([]Operand, bool, error) {
	ref := -1
	for i, op := range ops {
		if op.FromLoft && op.Xform == r3.Identity() {
			ref = i
			break
		}
	}
	if ref < 0 {
		for _, op := range ops {
			if op.Xform != r3.Identity() {
				return nil, false, nil
			}
		}
		return ops, true, nil
	}
	r := ops[ref]
	budget := proofbound.NewWorkBudget(ctx)
	out := make([]Operand, len(ops))
	for i, op := range ops {
		if i == ref {
			out[i] = op
			continue
		}
		onRef := op.Xform == r3.Identity() && sameFrame(op.Frame0, r.Frame0)
		var pm patternrecord.PlaneMap
		var region patternrecord.Region
		scale := 1.0
		if !onRef {
			var ok bool
			if pm, ok = patternrecord.NewPlaneMap(op.Frame0, op.Xform, r.Frame0, r.Xform); !ok {
				return nil, false, nil
			}
			region = patternrecord.Region{Outer: op.Section.Outer, Holes: op.Section.Holes}
			var err error
			if scale, err = patternrecord.PlacementScale(region, pm); err != nil {
				return nil, false, nil //nolint:nilerr // a section with no exact image is a miss, not a failure
			}
		}
		var gaps [2]float64
		for k, levels := range [2][2]*big.Rat{{op.Level0, r.Level0}, {op.Level1, r.Level1}} {
			got, lo, ok := patternrecord.PlaneHeight(op.Frame0, op.Xform, levels[0], r.Frame0, r.Xform)
			want, _, ok2 := patternrecord.PlaneHeight(r.Frame0, r.Xform, levels[1], r.Frame0, r.Xform)
			if !ok || !ok2 {
				return nil, false, nil
			}
			levelMag, _ := new(big.Rat).Abs(levels[0]).Float64()
			gaps[k] = patternrecord.HeightGap(got, want, lo)
			if gaps[k] > patternrecord.OffPlaneLimit*(scale+levelMag) {
				return nil, false, nil
			}
		}
		delta := 0.0
		if !onRef {
			moved, d, err := patternrecord.MovePlaneRegion(budget, region, pm)
			if err != nil {
				if cerr := ctx.Err(); cerr != nil {
					return nil, false, cerr
				}
				return nil, false, nil
			}
			op.Section = momentinput.Profile{Outer: moved.Outer, Holes: moved.Holes}
			delta = d
		}
		op.Delta = proofbound.AbsSumUpper(op.Delta, delta, gaps[0], gaps[1])
		op.Frame0, op.Frame1, op.Xform = r.Frame0, r.Frame1, r.Xform
		op.Plane0, op.Plane1, op.FromLoft = r.Plane0, r.Plane1, false
		op.Level0, op.Level1 = r.Level0, r.Level1
		out[i] = op
	}
	return out, true, nil
}

// sameFrame reports whether two frames have the same origin and axes bit for bit.
func sameFrame(a, b r3.Frame) bool {
	return a.Origin() == b.Origin() && a.U() == b.U() && a.V() == b.V()
}

// JoinDelta is how far the join of the operands' records can sit from the join
// of the solids they stand for: the largest per-operand section charge plus that
// operand's own lateral offset, plus the lateral offset of the reference operand,
// whose end frame the result keeps.
func JoinDelta(ops []Operand, ref int) float64 {
	worst := 0.0
	for _, op := range ops {
		worst = max(worst, proofbound.AbsSumUpper(op.Delta, op.Lateral))
	}
	return proofbound.AbsSumUpper(worst, ops[ref].Lateral)
}
