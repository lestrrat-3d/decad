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
	placed := false
	for _, op := range ops {
		placed = placed || op.Xform != r3.Identity()
	}
	if !placed {
		return ops, true, nil
	}
	ref := -1
	for i, op := range ops {
		if op.FromLoft && op.Xform == r3.Identity() {
			ref = i
			break
		}
	}
	if ref < 0 {
		return nil, false, nil
	}
	r := ops[ref]
	heights := func(op Operand) (h0, h1 *big.Rat, ok bool) {
		h0, ok0 := patternrecord.PlaneHeight(op.Frame0, op.Xform, op.Level0, r.Frame0, r3.Identity())
		h1, ok1 := patternrecord.PlaneHeight(op.Frame0, op.Xform, op.Level1, r.Frame0, r3.Identity())
		return h0, h1, ok0 && ok1
	}
	r0, r1, ok := heights(r)
	if !ok {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	out := make([]Operand, len(ops))
	for i, op := range ops {
		if op.Xform == r3.Identity() {
			out[i] = op
			continue
		}
		pm, ok := patternrecord.NewPlaneMap(op.Frame0, op.Xform, r.Frame0, r3.Identity())
		if !ok {
			return nil, false, nil
		}
		h0, h1, ok := heights(op)
		if !ok {
			return nil, false, nil
		}
		lift0, ok0 := heightGap(h0, r0, r.Level0)
		lift1, ok1 := heightGap(h1, r1, r.Level1)
		if !ok0 || !ok1 {
			return nil, false, nil
		}
		moved, delta, err := patternrecord.MovePlaneRegion(budget,
			patternrecord.Region{Outer: op.Section.Outer, Holes: op.Section.Holes}, pm)
		if err != nil {
			if cerr := ctx.Err(); cerr != nil {
				return nil, false, cerr
			}
			return nil, false, nil
		}
		op.Section = momentinput.Profile{Outer: moved.Outer, Holes: moved.Holes}
		op.Delta = proofbound.AbsSumUpper(op.Delta, delta, lift0, lift1)
		op.Frame0, op.Frame1, op.Xform = r.Frame0, r.Frame1, r3.Identity()
		op.Plane0, op.Plane1, op.FromLoft = r.Plane0, r.Plane1, false
		op.Level0, op.Level1 = r.Level0, r.Level1
		out[i] = op
	}
	return out, true, nil
}

// heightGapLimit bounds, relative to the end plane's offset from the frame
// origin plus one millimetre, how far a placed end plane may sit from the
// reference operand's. A placement matrix is exact to a few float steps, which
// is below this by a wide margin, so a larger gap names a different sweep.
// The limit only refuses: the gap an operand passes is charged in full.
const heightGapLimit = 0x1p-46

// heightGap is the exact distance between a placed end plane's height and the
// reference's, rounded up. ok is false when it exceeds heightGapLimit times the
// reference offset plus one.
func heightGap(got, want, level *big.Rat) (float64, bool) {
	gap := new(big.Rat).Abs(new(big.Rat).Sub(got, want))
	scale := new(big.Rat).Add(new(big.Rat).Abs(level), big.NewRat(1, 1))
	up := proofbound.RatFloatUp(gap)
	return up, up <= heightGapLimit*proofbound.RatFloatUp(scale)
}

// MaxDelta is the largest section displacement among the operands.
func MaxDelta(ops []Operand) float64 {
	worst := 0.0
	for _, op := range ops {
		worst = max(worst, op.Delta)
	}
	return worst
}
