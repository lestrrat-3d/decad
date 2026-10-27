package prismplacement

import (
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// Operand holds only the stored placement and sweep fields used by the
// shared-axis and relative-placement calculations. Z0Delta and Z1Delta bound
// how far each held level sits from the level it denotes, and Z0Denoted and
// Z1Denoted record that level where the producer knows it. The G5 interval
// relations read all three; an operand that leaves them zero states two
// undisplaced levels.
type Operand struct {
	Frame                r3.Frame
	Xform                r3.Transform
	Z0, Z1               float64
	Z0Delta, Z1Delta     float64
	Z0Denoted, Z1Denoted LevelDenotation
}

// SharedAxis is G3's exact shared-axis result. Shift is the rational axial
// displacement of B's origin from A's.
type SharedAxis struct {
	OK    bool
	Shift *big.Rat
}

// SharedAxisOf decides G3's shared-axis arms over the stored floats taken
// exactly. Equal transform bases and equal section axes make the section
// coordinates identical; only an exact displacement along the common sweep
// axis may change the recorded interval.
func SharedAxisOf(pa, pb Operand) SharedAxis {
	if pa.Xform.Basis() != pb.Xform.Basis() || pa.Frame.U() != pb.Frame.U() || pa.Frame.V() != pb.Frame.V() {
		return SharedAxis{}
	}
	oa, ob, n := pa.Frame.Origin(), pb.Frame.Origin(), pa.Frame.N()
	basis := pa.Xform.Basis()
	if !proofbound.FiniteVec(oa) || !proofbound.FiniteVec(ob) || !proofbound.FiniteVec(n) ||
		!proofbound.FiniteVec(pa.Xform.Translation()) || !proofbound.FiniteVec(pb.Xform.Translation()) ||
		!proofbound.FiniteVec(basis.EX) || !proofbound.FiniteVec(basis.EY) || !proofbound.FiniteVec(basis.EZ) {
		return SharedAxis{}
	}
	d := proofarith.DvSub(proofarith.DvTransform(pb.Xform, proofarith.DyVec(ob)),
		proofarith.DvTransform(pa.Xform, proofarith.DyVec(oa)))
	nd := proofarith.DvSub(proofarith.DvTransform(pa.Xform, proofarith.DyVec(n)),
		proofarith.DyVec(pa.Xform.Translation()))
	if !proofarith.DvIsZero(proofarith.DvCross(d, nd)) {
		return SharedAxis{}
	}
	// Any nonzero axis component gives s = d_i/axis_i. Choose the
	// largest magnitude without rounding the exact transformed axis.
	i := 0
	for j := 1; j < len(nd); j++ {
		if proofarith.DyCmp(proofarith.DyAbs(nd[j]), proofarith.DyAbs(nd[i])) > 0 {
			i = j
		}
	}
	if nd[i].Sign() == 0 {
		return SharedAxis{}
	}
	return SharedAxis{OK: true, Shift: new(big.Rat).Quo(d[i].Rat(), nd[i].Rat())}
}

// ZShift is G5's exact shift, or zero for G3's coplanar arm.
func ZShift(pa, pb Operand) *big.Rat {
	if sa := SharedAxisOf(pa, pb); sa.OK {
		return sa.Shift
	}
	return new(big.Rat)
}

// ShiftedInterval lifts B's sweep levels exactly before applying G5.
func ShiftedInterval(pa, pb Operand) (*big.Rat, *big.Rat, bool) {
	b0, b1 := proofarith.FloatRat(pb.Z0), proofarith.FloatRat(pb.Z1)
	if b0 == nil || b1 == nil {
		return nil, nil, false
	}
	shift := ZShift(pa, pb)
	return b0.Add(b0, shift), b1.Add(b1, shift), true
}

// AdmittedShiftedInterval reads the exact interval after G5 has admitted the
// pair. A failed lift here violates that gate's contract.
func AdmittedShiftedInterval(pa, pb Operand) (*big.Rat, *big.Rat) {
	z0, z1, ok := ShiftedInterval(pa, pb)
	if !ok {
		panic("decad: G5 admitted a prism pair whose sweep interval does not lift exactly")
	}
	return z0, z1
}

// IntersectLowerEnd chooses the higher of A's lower level and B's exact
// shifted lower level. The chosen endpoint carries its own axial displacement.
// A tie takes A's float with the larger displacement because both operands'
// coordinates then equally denote the result. A shifted B endpoint rounds
// once, and RationalFloatError charges that rounding beside B's displacement.
func IntersectLowerEnd(aVal, aDelta float64, bVal *big.Rat, bDelta float64) (float64, float64) {
	return intersectEnd(aVal, aDelta, bVal, bDelta, true)
}

// IntersectUpperEnd chooses the lower of A's upper level and B's exact
// shifted upper level, charging B's rounding when B supplies the result.
func IntersectUpperEnd(aVal, aDelta float64, bVal *big.Rat, bDelta float64) (float64, float64) {
	return intersectEnd(aVal, aDelta, bVal, bDelta, false)
}

func intersectEnd(aVal, aDelta float64, bVal *big.Rat, bDelta float64, lower bool) (float64, float64) {
	cmp := proofarith.FloatRat(aVal).Cmp(bVal)
	switch {
	case cmp == 0:
		return aVal, max(aDelta, bDelta)
	case lower && cmp > 0 || !lower && cmp < 0:
		return aVal, aDelta
	}
	held, _ := bVal.Float64()
	return held, proofbound.AbsSumUpper(bDelta, proofarith.RationalFloatError(bVal, held))
}

// levelRange is the exact range a level's true value lies in, lifted by
// shift: the single rational DenotedLevel proves, or else the held float
// widened by its displacement on both sides. ok is false for a non-finite
// level or displacement, or a negative displacement.
type levelRange struct {
	lo, hi *big.Rat
}

func rangeOf(held, delta float64, d LevelDenotation, shift *big.Rat) (levelRange, bool) {
	if exact := DenotedLevel(held, delta, d); exact != nil {
		exact.Add(exact, shift)
		return levelRange{lo: exact, hi: exact}, true
	}
	h, dr := proofarith.FloatRat(held), proofarith.FloatRat(delta)
	if h == nil || dr == nil || dr.Sign() < 0 {
		return levelRange{}, false
	}
	h.Add(h, shift)
	return levelRange{lo: new(big.Rat).Sub(h, dr), hi: new(big.Rat).Add(h, dr)}, true
}

// zRanges is the exact range of each sweep level of op on A's axis.
func zRanges(op Operand, shift *big.Rat) (levelRange, levelRange, bool) {
	r0, ok0 := rangeOf(op.Z0, op.Z0Delta, op.Z0Denoted, shift)
	r1, ok1 := rangeOf(op.Z1, op.Z1Delta, op.Z1Denoted, shift)
	return r0, r1, ok0 && ok1
}

// operandRanges is A's two level ranges and B's after G5's exact shift.
func operandRanges(pa, pb Operand) (a0, a1, b0, b1 levelRange, ok bool) {
	a0, a1, okA := zRanges(pa, new(big.Rat))
	b0, b1, okB := zRanges(pb, ZShift(pa, pb))
	return a0, a1, b0, b1, okA && okB
}

// CutZIntervalSpans reports that the shifted tool interval provably spans the
// target's. Each comparison holds for every true level the held floats and
// displacements allow: the tool's lower level lies at or below the target's
// whatever their displacements, or both levels denote rationals in that order,
// and likewise above. A cap meeting the target endpoint is a valid span when
// both denote one rational. Equal held floats within a displacement prove
// nothing, so such a pair declines.
func CutZIntervalSpans(target, tool Operand) bool {
	t0, t1, z0, z1, ok := operandRanges(target, tool)
	return ok && z0.hi.Cmp(t0.lo) <= 0 && z1.lo.Cmp(t1.hi) >= 0
}

// UnionZIntervalMatches reports that the shifted B interval provably equals
// A's: the held floats agree exactly after the shift, and each pair of levels
// denotes one rational. A displaced level without a denotation proves no
// equality.
func UnionZIntervalMatches(pa, pb Operand) bool {
	a0, a1 := proofarith.FloatRat(pa.Z0), proofarith.FloatRat(pa.Z1)
	z0, z1, ok := ShiftedInterval(pa, pb)
	if a0 == nil || a1 == nil || !ok || a0.Cmp(z0) != 0 || a1.Cmp(z1) != 0 {
		return false
	}
	r0, r1, s0, s1, ok := operandRanges(pa, pb)
	return ok && r0.lo.Cmp(r0.hi) == 0 && r0.lo.Cmp(s0.lo) == 0 && s0.lo.Cmp(s0.hi) == 0 &&
		r1.lo.Cmp(r1.hi) == 0 && r1.lo.Cmp(s1.lo) == 0 && s1.lo.Cmp(s1.hi) == 0
}

// IntersectZIntervalOverlaps reports that the shifted B interval provably
// overlaps A's: each lower level lies strictly below the other's upper level
// for every true level the displacements allow. A pair touching only at a
// cap, or whose caps meet within their displacements, has no proven overlap.
func IntersectZIntervalOverlaps(pa, pb Operand) bool {
	a0, a1, z0, z1, ok := operandRanges(pa, pb)
	return ok && a0.hi.Cmp(z1.lo) < 0 && z0.hi.Cmp(a1.lo) < 0
}

// Relative is the composed map from B's plane-local coordinates into A's.
type Relative struct {
	Map        r3.Transform
	Identity   bool
	Reflection bool
	TransAbs   float64
}

// Compose builds the relative map in the same frame and transform order as
// the prism boolean gate. Equal placements and shared axes are exact identity.
func Compose(pa, pb Operand) (Relative, error) {
	if (pa.Frame == pb.Frame && pa.Xform == pb.Xform) || SharedAxisOf(pa, pb).OK {
		return Relative{Identity: true}, nil
	}
	fail := func(err error) (Relative, error) {
		return Relative{}, fmt.Errorf(`decad: the operands' relative placement has no rigid composition: %w`, err)
	}
	m, err := r3.FromFrame(pb.Frame)
	if err != nil {
		return fail(err)
	}
	if m, err = m.Then(pb.Xform); err != nil {
		return fail(err)
	}
	invA, err := pa.Xform.Inverse()
	if err != nil {
		return fail(err)
	}
	if m, err = m.Then(invA); err != nil {
		return fail(err)
	}
	toWorldA, err := r3.FromFrame(pa.Frame)
	if err != nil {
		return fail(err)
	}
	invFrameA, err := toWorldA.Inverse()
	if err != nil {
		return fail(err)
	}
	if m, err = m.Then(invFrameA); err != nil {
		return fail(err)
	}
	return Relative{
		Map:        m,
		Reflection: m.IsReflection(),
		TransAbs:   proofbound.VecMaxAbs(m.Translation()),
	}, nil
}

// Point maps one plane-local point and increases the placement-rounding
// charge. The caller owns the running maximum across its mapped points.
func Point(re Relative, delta *float64, p sectionrecord.Point2) sectionrecord.Point2 {
	if re.Identity {
		return p
	}
	out := re.Map.Apply(r3.NewVec(p.U, p.V, 0))
	*delta = math.Max(*delta, proofbound.RigidRoundAllow(max(math.Abs(p.U), math.Abs(p.V)), re.TransAbs))
	return sectionrecord.Point2{U: out.X, V: out.Y}
}
