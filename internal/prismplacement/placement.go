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
// shared-axis and relative-placement calculations.
type Operand struct {
	Frame  r3.Frame
	Xform  r3.Transform
	Z0, Z1 float64
}

// SharedAxis is G3's exact shared-axis result. Shift is the rational axial
// displacement of B's origin from A's.
type SharedAxis struct {
	OK    bool
	Shift *big.Rat
}

// SharedAxisOf decides G3's shared-axis arm over the stored floats taken
// exactly. No float arithmetic is performed in its cross-product or shift.
func SharedAxisOf(pa, pb Operand) SharedAxis {
	if pa.Xform != pb.Xform || pa.Frame.U() != pb.Frame.U() || pa.Frame.V() != pb.Frame.V() {
		return SharedAxis{}
	}
	oa, ob, n := pa.Frame.Origin(), pb.Frame.Origin(), pa.Frame.N()
	if !proofbound.FiniteVec(oa) || !proofbound.FiniteVec(ob) || !proofbound.FiniteVec(n) {
		return SharedAxis{}
	}
	d := proofarith.DvSub(proofarith.DyVec(ob), proofarith.DyVec(oa))
	nd := proofarith.DyVec(n)
	if !proofarith.DvIsZero(proofarith.DvCross(d, nd)) {
		return SharedAxis{}
	}
	// Any nonzero N_i gives s = d_i/N_i. Choose the largest |N_i|.
	comps := [3]float64{n.X, n.Y, n.Z}
	i := 0
	for j := 1; j < len(comps); j++ {
		if math.Abs(comps[j]) > math.Abs(comps[i]) {
			i = j
		}
	}
	if comps[i] == 0 {
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

// CutZIntervalSpans compares the shifted tool interval against both target
// endpoints exactly. A cap meeting the target endpoint is a valid span.
func CutZIntervalSpans(target, tool Operand) bool {
	t0, t1 := proofarith.FloatRat(target.Z0), proofarith.FloatRat(target.Z1)
	z0, z1, ok := ShiftedInterval(target, tool)
	if t0 == nil || t1 == nil || !ok {
		return false
	}
	return z0.Cmp(t0) <= 0 && z1.Cmp(t1) >= 0
}

// UnionZIntervalMatches compares the exact shifted B interval to A's.
func UnionZIntervalMatches(pa, pb Operand) bool {
	a0, a1 := proofarith.FloatRat(pa.Z0), proofarith.FloatRat(pa.Z1)
	z0, z1, ok := ShiftedInterval(pa, pb)
	if a0 == nil || a1 == nil || !ok {
		return false
	}
	return a0.Cmp(z0) == 0 && a1.Cmp(z1) == 0
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
