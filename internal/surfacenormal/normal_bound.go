package surfacenormal

import (
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// This file owns the proof behind the bound every `Face.NormalAt` arm
// publishes (topology.go in the parent package).
//
// The exact answer a normal reading is judged against is the outward UNIT
// normal of the surface the face's own tag names, evaluated in exact
// arithmetic on that tag's held numbers. Those numbers are the surface the
// record denotes: a placement re-evaluates the record and stores its own
// coordinates, so the record IS what it denotes — the same rule
// prismPayload's sectionDelta states for a section
// (docs/prism-boolean-design.md §7).
//
// Two independent things separate that exact direction from the float triple
// an arm hands back, and the bound below covers both:
//
//   - The ARITHMETIC the arm runs. A difference of two points, a projection
//     onto the axis, a normalization, the cross product an r3.Frame derives a
//     Plane's normal from on every call — it stores its two in-plane axes and
//     no normal at all — and, for a Cone, a cosine and a sine of the held half
//     angle are each rounded, and none of them is charged anywhere else.
//   - The held tag's own departure from the UNIT length a direction must
//     have. A placed cap's frame normal is the rounded image of a rotation,
//     so its length is not exactly one; a vector of length 1+e is not the
//     exact unit normal of any surface at all, and the distance to the
//     nearest unit vector is exactly |e|.
//
// Face.NormalAt separately composes this arm bound with Face.normalBound where
// the surface actually carried is only bounded-close to its published tag.
//
// Every enclosure here is a rational interval, so the answer never depends on
// what the platform's own libm did — the same discipline moments.go and
// capblend_contour.go already run on. Where an enclosure cannot separate the
// direction from zero the reading REFUSES, rather than publish a bound
// nothing proves.
//
// None of this is a residual gate, and no small number here admits anything.
// The enclosure IS the exact answer, built from the tag's own numbers in
// rational arithmetic, so a zero bound records that the two agree EXACTLY —
// never that they agree closely. That is what separates it from the residual
// CLAUDE.md's reject-only rule bans an admission from: there is no upstream
// claim being blessed, only a closed form being evaluated twice.

// Status is the three-valued outcome of enclosing a face's own exact
// normal, the same shape clearance_degen.go's oracle uses: a proven answer, a
// proven degeneracy, and an undecided case that is never quietly folded into
// either.
type Status int

const (
	// Proven means the exact unit normal is enclosed and its bound
	// stated.
	Proven Status = iota
	// Zero means the direction is EXACTLY zero, so the surface gives
	// this point no normal at all.
	Zero
	// Unproven means the enclosure could not separate the direction
	// from zero, or a held number is not finite. Nothing is claimed.
	Unproven
)

// UnitVec3 encloses the exact unit vector of an enclosed direction. It is
// the 3D sibling of internal/capcontour's UnitVec, and like it the only
// widening a held-float input suffers is the length's own outward-rounded
// square root.
func UnitVec3(a survey2d.IvVec3) (survey2d.IvVec3, Status) {
	n2 := survey2d.IvVec3NormSq(a)
	if n2.Hi.Sign() <= 0 {
		return survey2d.IvVec3{}, Zero
	}
	if n2.Lo.Sign() <= 0 {
		return survey2d.IvVec3{}, Unproven
	}
	length, ok := proofbound.IntervalSqrt(n2)
	if !ok || length.Lo.Sign() <= 0 {
		return survey2d.IvVec3{}, Unproven
	}
	var out survey2d.IvVec3
	for i, comp := range a {
		q, ok := proofbound.IntervalQuo(comp, length)
		if !ok {
			return survey2d.IvVec3{}, Unproven
		}
		out[i] = q
	}
	return out, Proven
}

// unitDirAllow is the whole file's answer: an upper bound on the distance
// between the direction an arm computed and the exact unit normal its own
// enclosure names.
//
// The sign a reversed face applies is a float negation, which is exact, so
// the bound is the same for the outward direction and the geometric one and
// the caller may pass either — as long as both arguments carry the same sign.
func unitDirAllow(exact survey2d.IvVec3, held r3.Vec) (float64, Status) {
	unit, st := UnitVec3(exact)
	if st != Proven {
		return 0, st
	}
	ex := proofbound.IntervalFloatError(unit[0], held.X)
	ey := proofbound.IntervalFloatError(unit[1], held.Y)
	ez := proofbound.IntervalFloatError(unit[2], held.Z)
	if proofbound.IsNonFinite(ex) || proofbound.IsNonFinite(ey) || proofbound.IsNonFinite(ez) {
		return 0, Unproven
	}
	// The three coordinate errors are known separately, so the distance is
	// their own root sum of squares — proofbound.Radius3D's √3 factor is the answer for
	// ONE per-coordinate number, and would only loosen this one.
	sum := proofbound.AbsSumUpper(proofbound.ProductUpper(ex, ex), proofbound.ProductUpper(ey, ey), proofbound.ProductUpper(ez, ez))
	allow := proofbound.UpRound(math.Sqrt(sum))
	if proofbound.IsNonFinite(allow) {
		return 0, Unproven
	}
	return allow, Proven
}

// axialRadialExact encloses the exact vector from a surface's axis to p,
// perpendicular to that axis: rel − â(rel·â) with â the axis's OWN exact unit
// direction. Written as rel − a(rel·a)/(a·a) it needs no square root at all,
// so a held-float tag encloses it exactly.
//
// Writing it with the unit axis matters: the arms spell the projection with
// the tag's held Axis, which a placement leaves only near-unit, and the
// difference between the two spellings is part of what this file is charging.
func axialRadialExact(p, origin, axis r3.Vec) (survey2d.IvVec3, bool) {
	pi, okP := survey2d.IvVec3Of(p)
	oi, okO := survey2d.IvVec3Of(origin)
	ai, okA := survey2d.IvVec3Of(axis)
	if !okP || !okO || !okA {
		return survey2d.IvVec3{}, false
	}
	rel := survey2d.IvVec3Sub(pi, oi)
	share, ok := proofbound.IntervalQuo(survey2d.IvVec3Dot(rel, ai), survey2d.IvVec3NormSq(ai))
	if !ok {
		return survey2d.IvVec3{}, false
	}
	return survey2d.IvVec3Sub(rel, survey2d.IvVec3Mul(ai, share)), true
}

// PlaneAllow bounds the Plane arm's own reading. An r3.Frame stores no
// normal — it holds its origin and its two in-plane axes, and derives the
// normal as their cross product on every call — so the arm computes six
// products and three differences, each rounded, and the exact answer is the
// unit vector of the EXACT cross of those two held axes. The bound covers both
// halves of the distance to it: the cross product's own rounding and the
// resulting triple's departure from unit length. It is zero exactly where the
// held axes cross exactly and give a unit result, which is every axis-aligned
// frame.
func PlaneAllow(fr r3.Frame, held r3.Vec) (float64, Status) {
	u, okU := survey2d.IvVec3Of(fr.U())
	v, okV := survey2d.IvVec3Of(fr.V())
	if !okU || !okV {
		return 0, Unproven
	}
	return unitDirAllow(survey2d.IvVec3Cross(u, v), held)
}

// AxialAllow bounds the Cylinder arm's reading: its exact normal is the
// axis-to-point direction the projection above names.
func AxialAllow(p, origin, axis r3.Vec, held r3.Vec) (float64, Status) {
	exact, ok := axialRadialExact(p, origin, axis)
	if !ok {
		return 0, Unproven
	}
	return unitDirAllow(exact, held)
}

// RadialAllow bounds a reading whose exact direction is one held
// difference — the Sphere arm's centre-to-point.
func RadialAllow(p, center r3.Vec, held r3.Vec) (float64, Status) {
	pi, okP := survey2d.IvVec3Of(p)
	ci, okC := survey2d.IvVec3Of(center)
	if !okP || !okC {
		return 0, Unproven
	}
	return unitDirAllow(survey2d.IvVec3Sub(pi, ci), held)
}

// ConeAllow bounds the Cone arm's reading. The exact normal is
// r̂·cos(h) − â·sin(h) with r̂ and â the surface's own exact unit radial and
// axis, which is why this arm is inexact even on a body no placement ever
// touched: the arm reads cos and sin off float64 libm, and at every integer
// degree of half angle the pair it gets back does not satisfy cos² + sin² = 1.
func ConeAllow(p, origin, axis r3.Vec, half float64, held r3.Vec) (float64, Status) {
	radial, ok := axialRadialExact(p, origin, axis)
	if !ok {
		return 0, Unproven
	}
	rdir, st := UnitVec3(radial)
	if st != Proven {
		return 0, st
	}
	axisIv, okA := survey2d.IvVec3Of(axis)
	if !okA {
		return 0, Unproven
	}
	adir, st := UnitVec3(axisIv)
	if st != Proven {
		return 0, Unproven
	}
	rHalf := proofarith.FloatRat(half)
	if rHalf == nil {
		return 0, Unproven
	}
	sin, cos, okT := survey2d.RadSinCosInterval(rHalf)
	if !okT {
		return 0, Unproven
	}
	return unitDirAllow(survey2d.IvVec3Sub(survey2d.IvVec3Mul(rdir, cos), survey2d.IvVec3Mul(adir, sin)), held)
}

// TorusAllow bounds the Torus arm's reading: the exact direction runs
// from the tube centre at p's own azimuth to p, and the tube centre itself is
// already an enclosure, since it rides the unit radial.
func TorusAllow(p, center, axis r3.Vec, major float64, held r3.Vec) (float64, Status) {
	radial, ok := axialRadialExact(p, center, axis)
	if !ok {
		return 0, Unproven
	}
	rdir, st := UnitVec3(radial)
	if st != Proven {
		return 0, st
	}
	rMajor := proofarith.FloatRat(major)
	pi, okP := survey2d.IvVec3Of(p)
	ci, okC := survey2d.IvVec3Of(center)
	if rMajor == nil || !okP || !okC {
		return 0, Unproven
	}
	rel := survey2d.IvVec3Sub(pi, ci)
	return unitDirAllow(survey2d.IvVec3Sub(rel, survey2d.IvVec3Mul(rdir, proofbound.PointInterval(rMajor))), held)
}
