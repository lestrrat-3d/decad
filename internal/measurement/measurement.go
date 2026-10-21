package measurement

import (
	"math"

	"github.com/lestrrat-3d/decad/report"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Exactness is the public report.Exactness; see there for its meaning.
type Exactness = report.Exactness

const (
	Exact       = report.Exact
	Approximate = report.Approximate
)

// Measurement is a scalar quantity plus how far it can be trusted. Value is a
// typed quantity — never a naked float (agent-docs/api-design.md §5.1) — and Bound
// is the absolute error bound the evaluator proves, of the same Kind as
// Value: the error bound on a volume is a volume. Bound is zero when the
// measurement is Exact.
type Measurement = struct {
	Value     units.Value
	Exactness Exactness
	Bound     units.Value
}

// VecMeasurement is a computed coordinate or direction, with how far it can
// be trusted. It is what the API returns wherever the evaluator computes a
// vector: a computed vector never crosses the API as a bare r3.Vec.
//
// Value is a position — a length in millimetres, the agent-docs/api-design.md §5.2
// carve-out — or a unit direction, which is dimensionless and carries no unit
// at all. Bound follows Value: it is the radius of the ball around Value the
// true vector is proven to lie in, so its Kind is Length for a position and
// Dimensionless for a direction (the deviation of a unit vector from the true
// unit vector). Bound is zero when the measurement is Exact.
type VecMeasurement = struct {
	Value     r3.Vec
	Exactness Exactness
	Bound     units.Value
}

// Box is an axis-aligned bounding box, as tight as the evaluator can prove —
// and it reports its own exactness: a box around a curved body produced by a
// tessellation-backed boolean is bounded by faceted faces and is therefore
// not tight, and says so. Min and Max are positions, lengths in millimetres
// (agent-docs/api-design.md §5.2); Bound is the absolute error bound on them, of
// Kind Length, zero when the box is Exact.
type Box = struct {
	Min, Max  r3.Vec
	Exactness Exactness
	Bound     units.Value
}

// BoxesDisjoint reports whether the two bounds-inflated boxes have disjoint
// interiors. A body's interior lies within its box's interior, so disjoint
// box interiors prove the bodies cannot share volume — touching boxes
// included (evaluator §10).
func BoxesDisjoint(a, b Box) bool {
	ia := a.Bound.Base()
	ib := b.Bound.Base()
	return inflatedBoxEndsDisjoint(a.Max.X, ia, b.Min.X, ib) ||
		inflatedBoxEndsDisjoint(b.Max.X, ib, a.Min.X, ia) ||
		inflatedBoxEndsDisjoint(a.Max.Y, ia, b.Min.Y, ib) ||
		inflatedBoxEndsDisjoint(b.Max.Y, ib, a.Min.Y, ia) ||
		inflatedBoxEndsDisjoint(a.Max.Z, ia, b.Min.Z, ib) ||
		inflatedBoxEndsDisjoint(b.Max.Z, ib, a.Min.Z, ia)
}

// inflatedBoxEndsDisjoint compares the exact inflated endpoints. A strict
// float comparison has the same ordering because rounding is monotone; when
// both expressions round to the same float, compare their dyadic values.
func inflatedBoxEndsDisjoint(maxCoord, maxBound, minCoord, minBound float64) bool {
	upper, lower := maxCoord+maxBound, minCoord-minBound
	if upper != lower {
		return upper < lower
	}
	exactUpper := proofarith.DyAdd(proofarith.MustDyOf(maxCoord), proofarith.MustDyOf(maxBound))
	exactLower := proofarith.DySubScalar(proofarith.MustDyOf(minCoord), proofarith.MustDyOf(minBound))
	return proofarith.DyCmp(exactUpper, exactLower) <= 0
}

// ExactnessOf reports Exact for a zero bound and Approximate otherwise: a
// held value is exact only when its proven error bound is zero.
func ExactnessOf(bound float64) Exactness {
	if bound == 0 {
		return Exact
	}
	return Approximate
}

// FiniteValues reports whether every value is finite: neither NaN nor an
// infinity.
func FiniteValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}
