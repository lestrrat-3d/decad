package revolveaxis

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// AxisInput records a public axis variant's numeric fields.
type AxisInput interface{ axisInput() }

// SketchLine records the plane-local endpoints of a sketch axis.
type SketchLine struct{ StartU, StartV, EndU, EndV float64 }

func (SketchLine) axisInput() {}

// ConstructionAxis records a world-space origin and direction.
type ConstructionAxis struct{ Origin, Dir r3.Vec }

func (ConstructionAxis) axisInput() {}

// Line2 is a resolved plane-local axis with proven coordinate bounds.
type Line2 struct {
	AU, AV, AUBound, AVBound float64
	DU, DV, DUBound, DVBound float64
}

// FiniteAxisValues reports whether every derived axis value is representable.
// Axis inputs are checked before arithmetic, but subtracting finite endpoints
// or transforming a finite world point can still overflow.
func FiniteAxisValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

// AxisInPlane resolves an axis variant into plane-local coordinates,
// validating it non-degenerate and coplanar with the profile plane
// (docs/evaluator-design.md §6).
func AxisInPlane(a AxisInput, frame r3.Frame) (Line2, error) {
	switch a := a.(type) {
	case SketchLine:
		for _, c := range []float64{a.StartU, a.StartV, a.EndU, a.EndV} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return Line2{}, fmt.Errorf(`%w: a sketch-line axis endpoint is not finite`, decaderr.ErrNotFinite)
			}
		}
		du, dv := a.EndU-a.StartU, a.EndV-a.StartV
		if !FiniteAxisValues(du, dv) {
			return Line2{}, fmt.Errorf(`%w: a sketch-line axis delta is not finite`, decaderr.ErrNotFinite)
		}
		scale := math.Max(math.Abs(du), math.Abs(dv))
		if scale == 0 {
			return Line2{}, fmt.Errorf(`%w: a zero-length sketch line names no axis`, decaderr.ErrDegenerate)
		}
		scaledU, scaledV := du/scale, dv/scale
		l := math.Hypot(scaledU, scaledV)
		if !FiniteAxisValues(l) {
			return Line2{}, fmt.Errorf(`%w: a sketch-line axis length is not finite`, decaderr.ErrNotFinite)
		}
		dU, dV := scaledU/l, scaledV/l
		if !FiniteAxisValues(dU, dV) {
			return Line2{}, fmt.Errorf(`%w: a sketch-line axis direction is not finite`, decaderr.ErrNotFinite)
		}
		// Recover the held length for exact direction-bound checks. An overflowing
		// magnitude falls back to the conservative direction bound.
		l = scale * l
		dUBound, dVBound := sketchAxisDirectionBounds(a, l, dU, dV)
		return Line2{
			AU: a.StartU, AV: a.StartV,
			DU: dU, DV: dV,
			DUBound: dUBound,
			DVBound: dVBound,
		}, nil
	case ConstructionAxis:
		for _, c := range []float64{a.Origin.X, a.Origin.Y, a.Origin.Z, a.Dir.X, a.Dir.Y, a.Dir.Z} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return Line2{}, fmt.Errorf(`%w: a construction axis component is not finite`, decaderr.ErrNotFinite)
			}
		}
		dir, ok := a.Dir.Normalize()
		if !ok {
			return Line2{}, fmt.Errorf(`%w: a zero-direction construction axis names no axis`, decaderr.ErrDegenerate)
		}
		if !FiniteAxisValues(dir.X, dir.Y, dir.Z) {
			return Line2{}, fmt.Errorf(`%w: a normalized construction axis direction is not finite`, decaderr.ErrNotFinite)
		}
		local := frame.ToLocal(a.Origin)
		if !FiniteAxisValues(local.X, local.Y, local.Z) {
			return Line2{}, fmt.Errorf(`%w: a construction axis has non-finite plane-local coordinates`, decaderr.ErrNotFinite)
		}
		localLen := local.Len()
		if !FiniteAxisValues(localLen) {
			return Line2{}, fmt.Errorf(`%w: a construction axis plane-local length is not finite`, decaderr.ErrNotFinite)
		}
		scale := math.Max(1, localLen)
		if math.Abs(local.Z) > 1e-9*scale {
			return Line2{}, fmt.Errorf(`%w: the revolve axis does not lie in the profile plane`, decaderr.ErrDegenerate)
		}
		du, dv, dn := dir.Dot(frame.U()), dir.Dot(frame.V()), dir.Dot(frame.N())
		if !FiniteAxisValues(du, dv, dn) {
			return Line2{}, fmt.Errorf(`%w: a construction axis has a non-finite plane-local direction`, decaderr.ErrNotFinite)
		}
		if math.Abs(dn) > 1e-9 {
			return Line2{}, fmt.Errorf(`%w: the revolve axis does not lie in the profile plane`, decaderr.ErrDegenerate)
		}
		l := math.Hypot(du, dv)
		if !FiniteAxisValues(l) {
			return Line2{}, fmt.Errorf(`%w: a construction axis plane-local direction length is not finite`, decaderr.ErrNotFinite)
		}
		if l == 0 {
			return Line2{}, fmt.Errorf(`%w: a construction axis has no direction in the profile plane`, decaderr.ErrDegenerate)
		}
		dU, dV := du/l, dv/l
		if !FiniteAxisValues(dU, dV) {
			return Line2{}, fmt.Errorf(`%w: a normalized construction axis plane-local direction is not finite`, decaderr.ErrNotFinite)
		}
		// The anchor's plane-local coordinates take the ROUNDING their own
		// projection committed (internal/proofbound/bounds.go's proofbound.ExactFrameLocalRound), measured
		// exactly against the frame and the world origin as the exact leaves
		// they are — zero for an exactly representable projection, and never
		// the anchor's own distance from the frame origin, which bounds the
		// coordinate's magnitude and not its error. The magnitude envelope
		// survives only as the fallback for a component no rational holds.
		anchorUpper := proofbound.AbsSumUpper(
			a.Origin.X, a.Origin.Y, a.Origin.Z,
			frame.Origin().X, frame.Origin().Y, frame.Origin().Z,
		)
		aUBound := math.Min(
			proofbound.ExactFrameLocalRound(frame, a.Origin, frame.U(), local.X),
			proofbound.ConservativeValueError(local.X, anchorUpper),
		)
		aVBound := math.Min(
			proofbound.ExactFrameLocalRound(frame, a.Origin, frame.V(), local.Y),
			proofbound.ConservativeValueError(local.Y, anchorUpper),
		)
		// The bracket needs the axis direction's raw, PRE-normalize exact
		// rational dot products against the frame's in-plane axes: rawDU,
		// rawDV = a.Dir·frame.U(), a.Dir·frame.V(). dU/dV above take TWO
		// normalize steps — a.Dir.Normalize() in 3D, then Hypot(du,dv)
		// re-normalizes the projected pair to unit length within the plane
		// — and algebraically the two steps' magnitudes cancel:
		// du = rawDU/|a.Dir|, dv = rawDV/|a.Dir|, so
		// l = Hypot(du,dv) = sqrt(rawDU²+rawDV²)/|a.Dir|, and dU = du/l =
		// rawDU/sqrt(rawDU²+rawDV²) with |a.Dir| gone. The exact closed
		// form these two steps compute is exactly the du/dv shape the
		// SketchLine arm bounds, whatever frame.N() component a.Dir
		// carries — the coplanarity gate above rejects a direction the
		// N component makes materially non-planar, but the bracket below
		// needs no such assumption to be sound.
		rawDU, rawDV := ratVecDot(a.Dir, frame.U()), ratVecDot(a.Dir, frame.V())
		var dUBound, dVBound float64
		if rawDU == nil || rawDV == nil {
			dUBound, dVBound = proofbound.ConservativeValueError(dU, 1), proofbound.ConservativeValueError(dV, 1)
		} else {
			dUBound, dVBound = AxisDirectionSqrtBracket(rawDU, rawDV, dU, dV)
		}
		if (dU == 0 || math.Abs(dU) == 1) &&
			(dV == 0 || math.Abs(dV) == 1) &&
			dU*dU+dV*dV == 1 {
			dUBound, dVBound = 0, 0
		}
		return Line2{
			AU: local.X, AV: local.Y,
			AUBound: aUBound,
			AVBound: aVBound,
			DU:      dU, DV: dV,
			DUBound: dUBound,
			DVBound: dVBound,
		}, nil
	default:
		// EdgeAxis is gated before extent resolution; any other variant is
		// staged, never guessed.
		return Line2{}, fmt.Errorf(`%w: axis %T is not supported by this evaluator`, decaderr.ErrUnsupported, a)
	}
}

// ratVecDot is the exact rational dot product of two r3.Vec, each component
// read as the exact rational value its own float64 bit pattern denotes
// (floatRat). It returns nil only for a non-finite component; every caller
// here has already validated its vectors finite.
func ratVecDot(a, b r3.Vec) *big.Rat {
	ax, ay, az := proofarith.FloatRat(a.X), proofarith.FloatRat(a.Y), proofarith.FloatRat(a.Z)
	bx, by, bz := proofarith.FloatRat(b.X), proofarith.FloatRat(b.Y), proofarith.FloatRat(b.Z)
	if ax == nil || ay == nil || az == nil || bx == nil || by == nil || bz == nil {
		return nil
	}
	sum := new(big.Rat).Mul(ax, bx)
	sum.Add(sum, new(big.Rat).Mul(ay, by))
	sum.Add(sum, new(big.Rat).Mul(az, bz))
	return sum
}

func sketchAxisDirectionBounds(a SketchLine, heldLength, heldU, heldV float64) (float64, float64) {
	u0, v0 := proofarith.FloatRat(a.StartU), proofarith.FloatRat(a.StartV)
	u1, v1 := proofarith.FloatRat(a.EndU), proofarith.FloatRat(a.EndV)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return proofbound.ConservativeValueError(heldU, 1), proofbound.ConservativeValueError(heldV, 1)
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)
	fallbackU, fallbackV := AxisDirectionSqrtBracket(du, dv, heldU, heldV)
	length := proofarith.FloatRat(heldLength)
	if length == nil || length.Sign() == 0 {
		return fallbackU, fallbackV
	}
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	)
	if new(big.Rat).Mul(length, length).Cmp(lengthSquared) != 0 {
		return fallbackU, fallbackV
	}
	// The held length already proves an exact rational quotient for each
	// component: a Pythagorean or axis-aligned length that lands exactly
	// keeps a zero bound even where the sqrt bracket above cannot collapse
	// to a point (its own float division still rounds).
	exactComponent := func(delta *big.Rat, held, fallback float64) float64 {
		exact := new(big.Rat).Quo(delta, length)
		heldRat := proofarith.FloatRat(held)
		if heldRat != nil && exact.Cmp(heldRat) == 0 {
			return 0
		}
		return fallback
	}
	return exactComponent(du, heldU, fallbackU), exactComponent(dv, heldV, fallbackV)
}

// AxisDirectionSqrtBracket proves how far the held unit-direction components
// heldU, heldV — each dU = du/L, dV = dv/L with L = sqrt(du²+dv²) — can sit
// from the axis's own exact direction, through the same sqrt bracket the
// straight-prism campaign proved (internal/boundarywalk/walk.go's lineWalkBounds /
// dySqrtIntervalError): L² = du²+dv² is exact rational arithmetic, and
// proofbound.RatSqrtDown/proofbound.RatSqrtUp (internal/freeform/spline_length.go) bracket its root by exact
// comparison, without assuming any libm accuracy from the division that
// produced the held float. du and dv are the axis's own exact-rational
// leaves — a SketchLine's endpoint coordinate differences, or a
// ConstructionAxis's exact rational dot products of its held direction
// against the frame's in-plane axes. A degenerate direction, or a component
// the bracket cannot confirm sits as tightly as this proof can show, keeps
// proofbound.ConservativeValueError's magnitude envelope: math.Min only ever shrinks
// it, never replaces it with a wider answer.
func AxisDirectionSqrtBracket(du, dv *big.Rat, heldU, heldV float64) (float64, float64) {
	fallbackU, fallbackV := proofbound.ConservativeValueError(heldU, 1), proofbound.ConservativeValueError(heldV, 1)
	lengthSquared := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	if lengthSquared.Sign() == 0 {
		return fallbackU, fallbackV
	}
	sqrtIv, ok := proofbound.IntervalSqrt(proofbound.PointInterval(lengthSquared))
	if !ok {
		return fallbackU, fallbackV
	}
	uBound := fallbackU
	if enc, ok := proofbound.IntervalQuo(proofbound.PointInterval(du), sqrtIv); ok {
		uBound = math.Min(fallbackU, proofbound.IntervalFloatError(enc, heldU))
	}
	vBound := fallbackV
	if enc, ok := proofbound.IntervalQuo(proofbound.PointInterval(dv), sqrtIv); ok {
		vBound = math.Min(fallbackV, proofbound.IntervalFloatError(enc, heldV))
	}
	return uBound, vBound
}
