package sectionrecord

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/units"
)

var (
	ErrDegenerate        = decaderr.ErrDegenerate
	ErrNotFinite         = decaderr.ErrNotFinite
	ErrUnitKind          = decaderr.ErrUnitKind
	ErrNegativeMagnitude = decaderr.ErrNegativeMagnitude
)

// The root package keeps its private validation call sites while this package
// owns their behavior.
func FiniteSegmentValue(value float64) bool { return finiteSegmentValue(value) }

func ValidateNURBSSegmentSizes(segment NURBSSeg) error   { return validateNURBSSegmentSizes(segment) }
func ValidateNURBSSegmentContent(segment NURBSSeg) error { return validateNURBSSegmentContent(segment) }

func NormalizeSegment(segment CurveSegment) (CurveSegment, error) { return normalizeSegment(segment) }

// MagnitudeIn checks a typed, non-negative magnitude before conversion.
func MagnitudeIn(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, error) {
	if v.Kind() != kind {
		return 0, fmt.Errorf(`%w: %s must be a %s, got %s`, ErrUnitKind, what, kind, v.Kind())
	}
	m, err := v.In(unit)
	if err != nil {
		return 0, fmt.Errorf(`%w: %s is not representable: %s`, ErrNotFinite, what, err)
	}
	if m < 0 {
		return 0, fmt.Errorf(`%w: %s must be non-negative, got %s`, ErrNegativeMagnitude, what, v)
	}
	return m, nil
}

func finiteSegmentValue(v float64) bool {
	return !math.IsNaN(v) && !math.IsInf(v, 0)
}

// validateSegmentPoints checks a whole recorded point array. The per-point
// diagnostic is formatted only for the point that FAILS: this loop is the scan a
// caller-sized array pays for element by element, and formatting a description
// for every point made it some seventy times more expensive than the slice walk
// it is (spline design §5.2's linear-pass claim is about the walk).
func validateSegmentPoints(points []Point2, minimum int, what string) error {
	if len(points) < minimum {
		return fmt.Errorf(`%w: %s requires at least %d points, got %d`, ErrDegenerate, what, minimum, len(points))
	}
	for i, point := range points {
		if finiteSegmentValue(point.U) && finiteSegmentValue(point.V) {
			continue
		}
		return prependCodecPath(
			fmt.Errorf(`%w: %s point %d must have finite coordinates`, ErrNotFinite, what, i),
			fmt.Sprintf(`[%d]`, i),
		)
	}
	return nil
}

// validateNURBSSegmentSizes is the O(1) structural preflight: the degree and
// every slice length the content checks below index through. It reads no
// element, so a caller can refuse a malformed record before scanning — or
// charging for — a control array the caller sized. A degree-1 segment holding
// millions of control points and no knots is refused here in constant time.
func validateNURBSSegmentSizes(seg NURBSSeg) error {
	if seg.Degree < 1 {
		return prependCodecPath(
			fmt.Errorf(`%w: NURBS segment degree must be at least 1, got %d`, ErrDegenerate, seg.Degree),
			"degree",
		)
	}
	n := len(seg.Control)
	if seg.Degree >= n {
		return prependCodecPath(
			fmt.Errorf(`%w: NURBS segment requires more control points than its degree %d, got %d`, ErrDegenerate, seg.Degree, n),
			"degree",
		)
	}
	if len(seg.Knots) != n+seg.Degree+1 {
		return prependCodecPath(
			fmt.Errorf(`%w: NURBS segment needs %d knots, got %d`, ErrDegenerate, n+seg.Degree+1, len(seg.Knots)),
			"knots",
		)
	}
	if len(seg.Weights) != n {
		return prependCodecPath(
			fmt.Errorf(`%w: NURBS segment needs %d weights, got %d`, ErrDegenerate, n, len(seg.Weights)),
			"weights",
		)
	}
	return nil
}

// validateNURBSSegmentContent checks every element the sizes above admit:
// control-point finiteness, the knot vector's finiteness, ordering, clamping,
// domain and interior multiplicity, and each weight. Its callers run
// validateNURBSSegmentSizes first, which is what lets it index without
// re-testing a length.
//
// Each per-element diagnostic is formatted only for the element that FAILS. This
// is a scan whose length the caller chose, so a description built for every knot
// and every weight dominated the walk itself by roughly seventy times.
//
// On the moment path this runs behind freeform.ChargeRationalLift (work_budget.go), and
// every check here must keep that charge's invariant: a single walk over Control,
// Knots or Weights, whose length the charge counts. A check of any other shape —
// over an array none of those lengths measures, or more than a constant number of
// walks over one of them — is outside the invariant and owes a charge of its own.
func validateNURBSSegmentContent(seg NURBSSeg) error {
	n := len(seg.Control)
	if err := validateSegmentPoints(seg.Control, seg.Degree+1, "NURBS segment control"); err != nil {
		return prependCodecPath(err, "control")
	}
	for i, knot := range seg.Knots {
		if !finiteSegmentValue(knot) {
			return prependCodecPath(
				fmt.Errorf(`%w: NURBS segment knot %d must be finite`, ErrNotFinite, i),
				fmt.Sprintf(`knots[%d]`, i),
			)
		}
		if i > 0 && knot < seg.Knots[i-1] {
			return prependCodecPath(fmt.Errorf(`%w: NURBS segment knots must be non-decreasing`, ErrDegenerate), fmt.Sprintf(`knots[%d]`, i))
		}
	}
	for i := 1; i <= seg.Degree; i++ {
		if seg.Knots[i] != seg.Knots[0] {
			return prependCodecPath(fmt.Errorf(`%w: NURBS segment knot vector is not clamped at the start`, ErrDegenerate), fmt.Sprintf(`knots[%d]`, i))
		}
		if seg.Knots[len(seg.Knots)-1-i] != seg.Knots[len(seg.Knots)-1] {
			return prependCodecPath(fmt.Errorf(`%w: NURBS segment knot vector is not clamped at the end`, ErrDegenerate), fmt.Sprintf(`knots[%d]`, len(seg.Knots)-1-i))
		}
	}
	if seg.Knots[seg.Degree] >= seg.Knots[n] {
		return prependCodecPath(fmt.Errorf(`%w: NURBS segment knot domain is empty`, ErrDegenerate), "knots")
	}
	if err := validateNURBSInteriorMultiplicity(seg, n); err != nil {
		return err
	}
	for i, weight := range seg.Weights {
		if !finiteSegmentValue(weight) {
			return prependCodecPath(
				fmt.Errorf(`%w: NURBS segment weight %d must be finite`, ErrNotFinite, i),
				fmt.Sprintf(`weights[%d]`, i),
			)
		}
		if weight <= 0 {
			return prependCodecPath(
				fmt.Errorf(`%w: NURBS segment weight %d must be positive, got %v`, ErrDegenerate, i, weight),
				fmt.Sprintf(`weights[%d]`, i),
			)
		}
	}
	return nil
}

// validateNURBSInteriorMultiplicity rejects a knot strictly inside the clamped
// domain at which the recorded curve BREAKS APART, so the record states several
// disjoint pieces rather than the single connected boundary curve a loop's
// segment is. That is a malformed record, on the same footing as the clamping,
// monotonicity and empty-domain rules beside it.
//
// Multiplicity alone does not decide it. A multiplicity of degree is a corner
// and the curve is still one connected curve through it. At a multiplicity m of
// degree+1 or more the two one-sided limits are exactly two RECORDED control
// points — for the knot occupying indices j+1..j+m they are P_j and
// P_{j+m−degree}, adjacent when m is degree+1 — and the weights do not enter, so
// continuity there is decided SOLELY by whether those two coordinates are
// identical. Identical, and the curve is continuous and the body exists, so this
// admits it: what refuses it later is the evaluator's own stride-degree slicing
// precondition (freeform.BezierSliceCount), which is a limitation of the evaluator and
// reports ErrUnsupported. Different, and no such body exists, which is this
// refusal.
//
// Equality is exact identity on the recorded coordinates, never a tolerance:
// both directions are exactly decidable on the floats the record holds, which is
// what the falsify-never-bless rule requires.
func validateNURBSInteriorMultiplicity(seg NURBSSeg, n int) error {
	lo, hi := seg.Knots[seg.Degree], seg.Knots[n]
	start, run := 0, 0
	for i := seg.Degree + 1; i < n; i++ {
		knot := seg.Knots[i]
		if knot <= lo || knot >= hi {
			if err := validateNURBSKnotRun(seg, start, run); err != nil {
				return err
			}
			run = 0
			continue
		}
		if run > 0 && knot == seg.Knots[start] {
			run++
			continue
		}
		if err := validateNURBSKnotRun(seg, start, run); err != nil {
			return err
		}
		start, run = i, 1
	}
	return validateNURBSKnotRun(seg, start, run)
}

// validateNURBSKnotRun refuses one interior knot run whose two one-sided limits
// are different recorded control points.
func validateNURBSKnotRun(seg NURBSSeg, start, run int) error {
	if run <= seg.Degree {
		return nil
	}
	left, right := start-1, start-1+run-seg.Degree
	if left < 0 || right >= len(seg.Control) || seg.Control[left] == seg.Control[right] {
		return nil
	}
	return prependCodecPath(
		fmt.Errorf(
			`%w: NURBS segment interior knot %v repeats %d times at degree %d and its two one-sided limits are different control points, so the curve breaks into disjoint pieces`,
			ErrDegenerate, seg.Knots[start], run, seg.Degree,
		),
		fmt.Sprintf(`knots[%d]`, start),
	)
}

// errNilSegment rejects a nil variant pointer: it names no curve to record.
// It wraps ErrDegenerate so the failure is branchable wherever it surfaces —
// the codec, the integrals, or a feature call.
var errNilSegment = fmt.Errorf(`%w: nil curve segment`, ErrDegenerate)

var ErrNilSegment = errNilSegment

// normalizeSegment returns the value form of s. The variants implement
// CurveSegment with value receivers, so a *LineSeg satisfies the interface as
// readily as a LineSeg does — the codec accepts both and records the value
// the pointer names; the decode buffer normalizes through the same path, so
// the codec always hands back value forms.
func normalizeSegment(s CurveSegment) (CurveSegment, error) {
	switch s := s.(type) {
	case *LineSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *CircleSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *ArcSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *EllipseSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *EllipticalArcSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *SplineSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *NURBSSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *ClosedSplineSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *FitSplineSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	case *ConicSeg:
		if s == nil {
			return nil, errNilSegment
		}
		return *s, nil
	default:
		return s, nil
	}
}
