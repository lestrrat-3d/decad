package survey2d

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// WalkKind discriminates what a SegmentWalk's geometry IS. It replaced a
// line-versus-circular boolean because a free-form walk is neither: a two-state
// flag left every "not circular" branch silently building a straight line out of
// a spline, which is exactly the confidently-wrong answer decad exists to
// prevent (docs/spline-design.md §6.2).
//
// A switch on WalkKind MUST be total. A consumer that cannot yet handle
// WalkFreeform refuses before building an analytic face: where it needs a walk
// it uses boundarywalk.RequireAnalyticWalk, and where no resolution can contribute it gates
// the recorded free-form kind before walkOf.
type WalkKind uint8

const (
	// WalkLine is a straight walk between its endpoints.
	WalkLine WalkKind = iota
	// WalkCircular is a circular walk about (cU, cV) — a circle or an arc.
	WalkCircular
	// WalkFreeform is a free-form walk, whose geometry lives in its converted
	// Bézier spans rather than in the circular fields.
	WalkFreeform
)

// SegmentWalk is one boundary segment's walk geometry in plane coordinates.
type SegmentWalk struct {
	// start/end are the walk's endpoints in (u, v); closed is true for a
	// whole closed curve (no junction vertices at all).
	StartU, StartV float64
	EndU, EndV     float64
	// startBound/endBound are the PROVEN error bounds on the endpoint beside
	// them, in the coordinates' own millimetres — radiusBound's twin two fields
	// up, and stated for the same reason. An endpoint is an exact leaf only
	// where the record STATES it: a line's own bounds and an arc's own bounds
	// are recorded coordinates the walk reads verbatim (lerp2, pinArcWalkEnds),
	// and those read zero. That zero is about THIS walk's own rounding, not
	// about the recorded coordinate agreeing with the curve the record denotes
	// at that parameter — arcWalkEnd's own doc comment states where an arc's
	// two readings part company, and names who owes the difference. Every other endpoint is computed — a trimmed line's
	// is a float lerp, a trimmed arc's and EVERY circle's is a
	// math.Cos/math.Sin at an angle this package itself computed — so each kind
	// STATES what its own endpoint is worth (lineWalkEndBound,
	// circularWalkEndBound, freeformEndpointBounds) or REFUSES with +Inf, never
	// leaves it silently zero. A reading that folds an endpoint into an answer
	// charges it through proofbound.PointPerturbationAllow; one that cannot state the
	// charge refuses on the +Inf rather than publishing an exactness the
	// evaluator never proved.
	StartBound, EndBound proofbound.WalkEndBound
	Closed               bool
	// tanIn/tanOut are the walk tangents at start and end (unit not
	// required), for junction convexity.
	TanInU, TanInV   float64
	TanOutU, TanOutV float64
	// tanInBound/tanOutBound are the PROVEN error bound on EITHER component
	// of the tangent beside them, in the coordinates' own millimetres. A
	// tangent is NOT an exact leaf the way a recorded coordinate is: a line
	// walk's is the float difference of two endpoints, a circular walk's runs
	// through math.Sincos at a computed angle, and a free-form walk's is an
	// exact rational leg rounded once into float64. Each kind therefore
	// STATES its bound or REFUSES with +Inf — never leaves it silently zero —
	// so a reading composed from a tangent can charge the error the evaluator
	// actually committed. The refusal is the circular kind's: its held
	// components come from a trig evaluation at an angle that is itself
	// computed, and this walk states no enclosure of either, so +Inf is the
	// underivable bound every consumer refuses on rather than publishes
	// (arcWalkRadiusBound's own convention).
	TanInBound, TanOutBound float64
	Length                  float64
	LengthBound             float64
	LengthUpper             float64
	CoordUpper              float64
	AxisRadiusUpper         float64
	AxisMomentUpper         float64
	// startVBound/endVBound/cVBound are the PROVEN error bounds on the radial
	// (V) axis-coordinate beside them — startV/endV/cV's own displacement from
	// the value the axis's TRUE (unrounded) direction and anchor would give,
	// through axisFrame.toAxisRhoBound, composed for startV/endV with whatever
	// magnitude that walk's own axis snap discarded to assign an endpoint
	// exactly zero. They are set ONLY by axisFrame.walk,
	// which re-expresses a plane-local walk into axis coordinates: a walk that
	// has not been through it (every use before revolve resolves an axis)
	// leaves them at their zero value, meaningless there. axisFrame.toAxis
	// itself states no such bound (its own doc comment), so a caller
	// composing a reading from startV/endV/cV — the revolve minimum-radius
	// meridian survey (radiussurvey.Revolve) — reads these instead of
	// the coordinate as an exact leaf; revolvemass.AxisMoments folds the
	// SAME axis-direction/anchor uncertainty into the region's moments through
	// bounded arithmetic instead, and does not read these fields.
	StartVBound, EndVBound, CVBound float64
	// kind says which geometry the walk carries; the fields below it are
	// meaningful only for WalkCircular.
	Kind   WalkKind
	CU, CV float64
	Radius float64
	// radiusBound is the PROVEN error bound on radius (millimetres). A
	// CircleSeg states its radius, so its walk holds that number and the bound
	// is zero; an ArcSeg states Start and Center only, so its walk's radius is
	// a math.Hypot evaluation and the bound is arcWalkRadiusBound's rational
	// bracket. It exists because radius is NOT an exact leaf the way a
	// recorded coordinate is, and a reading that treats it as one can publish
	// an interval its own truth sits outside of. The analytic surveys
	// (survey.go's minimum-radius arms, and survey2d.go through
	// SurveyElem.rrBound) take it; a consumer that reads radius as a leaf
	// still owes its own account of the error, from its own envelope.
	RadiusBound float64
	Th0, Th1    float64
	// spans is the converted Bézier chain of a WalkFreeform walk, in the
	// curve's natural direction; reversed says the walk runs against it. Both
	// are zero for every other kind.
	//
	// The chain is SHARED and MUST NOT be mutated in place. A SegmentWalk
	// copies only the slice header, so every reader of one ProfileWalks set
	// holds the same ratPoints: the build (buildLoopSidesAs), the tessellation
	// (chordLoop), the extent readings, and a rigid re-evaluation that reads
	// the published set back. Writing through any of them writes through all
	// of them.
	//
	// The guard cannot catch such a write. ProfileWalks.Reusable decides on
	// matches, which compares the RECORD by float bits and never inspects the
	// walks, so a mutated set still reads back as the resolution of its own
	// record. The corruption would also be quiet rather than loud: a
	// re-anchoring subtracts a constant from every control point, which leaves
	// a valid curve sitting somewhere else, so the lengths and areas built from
	// it stay plausible and the build and the tessellation still agree with
	// each other, both being wrong in the same way.
	//
	// shiftFreeformSpans (spline_bezier.go) is exactly this write. It is safe
	// where it stands because validateFreeformMomentSegment converts its own
	// chain through freeformBezierSpans and hands it that private copy, never a
	// walk. A caller that hands it one of THESE chains instead has no test that
	// would fail. Re-anchor a copy, or convert afresh.
	Spans    []freeform.BezierSpan
	Reversed bool
	// fitInterpolated is set only for a WalkFreeform walk whose chain came
	// from FitSplineSeg's §5.1.2 conversion (spline_fit.go's isFitSplineSeg,
	// read on the segment walkOf resolved — walkOf normalizes as its first
	// statement, so freeformWalk's own seg, and every isFitSplineSeg check
	// downstream of it, always sees the normalized value form). §6.5's
	// convexity certificate needs it to apply the FitSplineSeg carve-out: a
	// joint interior to that conversion's chain is verdict 0 BY CONSTRUCTION,
	// never by jointConvexitySign's cross product, because that cross carries
	// sketch's own rounded SecondDerivs solve rather than a turn of the
	// recorded curve (docs/spline-design.md §6.5, §5.1.2). It is false for
	// every other Tier A kind, whose joints are genuine C⁰ corners the cross
	// product must still fold.
	FitInterpolated bool
}

// isCircular reports whether the walk is a circle or arc — the question the
// closed-form circular branches ask.
func (w SegmentWalk) IsCircular() bool { return w.Kind == WalkCircular }

// isLine reports whether the walk is straight. It is NOT "not circular": a
// free-form walk answers false to both.
func (w SegmentWalk) IsLine() bool { return w.Kind == WalkLine }

// SideWalk is one side face's walk after canonicalization: consecutive
// collinear line walks coalesce into one (evaluator §3 — "adjacent coplanar
// side faces merge"), and the merged face carries every constituent
// segment's role.
type SideWalk struct {
	SegmentWalk
	Segs []int // the recorded segment indices this walk covers
}
