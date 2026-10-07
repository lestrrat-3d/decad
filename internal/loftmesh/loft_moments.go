package loftmesh

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// LoftVertexDistance is local to one assembled Loft. A ready entry can hold
// +Inf: that is the exact upper-distance result when float64 saturates.
type LoftVertexDistance struct {
	Upper float64
	Ready bool
}

// WallTriangleArea brackets one wall triangle's own area between two floats,
// both PROVEN. u and v are the triangle's exact rational edge vectors, so
// |u×v|² is an exact rational and the area is the square root of |u×v|²/4;
// proofbound.RatSqrtDown/proofbound.RatSqrtUp (internal/freeform/spline_length.go) bracket that rational root with
// OUTWARD rounding decided by exact comparison, so lo ≤ area ≤ hi holds
// whatever the platform's own sqrt does.
//
// The cross product is taken over rationals and NEVER in float64.
// r3.Vec.Cross is the naive difference-of-products form, whose forward error
// scales with the PRODUCTS rather than with the result, so a thin triangle's
// float area carries an error larger than the held sum's own summation slop by
// roughly one over the triangle's aspect ratio — and a bound read off that
// held sum would not enclose it. The wall of a short loft over long recorded
// LineSegs is exactly that shape (docs/loft-design.md Table B splits every
// wall quad along a diagonal), so this is the ordinary case, not an edge one.
func WallTriangleArea(u, v proofbound.Xpt) (float64, float64) {
	w := meshbool.Xcross(u, v)
	q := proofbound.XdotRat(w, w)
	q.Quo(q, big.NewRat(4, 1))
	return proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
}

// PlacedCentroidAllow bounds how far one already-computed centroid
// coordinate can move under a placement's proven volume and first-moment
// allowances, mirroring moments.go's proofbound.BoundedQuotient formula (§12 PR 2a):
// coordRel is the coordinate's own value relative to the accumulator's
// anchor, epsM the proven first-moment allowance (proofbound.SweptMomentAllow), epsV
// the proven volume allowance (proofbound.SweptVolumeAllow), and clearance the
// caller's own proven positive gap between the held volume and epsV (S12's
// own test, checked once by the caller since it does not depend on
// coordRel).
func PlacedCentroidAllow(coordRel, epsM, epsV, clearance float64) float64 {
	numerator := proofbound.AbsSumUpper(epsM, proofbound.ProductUpper(math.Abs(coordRel), epsV))
	return proofbound.UpRound(numerator / clearance)
}

// LoftChordedAllow bundles the exact volume and first-moment corrections,
// the unsigned twist measure retained for tessellation's occupied-volume
// proof, the three residual volume terms, and the wall's two-leg area residual
// (docs/loft-design.md §5/§8, a10-plan.md
// Part 3 PR 6's integration task). Every field of the zero value is 0, the
// correct standing for a LineSeg-only loft that never calls
// computeLoftChordedAllow at all. It is also what a REFUSING call returns
// beside its error, where the zero stands for nothing at all and no consumer
// ever sees it: evalLoft propagates that error and publishes no measurement.
type LoftChordedAllow struct {
	WallAreaUpper         float64
	TwistVolumeUpper      float64
	TwistVolumeCorrection *big.Rat
	TwistMomentCorrection proofbound.RatV3
	MaxTwistOffsetUpper   float64
	CapVolumeUpper        float64
	SeamAllow             float64
	// areaCorrection moves Area.Value from the held chord facets to the
	// bilinear wall patches. Its bound covers the correction's one rounding;
	// bilinearAreaBound covers those patches' certified integration intervals.
	AreaCorrection      float64
	AreaCorrectionBound float64
	BilinearAreaBound   float64
	// areaExcess is the wall's ruled and station-shift residuals summed over
	// chorded cells. The held-to-bilinear step is in Area.Value above.
	AreaExcess float64
	// twistAreaAllow is that SAME held-to-bilinear step read as a BOUND
	// rather than as a correction: proofbound.CellTwistAreaAllow summed over the same
	// chorded cells the loop below walks
	// (docs/tessellation-reach-design.md §4). area() never reads it, because
	// areaCorrection has already MOVED Area.Value onto the bilinear patches
	// and charging the gap again there would double-count it. The
	// tessellation does read it: the mesh holds the UNCORRECTED held
	// triangles, so the step areaCorrection performs is, for that triangle
	// set, an outstanding area gap and the mesh's own areaSlack must carry it
	// (docs/tessellation-design.md §2's loftPayload row). It stays exactly 0
	// on a build with no chorded cell, the zero value's own standing.
	TwistAreaAllow float64
	// capAreaExcess is capAreaAllow0 and capAreaAllow1 (each
	// proofbound.SectionDisplacementArea over its own cap's boundary) composed by
	// proofbound.AbsSumUpper: the SAME two per-cap area allowances capVolumeUpper folds
	// into a volume via proofbound.CapAreaVolumeAllow's own |h|/3 identity, spent here
	// UNFOLDED as an area instead — area()'s own AREA reading, unlike
	// volume(), has no plane offset h to divide by, since a cap's own
	// published area IS the built polygon's area and the true denoted area
	// differs from it by exactly this much (docs/loft-design.md §5/§8).
	CapAreaExcess float64
}

// ErrLoftCapOffsetUnderivable is the sentinel docs/loft-design.md Table S row
// S14 carries for the cap planeOffsetUpper term (§5.2), raised in the
// CONSTRUCTION arm §4's gate-order paragraph assigns that term — it reads the
// held vertex table, which the record-only arm does not have. Like its
// certified-sagitta and station-displacement twins the shape itself is fine and
// the chord set is buildable; only one of the proven displacement terms the
// published cap volume allowance is composed from cannot be stated, so the
// sentinel is ErrUnsupported and no finite value — least of all a zero — is
// published in its place.
var ErrLoftCapOffsetUnderivable = fmt.Errorf(
	`%w: a chorded loft cap's plane offset from the mass anchor has no derivation from this assembly`, decaderr.ErrUnsupported,
)

// ChordCellDeltaUpper is docs/loft-design.md §5.2's matchedDelta row: it
// composes a chord's own departure from the curve it chords (the certified
// sagitta, that table's sectionDelta row, read per cell or build-wide) with the
// displacement of the two held stations that chord actually joins (that table's
// delta row) into the single PARAMETER-MATCHED bound every chorded leg charges.
// Both terms are listed there with the quantity each bounds and the certified
// source each is read from.
//
// The two are accumulated apart — the generator publishes them apart and the
// payload carries them apart (loftPayload.sectionDelta and loftPayload.delta),
// which is the rule §5.2's table states for them — and this helper is only ever
// a consumer's own composition. Composing them is not optional for such a
// consumer: the sagitta bounds the IDEAL chord between the two points the record
// denotes, and the chord the build DREW joins two stations each displaced by
// delta from those points, so a leg charging the sagitta alone leaves the
// computed station's own displacement uncharged.
//
// That table owns the composition's derivation and its rounding direction, and
// this helper adds no mechanism of its own to either. What the code here does
// state is that both terms are read at the same s, which is what keeps the
// published bound PARAMETER-MATCHED (loftCellStations' own doc comment).
//
// Either term underivable answers +Inf, the answer §5.2's table assigns those
// rows, and the caller refuses on it.
func ChordCellDeltaUpper(sagittaUpper, deltaUpper float64) float64 {
	if proofbound.IsNonFinite(sagittaUpper) || proofbound.IsNonFinite(deltaUpper) {
		return math.Inf(1)
	}
	return proofbound.AbsSumUpper(sagittaUpper, deltaUpper)
}
