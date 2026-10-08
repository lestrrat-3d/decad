package loftmesh

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

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
func WallTriangleArea(u, v proof.Xpt) (float64, float64) {
	w := proof.Xcross(u, v)
	q := proof.XdotRat(w, w)
	q.Quo(q, big.NewRat(4, 1))
	return proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
}

// LoftChordedAllow bundles the exact volume and first-moment corrections,
// the unsigned twist measure retained for tessellation's occupied-volume
// proof, the per-cell wall and skirt volume legs, and the wall's two-leg area
// residual (docs/loft-design.md §5.2/§8, docs/loft-gear-bounds-design.md §2).
// Every field of the zero value is 0, the correct standing for a LineSeg-only
// loft that never calls ComputeLoftChordedAllow at all.
type LoftChordedAllow struct {
	// WallAreaUpper is Σ proofbound.CellChordCurveAreaUpper over the charged
	// cells. No measurement reads it: WallLeg charges each cell's own area at
	// its own departure, and the tests compare it with the build-wide
	// matchedDelta × WallAreaUpper form.
	WallAreaUpper float64
	// WallLeg is docs/loft-gear-bounds-design.md §2's per-cell wall leg:
	// Σ proofbound.ProductUpper(cellMatched_k, cellWallUpper_k) over the charged
	// cells, each charged at its own matched departure rather than the
	// build-wide maximum. It bounds the measure the charged cells sweep as
	// they move from the bilinear patch at the held corners to the true wall.
	WallLeg float64
	// SkirtLeg is §2's skirt between the held seam and its projection onto the
	// cap plane, productUpper(productUpper(matchedDelta, delta),
	// seamPerimeterUpper) with the perimeter summed over EVERY seam cell, each
	// side the larger of its arc-length upper bound and its held chord's exact
	// length, and exactly 0 at delta == 0.
	SkirtLeg              float64
	TwistVolumeUpper      float64
	TwistVolumeCorrection *big.Rat
	TwistMomentCorrection proofbound.RatV3
	MaxTwistOffsetUpper   float64
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
	// CapAreaExcess is the per-cell cap tube over both caps
	// (docs/loft-gear-bounds-design.md §4): how far each cap's held chord
	// polygon can differ in AREA from the region its recorded boundary
	// denotes, Area()'s own cap term.
	CapAreaExcess float64
}

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
