package decad

import (
	"fmt"
	"math"
)

// This file is the §5 audit of a fillet's rewritten section
// (docs/modify-design.md §5), run before any face is built, in the order §4
// fixes: S8 (orientation — the existence question, asked first), S6 (no walk
// consumed by its own corners), S7 (no crossing OR boundary contact), then S9
// (nesting). Every test is a closed-form fact of decad's own line and arc
// segments, so its verdict is the same under every evaluator — never a residual.
//
// S7 rejects boundary CONTACT as well as crossing, so that the loops the
// rewrite hands S9 are strictly DISJOINT. Two Jordan loops stop being disjoint
// in exactly two ways: their boundaries CROSS, or they merely TOUCH (a
// tangency, or a shared boundary point with no interior crossing). A large
// fillet can pinch the rewritten loops into contact without crossing, so S7
// tests both (segCross for a transversal crossing, segMinDist for a tangency or
// shared point).
//
// Disjoint is not the same as nested: two disjoint Jordan loops are either
// nested OR mutually exterior, and S8 (each loop's own signed area) reads no
// relative position, so it cannot tell the two apart. A large fillet can shrink
// the outer loop past a near-corner hole, leaving the hole in the removed corner
// region — disjoint from every outer segment, yet OUTSIDE the rounded material.
// So S9 (nestingAuditBudget) is COMPUTED, not discharged by construction: it
// classifies one point of each hole against the outer loop and each other hole,
// using the same ray-parity walk with direction retries that survey2d.go runs
// (loopContains). An undecidable containment is S9 ErrUnsupported — the
// evaluator declines rather than guess; a hole PROVEN outside the outer loop, or
// nested inside another hole, is nesting decidably broken — the fillet consumed
// the region the caller's section lived in, so no such body exists and it is an
// S8-family ErrDegenerate (§1 existence test).

// auditDiagnosticError carries the coordinate-rich rendering used by Fillet and
// Chamfer while keeping the shared audit's established error text for Shell.
// The legacy error remains the unwrap chain, so callers can still branch on its
// sentinel regardless of which rendering they receive.
type auditDiagnosticError struct {
	legacy   error
	detailed string
}

func (e *auditDiagnosticError) Error() string {
	return e.legacy.Error()
}

func (e *auditDiagnosticError) Unwrap() error {
	return e.legacy
}

type renderedAuditDiagnosticError struct {
	cause   error
	message string
}

func (e *renderedAuditDiagnosticError) Error() string {
	return e.message
}

func (e *renderedAuditDiagnosticError) Unwrap() error {
	return e.cause
}

func auditError(legacy error, detailed string) error {
	return &auditDiagnosticError{legacy: legacy, detailed: detailed}
}

// renderAuditCoordinates opts a Fillet or Chamfer failure into the detailed
// diagnostic without changing the shared audit error seen by Shell.
func renderAuditCoordinates(err error) error {
	diagnostic, ok := err.(*auditDiagnosticError)
	if !ok {
		return err
	}
	return &renderedAuditDiagnosticError{cause: err, message: diagnostic.detailed}
}

// auditRewriteBudget runs the §5 audit on the rewritten profile in §4's order. It is
// shared by every modify op (Fillet, Chamfer): its only op-specific input is the
// per-corner cutback the S6 self-consuming-trim test sums, carried by the shared
// cornerBlend, so the audit itself forks nothing.
func auditRewriteBudget(budget *workBudget, orig, rewritten ProfileRecord, loops []cornerLoop, blendAt []map[int]*cornerBlend) error {
	origLoops := append([]LoopRecord{orig.Outer}, orig.Holes...)
	newLoops := append([]LoopRecord{rewritten.Outer}, rewritten.Holes...)

	// S6, computed up front so S8 can consult it: a walk whose two ends' cutbacks
	// reach or pass its far end is consumed by its own corners (§6, Table S). This
	// is a LOCAL fact — a cutback length against a walk length, needing no
	// assembled loop — and it covers BOTH shapes Table S names: two corners
	// claiming one wall from both ends, AND a single corner whose own cutback
	// reaches the far end of an adjacent walk. It is the more-specific reading of
	// an over-large setback, so when such an overrun ALSO flips the assembled
	// loop's signed area, the flip IS the overrun and reads S6 (ErrUnsupported),
	// not S8 — S8 still owns every genuine inversion a cutback overrun does not
	// explain.
	overrunWalk := make([]int, len(loops))
	overrun := make([]bool, len(loops))
	for li, cl := range loops {
		var err error
		overrunWalk[li], overrun[li], err = loopOverrunBudget(budget, cl, blendAt[li])
		if err != nil {
			return err
		}
	}

	// S8: orientation preserved — a loop whose signed area changed sign (or
	// collapsed) has turned itself inside out; the modification consumed it —
	// unless a local cutback overrun on that loop explains the flip, which is
	// S6's more-specific verdict (Table S: an overrun is ErrUnsupported even when
	// it also flips the loop).
	for i := range origLoops {
		oa, err := loopSignedAreaBudget(budget, origLoops[i])
		if err != nil {
			return auditError(err, fmt.Sprintf(`original loop %d: %v`, i, err))
		}
		na, err := loopSignedAreaBudget(budget, newLoops[i])
		if err != nil {
			return auditError(err, fmt.Sprintf(`rewritten loop %d: %v`, i, err))
		}
		if math.Signbit(oa) != math.Signbit(na) || math.Abs(na) <= 1e-9*math.Abs(oa) {
			if overrun[i] {
				return errCutbackOverrun(i, overrunWalk[i], loops[i])
			}
			legacy := fmt.Errorf(`%w: the rewrite turned a loop inside out — the modification does not fit`, ErrDegenerate)
			return auditError(legacy,
				fmt.Sprintf(`%v: rewritten loop %d turned inside out — the modification does not fit`, ErrDegenerate, i))
		}
	}

	// S6: no walk consumed by its own corners — reported for the loops S8 did not
	// already resolve (an overrun that did not flip the loop's signed area).
	for li := range loops {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		if overrun[li] {
			return errCutbackOverrun(li, overrunWalk[li], loops[li])
		}
	}

	// S7 and S9 both work on the rewritten loops as boundary primitives, so
	// resolve every segment's walk once and share it.
	segs, err := buildSegEntriesBudget(budget, newLoops)
	if err != nil {
		return err
	}

	// S7: no crossing AND no boundary contact — any pair of non-adjacent
	// segments meeting in both their interiors is a self-intersection, and a
	// pair that merely touches (a tangency, or a shared boundary point) is a
	// pinch; either is a rewrite a resolving kernel would have to trim.
	if err := crossingAuditBudget(budget, segs); err != nil {
		return err
	}

	// S9: nesting preserved — with the loops proven disjoint by S7, each hole
	// lies wholly inside or wholly outside the outer loop and every other hole;
	// classify one point of each to prove the outer loop still contains every
	// hole and the holes stay mutually exterior.
	return nestingAuditBudget(budget, segs, len(newLoops))
}

// loopOverrunBudget reports the first walk consumed by its own two corners and whether
// one was found. The arriving corner's cutback plus the leaving corner's
// cutback reaches or passes the walk's far end (§6, S6). It is a LOCAL test — a
// cutback length against a walk length — so it needs no assembled loop, which
// is what lets S8 consult it before reading the loop's orientation: a flip a
// single over-large corner produces is this overrun (ErrUnsupported), not a
// genuinely inside-out section (ErrDegenerate). The sum folds both Table S
// shapes: a lone corner leaves one end's cutback zero, so its own cutback alone
// must clear the walk; two corners of a short wall claim it from both ends.
func loopOverrunBudget(budget *workBudget, cl cornerLoop, blends map[int]*cornerBlend) (int, bool, error) {
	n := len(cl.walks)
	for i, w := range cl.walks {
		if err := wallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		cut := 0.0
		if cb := blends[i]; cb != nil {
			cut += cb.cutbackB
		}
		if cb := blends[(i+1)%n]; cb != nil {
			cut += cb.cutbackA
		}
		if cut >= w.length-1e-9*math.Max(1, w.length) {
			return i, true, nil
		}
	}
	return 0, false, nil
}

// errCutbackOverrun is S6's op-neutral refusal (§6, Table S): a corner's setback
// reaches or passes the far end of an adjacent wall, so the rewrite's pieces
// must be resolved against each other before they bound anything — a body a
// trimmed-offset kernel could build but this evaluator cannot (ErrUnsupported).
func errCutbackOverrun(loop, walk int, cl cornerLoop) error {
	w := cl.walks[walk]
	next := (walk + 1) % len(cl.walks)
	legacy := fmt.Errorf(`%w: a corner's setback reaches the far end of an adjacent wall; merging the rewrites there is not supported`, ErrUnsupported)
	detailed := fmt.Sprintf(`%v: loop %d walk %d from corner %d at (u, v) = (%s, %s) to corner %d at (u, v) = (%s, %s) is consumed by its corner setbacks; merging the rewrites there is not supported`,
		ErrUnsupported, loop, walk, walk, renderCoord(w.startU), renderCoord(w.startV),
		next, renderCoord(w.endU), renderCoord(w.endV))
	return auditError(legacy, detailed)
}

// buildSegEntries resolves every loop's recorded segments into boundary walks
// tagged by the loop and position they came from (for adjacency).
func buildSegEntries(loops []LoopRecord) ([]segEntry, error) {
	return buildSegEntriesBudget(nil, loops)
}

func buildSegEntriesBudget(budget *workBudget, loops []LoopRecord) ([]segEntry, error) {
	var segs []segEntry
	for li, loop := range loops {
		n := len(loop.Segments)
		for i, seg := range loop.Segments {
			if err := wallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := walkOf(seg)
			if err != nil {
				return nil, auditError(err, fmt.Sprintf(`loop %d segment %d: %v`, li, i, err))
			}
			segs = append(segs, segEntry{loop: li, idx: i, n: n, w: w})
		}
	}
	return segs, nil
}

// loopSignedAreaBudget is one loop's signed area (positive counter-clockwise): the
// Green's-theorem boundary integral of its own segments.
func loopSignedAreaBudget(budget *workBudget, loop LoopRecord) (float64, error) {
	var ig regionIntegrals
	for _, seg := range loop.Segments {
		if err := wallBudgetStep(budget); err != nil {
			return 0, err
		}
		// Integrated about the plane origin itself: this audit compares one
		// loop's own signed area against zero, so it needs no walk anchor.
		if err := ig.add(seg, Point2{}); err != nil {
			return 0, err
		}
	}
	return ig.area, nil
}

// segEntry is one rewritten segment as a boundary primitive, tagged by the loop
// and position it came from (for adjacency).
type segEntry struct {
	loop int
	idx  int
	n    int // segments in this loop
	w    segmentWalk
}

// crossingAuditBudget tests every pair of segments for an interior crossing OR a
// boundary contact — a tangency or a shared boundary point — skipping only
// pairs that legitimately share an endpoint (same-loop neighbours). A crossing
// and a contact both stop two Jordan loops being disjoint, so both must be
// rejected before S9's nesting can be tested: S9 classifies a point of one loop
// against another, and that classification is only defined once the loops are
// strictly disjoint. An interior-only crossing test would pass a large fillet
// whose rewritten loops merely pinch, and Body.Tessellate would then refuse the
// very body Fillet returned. Both are S7 (ErrUnsupported): the touch is the
// boundary case of a crossing, a body a resolving/trimmed-offset kernel could
// build but this evaluator cannot (§1 existence test — the body exists; §4
// Table S, S7).
func crossingAuditBudget(budget *workBudget, segs []segEntry) error {
	touchFloor, err := contactFloorBudget(budget, segs)
	if err != nil {
		return err
	}
	for i := range segs {
		for j := i + 1; j < len(segs); j++ {
			if err := wallBudgetStep(budget); err != nil {
				return err
			}
			if adjacent(segs[i], segs[j]) {
				continue
			}
			if segCross(segs[i].w, segs[j].w) {
				legacy := fmt.Errorf(`%w: the rewrite crosses itself; a resolving kernel is not available`, ErrUnsupported)
				detailed := fmt.Sprintf(`%v: rewritten loop %d segment %d and loop %d segment %d cross; a resolving kernel is not available`,
					ErrUnsupported, segs[i].loop, segs[i].idx, segs[j].loop, segs[j].idx)
				return auditError(legacy, detailed)
			}
			if segMinDist(segs[i].w, segs[j].w) <= touchFloor {
				legacy := fmt.Errorf(`%w: the rewrite brings two boundaries into contact; a resolving kernel is not available`, ErrUnsupported)
				detailed := fmt.Sprintf(`%v: rewritten loop %d segment %d and loop %d segment %d are in contact; a resolving kernel is not available`,
					ErrUnsupported, segs[i].loop, segs[i].idx, segs[j].loop, segs[j].idx)
				return auditError(legacy, detailed)
			}
		}
	}
	return nil
}

// contactEps is the noise-floor coefficient ε of the boundary-contact test,
// the SAME ε verification design §4 fixes for its diameter-anchored noise floor
// δ = ε·D (see gapWithinTolerance, the Clearance.Gap gate). It is not a
// distance; contactFloorBudget multiplies it by the section's own scale.
const contactEps = 1e-9

// contactFloorBudget is the boundary-contact threshold for the §5 audit, anchored to
// the SECTION'S scale exactly as verification design §4 anchors a length's
// noise floor: δ = ε·D with ε = contactEps and D the section's diameter (its
// (u, v) bounding-box diagonal — the standard decad reading of D, as in
// Body.STL and the boolean chord tolerance). Below δ two boundaries are
// indistinguishable from a pinch, so the test REFUSES; comfortably above it is
// a real positive gap that builds. The threshold is reject-only and SCALES with
// the section — a fixed absolute band mis-scales, rejecting a macroscopic gap
// on a sub-millimetre section and accepting a real pinch on a huge one.
func contactFloorBudget(budget *workBudget, segs []segEntry) (float64, error) {
	minU, minV, maxU, maxV, ok, err := sectionBBoxBudget(budget, segs)
	if err != nil {
		return 0, err
	}
	if !ok {
		return 0, nil
	}
	return contactEps * math.Hypot(maxU-minU, maxV-minV), nil
}

// sectionBBoxBudget is the TRUE (u, v) bounding box of a rewritten section — the box
// the §5 D reads. An arc bulges outside its endpoint chord (a semicircle from
// (−R,0) to (R,0) reaches (0,R); a full circle's endpoints collapse to a
// point), so a line contributes its two endpoints and an arc its endpoints AND
// every cardinal extremum (cU±R, cV±R) its own angular walk actually reaches —
// never the endpoint box alone, which understates D.
func sectionBBoxBudget(budget *workBudget, segs []segEntry) (minU, minV, maxU, maxV float64, ok bool, err error) {
	minU, minV = math.Inf(1), math.Inf(1)
	maxU, maxV = math.Inf(-1), math.Inf(-1)
	fold := func(x, y float64) {
		minU, maxU = math.Min(minU, x), math.Max(maxU, x)
		minV, maxV = math.Min(minV, y), math.Max(maxV, y)
	}
	for _, s := range segs {
		if err := wallBudgetStep(budget); err != nil {
			return 0, 0, 0, 0, false, err
		}
		w := s.w
		fold(w.startU, w.startV)
		fold(w.endU, w.endV)
		if !w.circular {
			continue
		}
		lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
		for q := range 4 { // the four cardinal bearings 0, π/2, π, 3π/2
			if err := wallBudgetStep(budget); err != nil {
				return 0, 0, 0, 0, false, err
			}
			base := float64(q) * (math.Pi / 2)
			th := base + 2*math.Pi*math.Ceil((lo-base)/(2*math.Pi))
			for ; th <= hi+1e-12; th += 2 * math.Pi {
				fold(w.cU+w.radius*math.Cos(th), w.cV+w.radius*math.Sin(th))
			}
		}
	}
	if math.IsInf(minU, 1) {
		return 0, 0, 0, 0, false, nil
	}
	return minU, minV, maxU, maxV, true, nil
}

// nestingAuditBudget is the §5 test-4 containment audit (S9): with S7 having proven
// the rewritten loops strictly disjoint, each hole lies wholly inside or wholly
// outside the outer loop and every other hole. It classifies one point of each
// hole against the outer loop's boundary and against every other hole's, using
// the ray-parity walk with direction retries survey2d.go already runs
// (loopContains, over rayCrossings). The audit passes only when the outer loop
// is PROVEN to contain each hole and the holes are proven mutually exterior.
//
// An undecided classification is S9 ErrUnsupported — a build-time audit has no
// Suspect to fall back on, so the evaluator declines rather than guess. A hole
// PROVEN outside the outer loop, or PROVEN inside another hole, is nesting
// decidably broken: the fillet consumed the region the caller's nested section
// lived in, so no such body exists (§1 existence test) — an S8-family
// ErrDegenerate, the same "modification consumed the region" verdict S8 gives an
// inverted loop.
func nestingAuditBudget(budget *workBudget, segs []segEntry, nLoops int) error {
	if nLoops <= 1 { // no holes: nothing to contain
		return nil
	}
	bounds := make([][]surveyElem, nLoops)
	pts := make([][2]float64, nLoops)
	hasPt := make([]bool, nLoops)
	for _, s := range segs {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		if e, ok := elemOf(s.w); ok {
			bounds[s.loop] = append(bounds[s.loop], e)
		}
		if !hasPt[s.loop] {
			pts[s.loop] = [2]float64{s.w.startU, s.w.startV}
			hasPt[s.loop] = true
		}
	}
	minU, minV, maxU, maxV, ok, err := sectionBBoxBudget(budget, segs)
	if err != nil {
		return err
	}
	if !ok {
		return nil
	}
	scale := math.Max(1, math.Max(math.Max(math.Abs(minU), math.Abs(maxU)), math.Max(math.Abs(minV), math.Abs(maxV))))
	tol := contactEps * scale

	undecidable := func(container, pointLoop int, point [2]float64) error {
		legacy := fmt.Errorf(`%w: the rewrite's nesting cannot be decided; a resolving kernel is not available`, ErrUnsupported)
		detailed := fmt.Sprintf(`%v: nesting of loop %d and loop %d at (u, v) = (%s, %s) cannot be decided; a resolving kernel is not available`,
			ErrUnsupported, container, pointLoop, renderCoord(point[0]), renderCoord(point[1]))
		return auditError(legacy, detailed)
	}
	// Every hole must sit inside the rewritten outer loop.
	for h := 1; h < nLoops; h++ {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		if !hasPt[h] {
			continue
		}
		inside, decided, err := loopContains(budget, bounds[0], pts[h][0], pts[h][1], tol)
		if err != nil {
			return err
		}
		if !decided {
			return undecidable(0, h, pts[h])
		}
		if !inside {
			legacy := fmt.Errorf(`%w: the rewrite left a hole outside the outer loop, consuming the region it lived in`, ErrDegenerate)
			detailed := fmt.Sprintf(`%v: hole loop %d at (u, v) = (%s, %s) lies outside outer loop 0, consuming the region it lived in`,
				ErrDegenerate, h, renderCoord(pts[h][0]), renderCoord(pts[h][1]))
			return auditError(legacy, detailed)
		}
	}
	// Holes must stay mutually exterior — neither nested in the other.
	for a := 1; a < nLoops; a++ {
		for b := a + 1; b < nLoops; b++ {
			if err := wallBudgetStep(budget); err != nil {
				return err
			}
			if !hasPt[a] || !hasPt[b] {
				continue
			}
			for _, pr := range [2]struct {
				in int
				pt [2]float64
			}{{a, pts[b]}, {b, pts[a]}} {
				inside, decided, err := loopContains(budget, bounds[pr.in], pr.pt[0], pr.pt[1], tol)
				if err != nil {
					return err
				}
				if !decided {
					pointLoop := a
					if pr.in == a {
						pointLoop = b
					}
					return undecidable(pr.in, pointLoop, pr.pt)
				}
				if inside {
					pointLoop := a
					if pr.in == a {
						pointLoop = b
					}
					legacy := fmt.Errorf(`%w: the rewrite nested one hole inside another`, ErrDegenerate)
					detailed := fmt.Sprintf(`%v: hole loop %d at (u, v) = (%s, %s) lies inside hole loop %d`,
						ErrDegenerate, pointLoop, renderCoord(pr.pt[0]), renderCoord(pr.pt[1]), pr.in)
					return auditError(legacy, detailed)
				}
			}
		}
	}
	return nil
}

// elemOf converts a segment walk into a survey2d boundary element (the material
// side is irrelevant to ray parity, so an arc's walk sense is not consulted).
func elemOf(w segmentWalk) (surveyElem, bool) {
	if w.circular {
		return arcElem(w.cU, w.cV, w.radius, w.th0, w.th1, w.closed)
	}
	return lineElem(w.startU, w.startV, w.endU, w.endV)
}

// loopContains is the named boundary-scan phase used by cancellation probes.
func loopContains(budget *workBudget, boundary []surveyElem, px, py, tol float64) (inside, decided bool, err error) {
	if err := wallBudgetErr(budget); err != nil {
		return false, false, err
	}
	return loopContainsBudget(budget, boundary, px, py, tol)
}

// loopContainsBudget classifies (px, py) against a loop's boundary by crossing parity
// of a ray, retried across the golden-angle direction sequence when a crossing
// is ambiguous — the same walk wallKernel.contains runs. decided is false when
// every direction is ambiguous; the answer is never guessed.
func loopContainsBudget(budget *workBudget, boundary []surveyElem, px, py, tol float64) (inside, decided bool, err error) {
	if err := wallBudgetErr(budget); err != nil {
		return false, false, err
	}
	for i := range 16 {
		if err := wallBudgetStep(budget); err != nil {
			return false, false, err
		}
		th := 0.5 + float64(i)*2.399963229728653 // golden-angle sequence
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, good := 0, true
		for _, e := range boundary {
			if err := wallBudgetStep(budget); err != nil {
				return false, false, err
			}
			n, ok := rayCrossings(e, px, py, dx, dy, tol)
			if !ok {
				good = false
				break
			}
			crossings += n
		}
		if good {
			return crossings%2 == 1, true, nil
		}
	}
	return false, false, nil
}

// adjacent reports whether two segments legitimately share an endpoint: the
// same loop and cyclically consecutive positions.
func adjacent(a, b segEntry) bool {
	if a.loop != b.loop {
		return false
	}
	d := a.idx - b.idx
	if d < 0 {
		d = -d
	}
	return d == 1 || d == a.n-1
}

// segCrossEps is the interior margin for the crossing test: an intersection
// within it of a segment's endpoint is a shared vertex, not a crossing.
const segCrossEps = 1e-7

// segCross reports whether two segment primitives meet in both their interiors.
func segCross(a, b segmentWalk) bool {
	switch {
	case !a.circular && !b.circular:
		return lineLineSegCross(a, b)
	case a.circular && b.circular:
		return arcArcSegCross(a, b)
	case a.circular:
		return lineArcSegCross(b, a)
	default:
		return lineArcSegCross(a, b)
	}
}

// lineLineSegCross reports an interior crossing of two line segments.
func lineLineSegCross(a, b segmentWalk) bool {
	rx, ry := a.endU-a.startU, a.endV-a.startV
	sx, sy := b.endU-b.startU, b.endV-b.startV
	den := rx*sy - ry*sx
	if math.Abs(den) <= filletTol {
		return false // parallel: no transversal crossing
	}
	qpx, qpy := b.startU-a.startU, b.startV-a.startV
	t := (qpx*sy - qpy*sx) / den
	u := (qpx*ry - qpy*rx) / den
	return interior(t) && interior(u)
}

// lineArcSegCross reports an interior crossing of a line segment and an arc.
func lineArcSegCross(line, arc segmentWalk) bool {
	dx, dy := line.endU-line.startU, line.endV-line.startV
	dl := math.Hypot(dx, dy)
	if dl <= filletTol {
		return false
	}
	ux, uy := dx/dl, dy/dl
	fx, fy := line.startU-arc.cU, line.startV-arc.cV
	bb := fx*ux + fy*uy
	cc := fx*fx + fy*fy - arc.radius*arc.radius
	disc := bb*bb - cc
	if disc < 0 {
		return false
	}
	sq := math.Sqrt(disc)
	for _, s := range []float64{-bb + sq, -bb - sq} {
		x, y := line.startU+s*ux, line.startV+s*uy
		t := s / dl
		if !interior(t) {
			continue
		}
		if angleInterior(arc, x, y) {
			return true
		}
	}
	return false
}

// arcArcSegCross reports an interior crossing of two arcs.
func arcArcSegCross(a, b segmentWalk) bool {
	pts := circleCircle(a.cU, a.cV, a.radius, b.cU, b.cV, b.radius)
	for _, p := range pts {
		if angleInterior(a, p[0], p[1]) && angleInterior(b, p[0], p[1]) {
			return true
		}
	}
	return false
}

// interior reports whether a segment parameter is strictly inside (0, 1).
func interior(t float64) bool { return t > segCrossEps && t < 1-segCrossEps }

// angleInterior reports whether the point (x, y) lies strictly inside the arc's
// angular walk range.
func angleInterior(arc segmentWalk, x, y float64) bool {
	lo, hi := math.Min(arc.th0, arc.th1), math.Max(arc.th0, arc.th1)
	a := math.Atan2(y-arc.cV, x-arc.cU)
	for k := math.Floor((lo-a)/(2*math.Pi)) * 2 * math.Pi; a+k <= hi+segCrossEps; k += 2 * math.Pi {
		th := a + k
		if th > lo+segCrossEps && th < hi-segCrossEps {
			return true
		}
	}
	return false
}

// angleWithin reports whether (x, y) lies within the arc's angular walk range,
// inclusive of its endpoints — the membership the minimum-distance candidates
// need (a nearest point may sit at an arc's own end).
func angleWithin(arc segmentWalk, x, y float64) bool {
	lo, hi := math.Min(arc.th0, arc.th1), math.Max(arc.th0, arc.th1)
	a := math.Atan2(y-arc.cV, x-arc.cU)
	for k := math.Floor((lo-a)/(2*math.Pi)) * 2 * math.Pi; a+k <= hi+segCrossEps; k += 2 * math.Pi {
		th := a + k
		if th >= lo-segCrossEps && th <= hi+segCrossEps {
			return true
		}
	}
	return false
}

// segMinDist is the minimum Euclidean distance between two closed segment
// primitives (line or arc), in closed form over their own line and arc data. It
// is the boundary-contact classifier behind S7: a value at or below the
// section-scaled contactFloorBudget is a tangency or a shared boundary point the
// interior-only crossing test misses. The candidate set is complete for the attained infimum over line/arc
// boundaries — the four endpoint-against-the-other distances, the interior
// radial/aligned criticals, and zero at any interior intersection.
func segMinDist(a, b segmentWalk) float64 {
	switch {
	case !a.circular && !b.circular:
		return lineLineMinDist(a, b)
	case a.circular && b.circular:
		return arcArcMinDist(a, b)
	case a.circular:
		return lineArcMinDist(b, a)
	default:
		return lineArcMinDist(a, b)
	}
}

// pointLineSegDist is the distance from (px, py) to the closed line segment.
func pointLineSegDist(px, py float64, l segmentWalk) float64 {
	dx, dy := l.endU-l.startU, l.endV-l.startV
	l2 := dx*dx + dy*dy
	if l2 <= filletTol*filletTol {
		return math.Hypot(px-l.startU, py-l.startV)
	}
	t := ((px-l.startU)*dx + (py-l.startV)*dy) / l2
	t = math.Max(0, math.Min(1, t))
	return math.Hypot(px-(l.startU+t*dx), py-(l.startV+t*dy))
}

// pointArcDist is the distance from (px, py) to the closed arc: the radial gap
// when the point's bearing falls within the walk range, else the nearer of the
// two arc endpoints.
func pointArcDist(px, py float64, a segmentWalk) float64 {
	best := math.Min(math.Hypot(px-a.startU, py-a.startV), math.Hypot(px-a.endU, py-a.endV))
	dc := math.Hypot(px-a.cU, py-a.cV)
	if dc > filletTol && angleWithin(a, px, py) {
		best = math.Min(best, math.Abs(dc-a.radius))
	}
	return best
}

// lineLineMinDist is the minimum distance between two closed line segments:
// zero where their interiors cross, else the nearest endpoint-to-segment reach.
func lineLineMinDist(a, b segmentWalk) float64 {
	if lineLineSegCross(a, b) {
		return 0
	}
	return math.Min(
		math.Min(pointLineSegDist(a.startU, a.startV, b), pointLineSegDist(a.endU, a.endV, b)),
		math.Min(pointLineSegDist(b.startU, b.startV, a), pointLineSegDist(b.endU, b.endV, a)),
	)
}

// lineArcMinDist is the minimum distance between a closed line segment and a
// closed arc.
func lineArcMinDist(line, arc segmentWalk) float64 {
	if lineArcSegCross(line, arc) {
		return 0
	}
	best := math.Min(
		math.Min(pointArcDist(line.startU, line.startV, arc), pointArcDist(line.endU, line.endV, arc)),
		math.Min(pointLineSegDist(arc.startU, arc.startV, line), pointLineSegDist(arc.endU, arc.endV, line)),
	)
	// Interior critical: the arc point along the perpendicular from the centre
	// to the line, when its foot is interior to the segment and its bearing is
	// within the walk — the tangency/near-approach the endpoints miss.
	dx, dy := line.endU-line.startU, line.endV-line.startV
	l2 := dx*dx + dy*dy
	if l2 > filletTol*filletTol {
		s := ((arc.cU-line.startU)*dx + (arc.cV-line.startV)*dy) / l2
		if s > 0 && s < 1 {
			fx, fy := line.startU+s*dx, line.startV+s*dy
			ux, uy := fx-arc.cU, fy-arc.cV
			ul := math.Hypot(ux, uy)
			if ul > filletTol {
				cpx, cpy := arc.cU+arc.radius*ux/ul, arc.cV+arc.radius*uy/ul
				if angleWithin(arc, cpx, cpy) {
					best = math.Min(best, pointLineSegDist(cpx, cpy, line))
				}
			}
		}
	}
	return best
}

// arcArcMinDist is the minimum distance between two closed arcs.
func arcArcMinDist(a, b segmentWalk) float64 {
	if arcArcSegCross(a, b) {
		return 0
	}
	best := math.Min(
		math.Min(pointArcDist(a.startU, a.startV, b), pointArcDist(a.endU, a.endV, b)),
		math.Min(pointArcDist(b.startU, b.startV, a), pointArcDist(b.endU, b.endV, a)),
	)
	dcx, dcy := b.cU-a.cU, b.cV-a.cV
	d := math.Hypot(dcx, dcy)
	if d > filletTol {
		ux, uy := dcx/d, dcy/d
		// The mutually nearest points of two non-concentric circles lie on the
		// centre line; test each circle's two axis points against the other arc,
		// which captures external and internal tangency alike.
		for _, s := range []float64{1, -1} {
			pax, pay := a.cU+s*a.radius*ux, a.cV+s*a.radius*uy
			if angleWithin(a, pax, pay) {
				best = math.Min(best, pointArcDist(pax, pay, b))
			}
			pbx, pby := b.cU+s*b.radius*ux, b.cV+s*b.radius*uy
			if angleWithin(b, pbx, pby) {
				best = math.Min(best, pointArcDist(pbx, pby, a))
			}
		}
	} else if arcSpansOverlap(a, b) {
		// Concentric arcs whose walks overlap in bearing: the radial gap.
		best = math.Min(best, math.Abs(a.radius-b.radius))
	}
	return best
}

// arcSpansOverlap reports whether two concentric arcs' walk ranges share any
// bearing — an endpoint of one falling within the other's range.
func arcSpansOverlap(a, b segmentWalk) bool {
	return angleWithin(b, a.startU, a.startV) || angleWithin(b, a.endU, a.endV) ||
		angleWithin(a, b.startU, b.startV) || angleWithin(a, b.endU, b.endV)
}
