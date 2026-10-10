package sectionaudit

import (
	"fmt"
	"math"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

func renderCoord(c float64) string {
	if c == 0 {
		c = 0
	}
	return strconv.FormatFloat(c, 'g', -1, 64)
}

// Crossing tests every pair of segments for an interior crossing OR a
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
func Crossing(budget *proofbound.WorkBudget, segs []Entry) error {
	touchFloor, err := ContactFloor(budget, segs)
	if err != nil {
		return err
	}
	return CrossingWithFloor(budget, segs, touchFloor)
}

// CrossingWithFloor checks analytic entries against a section-scale contact
// floor supplied by a caller that also bounds free-form segments.
func CrossingWithFloor(budget *proofbound.WorkBudget, segs []Entry, touchFloor float64) error {
	for i := range segs {
		for j := i + 1; j < len(segs); j++ {
			if err := survey2d.WallBudgetStep(budget); err != nil {
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

// ContactEps is the noise-floor coefficient ε of the boundary-contact test,
// the SAME ε verification design §4 fixes for its diameter-anchored noise floor
// δ = ε·D (see appendClearance, the Clearance.Gap gate). It is not a
// distance; ContactFloor multiplies it by the section's own scale.
const ContactEps = 1e-9

const contactEps = ContactEps

// ContactFloor is the boundary-contact threshold for the §5 audit, anchored to
// the SECTION'S scale exactly as verification design §4 anchors a length's
// noise floor: δ = ε·D with ε = contactEps and D the section's diameter (its
// (u, v) bounding-box diagonal — the standard decad reading of D, as in
// export.STL and the boolean chord tolerance). Below δ two boundaries are
// indistinguishable from a pinch, so the test REFUSES; comfortably above it is
// a real positive gap that builds. The threshold is reject-only and SCALES with
// the section — a fixed absolute band mis-scales, rejecting a macroscopic gap
// on a sub-millimetre section and accepting a real pinch on a huge one.
func ContactFloor(budget *proofbound.WorkBudget, segs []Entry) (float64, error) {
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
func sectionBBoxBudget(budget *proofbound.WorkBudget, segs []Entry) (minU, minV, maxU, maxV float64, ok bool, err error) {
	minU, minV = math.Inf(1), math.Inf(1)
	maxU, maxV = math.Inf(-1), math.Inf(-1)
	fold := func(x, y float64) {
		minU, maxU = math.Min(minU, x), math.Max(maxU, x)
		minV, maxV = math.Min(minV, y), math.Max(maxV, y)
	}
	for _, s := range segs {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, 0, 0, 0, false, err
		}
		w := s.w
		fold(w.StartU, w.StartV)
		fold(w.EndU, w.EndV)
		if !w.IsCircular() {
			continue
		}
		lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
		for q := range 4 { // the four cardinal bearings 0, π/2, π, 3π/2
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return 0, 0, 0, 0, false, err
			}
			base := float64(q) * (math.Pi / 2)
			th := base + 2*math.Pi*math.Ceil((lo-base)/(2*math.Pi))
			for ; th <= hi+1e-12; th += 2 * math.Pi {
				fold(w.CU+w.Radius*math.Cos(th), w.CV+w.Radius*math.Sin(th))
			}
		}
	}
	if math.IsInf(minU, 1) {
		return 0, 0, 0, 0, false, nil
	}
	return minU, minV, maxU, maxV, true, nil
}

// Nesting is the §5 test-4 containment audit (S9): with S7 having proven
// the rewritten loops strictly disjoint, each hole lies wholly inside or wholly
// outside the outer loop and every other hole. It classifies one point of each
// hole against the outer loop's boundary and against every other hole's, using
// the ray-parity walk with direction retries internal/survey2d/wall_kernel.go already runs
// (loopContains, over survey2d.RayCrossings). The audit passes only when the outer loop
// is PROVEN to contain each hole and the holes are proven mutually exterior.
//
// An undecided classification is S9 ErrUnsupported — a build-time audit has no
// Suspect to fall back on, so the evaluator declines rather than guess. A hole
// PROVEN outside the outer loop, or PROVEN inside another hole, is nesting
// decidably broken: the fillet consumed the region the caller's nested section
// lived in, so no such body exists (§1 existence test) — an S8-family
// ErrDegenerate, the same "modification consumed the region" verdict S8 gives an
// inverted loop.
func Nesting(budget *proofbound.WorkBudget, segs []Entry, nLoops int) error {
	if nLoops <= 1 { // no holes: nothing to contain
		return nil
	}
	bounds := make([][]survey2d.SurveyElem, nLoops)
	pts := make([][2]float64, nLoops)
	hasPt := make([]bool, nLoops)
	for _, s := range segs {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		if e, ok := elemOf(s.w); ok {
			bounds[s.loop] = append(bounds[s.loop], e)
		}
		if !hasPt[s.loop] {
			pts[s.loop] = [2]float64{s.w.StartU, s.w.StartV}
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
		if err := survey2d.WallBudgetStep(budget); err != nil {
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
			if err := survey2d.WallBudgetStep(budget); err != nil {
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

// elemOf converts a segment walk into a survey2d boundary element. It is
// walkElem under this file's own name; the conversion lives in one place so a
// new walk kind is decided once (survey.go).
func elemOf(w survey2d.SegmentWalk) (survey2d.SurveyElem, bool) { return survey2d.WalkElem(w) }

// loopContains is the named boundary-scan phase used by cancellation probes.
func loopContains(budget *proofbound.WorkBudget, boundary []survey2d.SurveyElem, px, py, tol float64) (inside, decided bool, err error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return false, false, err
	}
	return loopContainsBudget(budget, boundary, px, py, tol)
}

// loopContainsBudget classifies (px, py) against a loop's boundary by crossing parity
// of a ray, retried across the golden-angle direction sequence when a crossing
// is ambiguous — the same walk survey2d.WallKernel.contains runs. decided is false when
// every direction is ambiguous; the answer is never guessed.
func loopContainsBudget(budget *proofbound.WorkBudget, boundary []survey2d.SurveyElem, px, py, tol float64) (inside, decided bool, err error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return false, false, err
	}
	for i := range 16 {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return false, false, err
		}
		th := 0.5 + float64(i)*2.399963229728653 // golden-angle sequence
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, good := 0, true
		for _, e := range boundary {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return false, false, err
			}
			n, ok := survey2d.RayCrossings(e, px, py, dx, dy, tol)
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
