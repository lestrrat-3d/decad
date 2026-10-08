package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file orders the §5 audit of a fillet's rewritten section
// (docs/modify-design.md §5) over internal/sectionaudit/, before any face is
// built, in the order §4 fixes: S8 (orientation, asked first), S6 (no walk
// consumed by its own corners), S7 (no crossing or boundary contact), then S9
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
// using the same ray-parity walk with direction retries that internal/survey2d/wall_kernel.go runs
// (loopContains). An undecidable containment is S9 ErrUnsupported — the
// evaluator declines rather than guess; a hole PROVEN outside the outer loop, or
// nested inside another hole, is nesting decidably broken — the fillet consumed
// the region the caller's section lived in, so no such body exists and it is an
// S8-family ErrDegenerate (§1 existence test).

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
	return sectionaudit.NewError(legacy, detailed)
}

// renderAuditCoordinates opts a Fillet or Chamfer failure into the detailed
// diagnostic without changing the shared audit error seen by Shell.
func renderAuditCoordinates(err error) error {
	diagnostic, ok := err.(*sectionaudit.DiagnosticError)
	if !ok {
		return err
	}
	return &renderedAuditDiagnosticError{cause: err, message: diagnostic.Detailed()}
}

// auditRewriteBudget runs the §5 audit on the rewritten profile in §4's order. It is
// shared by every modify op (Fillet, Chamfer): its only op-specific input is the
// per-corner cutback the S6 self-consuming-trim test sums, carried by the shared
// cornerBlend, so the audit itself forks nothing.
func auditRewriteBudget(budget *proofbound.WorkBudget, orig, rewritten ProfileRecord, loops []cornerLoop, blendAt []map[int]*cornerBlend) error {
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
		if err := survey2d.WallBudgetStep(budget); err != nil {
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
func loopOverrunBudget(budget *proofbound.WorkBudget, cl cornerLoop, blends map[int]*cornerBlend) (int, bool, error) {
	n := len(cl.walks)
	for i, w := range cl.walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		cut := 0.0
		if cb := blends[i]; cb != nil {
			cut += cb.CutbackB
		}
		if cb := blends[(i+1)%n]; cb != nil {
			cut += cb.CutbackA
		}
		if cut >= w.Length-1e-9*math.Max(1, w.Length) {
			return i, true, nil
		}
	}
	return 0, false, nil
}

// auditTrimmedWall is S6 for a swept brep face route E trims
// (docs/brep-modify-design.md §5.3 step 6): the claims a blend makes on the
// wall's start and end, each the cutback its end face's corner took on the
// same carrier, must sum strictly below the wall's walk length, by
// loopOverrunBudget's test. A wall they reach or pass is consumed by its own
// blends, ErrUnsupported as errCutbackOverrun is.
func auditTrimmedWall(w survey2d.SegmentWalk, claimStart, claimEnd float64) error {
	if claimStart+claimEnd < w.Length-1e-9*math.Max(1, w.Length) {
		return nil
	}
	legacy := fmt.Errorf(`%w: a corner's setback reaches the far end of an adjacent wall; merging the rewrites there is not supported`, ErrUnsupported)
	detailed := fmt.Sprintf(`%v: the swept wall from (u, v) = (%s, %s) to (%s, %s) is consumed by its blends' setbacks; merging the rewrites there is not supported`,
		ErrUnsupported, renderCoord(w.StartU), renderCoord(w.StartV), renderCoord(w.EndU), renderCoord(w.EndV))
	return auditError(legacy, detailed)
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
		ErrUnsupported, loop, walk, walk, renderCoord(w.StartU), renderCoord(w.StartV),
		next, renderCoord(w.EndU), renderCoord(w.EndV))
	return auditError(legacy, detailed)
}

// buildSegEntries resolves every loop's recorded segments into boundary walks
// tagged by the loop and position they came from (for adjacency).
func buildSegEntries(loops []LoopRecord) ([]segEntry, error) {
	return buildSegEntriesBudget(nil, loops)
}

func buildSegEntriesBudget(budget *proofbound.WorkBudget, loops []LoopRecord) ([]segEntry, error) {
	// One free-form counter for the whole audited section: the loops handed here
	// are one record, and no preflight has run on them.
	work := freeform.NewFreeformWork()
	var segs []segEntry
	for li, loop := range loops {
		n := len(loop.Segments)
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, auditError(err, fmt.Sprintf(`loop %d segment %d: %v`, li, i, err))
			}
			if err := requireAnalyticWalk(w, "the section audit"); err != nil {
				return nil, auditError(err, fmt.Sprintf(`loop %d segment %d: %v`, li, i, err))
			}
			segs = append(segs, segEntry{loop: li, idx: i, n: n, w: w})
		}
	}
	return segs, nil
}

// segEntry is one rewritten segment tagged with its loop position.
type segEntry struct {
	loop int
	idx  int
	n    int
	w    survey2d.SegmentWalk
}

const contactEps = sectionaudit.ContactEps

func auditEntries(segs []segEntry) []sectionaudit.Entry {
	entries := make([]sectionaudit.Entry, len(segs))
	for i, seg := range segs {
		entries[i] = sectionaudit.NewEntry(seg.loop, seg.idx, seg.n, seg.w)
	}
	return entries
}

func loopSignedAreaBudget(budget *proofbound.WorkBudget, loop LoopRecord) (float64, error) {
	return sectionaudit.LoopSignedArea(budget, loop)
}

func crossingAuditBudget(budget *proofbound.WorkBudget, segs []segEntry) error {
	return sectionaudit.Crossing(budget, auditEntries(segs))
}

func contactFloorBudget(budget *proofbound.WorkBudget, segs []segEntry) (float64, error) {
	return sectionaudit.ContactFloor(budget, auditEntries(segs))
}

func nestingAuditBudget(budget *proofbound.WorkBudget, segs []segEntry, nLoops int) error {
	return sectionaudit.Nesting(budget, auditEntries(segs), nLoops)
}
