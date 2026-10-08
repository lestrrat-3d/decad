package decad

import (
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is the receiver-side half of the undercut survey's fu155 fix
// (docs/verification-design.md §6, docs/modify-reach-design.md §12 Table DX
// row DX7): the exact three-valued reader for a body's own unchanged walls
// and caps, which prismUndercuts, cupUndercuts and capBlendUndercuts's
// receiver-wall loop all now share.
//
// Before this file, a receiver face's normal-component range was read as a
// bare float (survey.go's now-removed wallNormalRange) and handed to
// survey2d.OpposesPull's strict `m < 0 && M > -1` test — sound only where floating
// point lands exactly where the test expects, which a genuinely
// perpendicular or antiparallel face need not do (fu155's own repro:
// -4.6811112914356013e-17 and -0.99999999999999989 for faces whose exact
// components are 0 and -1). The cap-blend patch loop already carried a
// bounded three-valued answer; this file brings the receiver's own faces to
// the same standard, over the rationals rather than a float allowance,
// because both carve-outs are exactly decidable from held numbers alone: a
// straight wall's plane-local tangent, a circular wall's swept angle, and the
// placed frame's own exact directions (survey2d.NewPlacedFrameMap,
// capblend_normal.go).
//
// The caller's ORIGINAL pull is used throughout, never its normalized form:
// normalizing rounds, and that rounding is exactly what pushed the
// antiparallel wall of the repro to -0.99999999999999989 instead of the
// exact -1 its tangent and the pull's own direction agree on. Squaring both
// sides of every comparison against -1 (or 0) is what lets this file decide
// without ever taking a square root of an irrational quantity: a component
// c = num / sqrt(scale2 * pull2) satisfies c >= 0 iff num >= 0, and (for
// num <= 0) c <= -1 iff num^2 >= scale2*pull2 — an exact rational test
// either way.
//
// revolveUndercuts is NOT converted: it carries the same defect through
// revolveangle.Extremes, a genuinely different reader, and fixing it is fu188's
// scope, not this one's.

// listVerdict folds one face's verdict into the running faces list and
// undecided flag prismUndercuts, cupUndercuts and capBlendUndercuts's
// receiver-wall loop each build over their own faces: ok=false (a
// non-finite input or a failed enclosure) propagates as a total refusal to
// the caller, matching every other refusal path these surveys already take.
func listVerdict(faces *[]*Face, undecided *bool, f *Face, verdict survey2d.PullVerdict, ok bool) bool {
	if !ok {
		return false
	}
	switch verdict {
	case survey2d.PullOpposes:
		*faces = append(*faces, f)
	case survey2d.PullUndecided:
		*undecided = true
	}
	return true
}
