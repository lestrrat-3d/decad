package sectionaudit_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func polarPt(r, a float64) sectionrecord.Point2 {
	return sectionrecord.Point2{U: r * math.Cos(a), V: r * math.Sin(a)}
}

func lineBetween(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
	return sectionrecord.LineSeg{Start: a, End: b, TEnd: 1}
}

// windowLoop is a clockwise annular-sector hole about the origin. With
// chamfer set, the radial side at the outer arc's end is split at radius
// 13.5, so a later walk start lies on neither concentric circle.
func windowLoop(phi float64, chamfer bool) sectionrecord.LoopRecord {
	const half = 0.3
	c := sectionrecord.Point2{}
	segs := []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: c, Start: polarPt(14, phi-half), End: polarPt(14, phi+half), TStart: 1, TEnd: 0},
	}
	if chamfer {
		segs = append(segs, lineBetween(polarPt(14, phi-half), polarPt(13.5, phi-half)),
			lineBetween(polarPt(13.5, phi-half), polarPt(8, phi-half)))
	} else {
		segs = append(segs, lineBetween(polarPt(14, phi-half), polarPt(8, phi-half)))
	}
	segs = append(segs,
		sectionrecord.ArcSeg{Center: c, Start: polarPt(8, phi-half), End: polarPt(8, phi+half), TStart: 0, TEnd: 1},
		lineBetween(polarPt(8, phi+half), polarPt(14, phi+half)))
	return sectionrecord.LoopRecord{Segments: segs}
}

func nestingOf(t *testing.T, holes ...sectionrecord.LoopRecord) error {
	t.Helper()
	disc := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Millimeters(20), CCW: true, TEnd: 1}}}
	loops := append([]sectionrecord.LoopRecord{disc}, holes...)
	budget := proofbound.NewWorkBudget(t.Context())
	entries, err := sectionaudit.EntriesOf(budget, loops)
	require.NoError(t, err)
	return sectionaudit.Nesting(budget, entries, len(loops))
}

// A window whose first walk start lies on a concentric window's carrier
// circle is classified by a later start that lies on neither.
func TestNestingTriesLaterWalkStarts(t *testing.T) {
	t.Parallel()
	require.NoError(t, nestingOf(t, windowLoop(0.6, true), windowLoop(3.4, true)))
	// With every start of the windows on one of the two concentric circles
	// nothing decides the ray test, and the audit still refuses.
	err := nestingOf(t, windowLoop(0.6, false), windowLoop(3.4, false))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
