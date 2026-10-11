package smoothloft

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestFittedLoftArcRequiresForwardPolarWalk(t *testing.T) {
	t.Parallel()
	arc := sectionrecord.ArcSeg{
		Start: sectionrecord.Point2{U: 4}, End: sectionrecord.Point2{V: 4},
		TStart: 0, TEnd: 1,
	}
	w := survey2d.SegmentWalk{
		Kind:   survey2d.WalkCircular,
		StartU: arc.Start.U, StartV: arc.Start.V,
		EndU: arc.End.U, EndV: arc.End.V,
	}
	_, validRadius := fitArcRadius(arc)
	require.True(t, validRadius)
	require.True(t, fitArcWalkForward(arc, w))

	w.Reversed = true
	require.False(t, fitArcWalkForward(arc, w))
	w.Reversed = false
	w.StartU, w.StartV = arc.End.U, arc.End.V
	w.EndU, w.EndV = arc.Start.U, arc.Start.V
	require.False(t, fitArcWalkForward(arc, w))

	arc.TStart, arc.TEnd = 1, 0
	_, validRadius = fitArcRadius(arc)
	require.False(t, validRadius)
}

func TestFittedLoftArcIncludesLastCellToRecordedEnd(t *testing.T) {
	t.Parallel()
	var parts [4]fitSectionPart
	parts[0] = fitSectionPart{
		arc: []sectionrecord.Point2{{U: 4}, {U: 0, V: 4}},
		arcSeg: sectionrecord.ArcSeg{
			Start: sectionrecord.Point2{U: 4}, End: sectionrecord.Point2{U: -4},
			TStart: 0, TEnd: 1,
		},
		arcWalk: survey2d.SegmentWalk{Kind: survey2d.WalkCircular, LengthUpper: 13},
	}
	// Supply one cell in every remaining part so the builder has a complete
	// four-part sequence without requiring a Sketch source in this count test.
	for i := 1; i < 4; i++ {
		parts[i] = parts[0]
	}
	cells, err := fitRefineCells(parts)
	require.NoError(t, err)
	require.Len(t, cells, 8)
	require.Equal(t, parts[0].arcSeg.End, cells[1].arcNext)
	require.Equal(t, parts[0].arc[1], cells[1].arc)
}
