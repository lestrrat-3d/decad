package prismshell_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// squareAt is a counter-clockwise square of the given side from (u, v).
func squareAt(u, v, side float64) sectionrecord.LoopRecord {
	pts := [][2]float64{{u, v}, {u + side, v}, {u + side, v + side}, {u, v + side}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		segs[i] = sectionrecord.LineSeg{
			Start:  sectionrecord.Point2{U: p[0], V: p[1]},
			End:    sectionrecord.Point2{U: q[0], V: q[1]},
			TStart: 0, TEnd: 1,
		}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func TestShellWallBandsPairsLoops(t *testing.T) {
	outer := momentinput.Profile{Outer: squareAt(0, 0, 10), Holes: []sectionrecord.LoopRecord{squareAt(4, 4, 2)}}
	cavity := momentinput.Profile{Outer: squareAt(1, 1, 8), Holes: []sectionrecord.LoopRecord{squareAt(3, 3, 4)}}
	bands, err := prismshell.ShellWallBands(t.Context(), outer, cavity)
	require.NoError(t, err)
	require.Len(t, bands, 2)

	require.Equal(t, outer.Outer, bands[0].Outer)
	require.Len(t, bands[0].Holes, 1)
	// The cavity's outer loop is reversed: it starts at the original's last
	// segment end, which is the original start (1, 1) walked backwards.
	first, ok := bands[0].Holes[0].Segments[0].(sectionrecord.LineSeg)
	require.True(t, ok)
	lastOrig, ok := cavity.Outer.Segments[len(cavity.Outer.Segments)-1].(sectionrecord.LineSeg)
	require.True(t, ok)
	require.Equal(t, lastOrig.End, first.Start)
	require.Equal(t, lastOrig.Start, first.End)

	require.Equal(t, outer.Holes[0], bands[1].Holes[0])
	require.NotEqual(t, outer.Holes[0], bands[1].Outer)

	_, err = prismshell.ShellWallBands(t.Context(), outer, momentinput.Profile{Outer: cavity.Outer})
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Contains(t, err.Error(), "different loop counts")
}
