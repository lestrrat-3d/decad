package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type foreignPathSegment struct {
	decad.PathSegment
}

func TestPathRecordsOwnedSegments(t *testing.T) {
	start := r3.NewVec(1, 2, 3)
	line := &decad.LineTo{End: r3.NewVec(1, 2, 8)}
	arc := &decad.ArcThrough{
		Through: r3.NewVec(2, 2, 9),
		End:     r3.NewVec(3, 2, 8),
	}
	input := []decad.PathSegment{line, arc}

	path, err := decad.NewPath(start, input...)
	require.NoError(t, err)

	line.End = r3.NewVec(90, 90, 90)
	arc.End = r3.NewVec(80, 80, 80)
	input[0] = decad.LineTo{End: r3.NewVec(70, 70, 70)}

	require.Equal(t, start, path.Start())
	require.Equal(t, r3.NewVec(3, 2, 8), path.End())
	require.Equal(t, []decad.PathSegment{
		decad.LineTo{End: r3.NewVec(1, 2, 8)},
		decad.ArcThrough{
			Through: r3.NewVec(2, 2, 9),
			End:     r3.NewVec(3, 2, 8),
		},
	}, path.Segments())

	got := path.Segments()
	got[0] = decad.LineTo{End: r3.NewVec(60, 60, 60)}
	require.Equal(t, decad.LineTo{End: r3.NewVec(1, 2, 8)}, path.Segments()[0])
}

func TestNewPathRejectsInvalidGeometry(t *testing.T) {
	start := r3.NewVec(0, 0, 0)
	var nilSegment decad.PathSegment
	var nilLine *decad.LineTo
	var nilArc *decad.ArcThrough

	tests := []struct {
		name     string
		start    r3.Vec
		segments []decad.PathSegment
		want     error
	}{
		{name: "no segments", start: start, want: decad.ErrDegenerate},
		{
			name:     "non-finite start",
			start:    r3.NewVec(math.NaN(), 0, 0),
			segments: []decad.PathSegment{decad.LineTo{End: r3.NewVec(0, 0, 1)}},
			want:     decad.ErrNotFinite,
		},
		{
			name:     "nil interface segment",
			start:    start,
			segments: []decad.PathSegment{nilSegment},
			want:     decad.ErrDegenerate,
		},
		{
			name:     "nil line pointer",
			start:    start,
			segments: []decad.PathSegment{nilLine},
			want:     decad.ErrDegenerate,
		},
		{
			name:     "nil arc pointer",
			start:    start,
			segments: []decad.PathSegment{nilArc},
			want:     decad.ErrDegenerate,
		},
		{
			name:     "foreign segment",
			start:    start,
			segments: []decad.PathSegment{foreignPathSegment{}},
			want:     decad.ErrDegenerate,
		},
		{
			name:     "non-finite line endpoint",
			start:    start,
			segments: []decad.PathSegment{decad.LineTo{End: r3.NewVec(0, math.Inf(1), 1)}},
			want:     decad.ErrNotFinite,
		},
		{
			name:     "zero-length line",
			start:    start,
			segments: []decad.PathSegment{decad.LineTo{End: start}},
			want:     decad.ErrDegenerate,
		},
		{
			name:  "later zero-length line",
			start: start,
			segments: []decad.PathSegment{
				decad.LineTo{End: r3.NewVec(0, 0, 1)},
				decad.LineTo{End: r3.NewVec(0, 0, 1)},
			},
			want: decad.ErrDegenerate,
		},
		{
			name:     "non-finite arc point",
			start:    start,
			segments: []decad.PathSegment{decad.ArcThrough{Through: r3.NewVec(0, 1, 0), End: r3.NewVec(1, 0, math.Inf(-1))}},
			want:     decad.ErrNotFinite,
		},
		{
			name:     "arc repeats start at through",
			start:    start,
			segments: []decad.PathSegment{decad.ArcThrough{Through: start, End: r3.NewVec(1, 0, 0)}},
			want:     decad.ErrDegenerate,
		},
		{
			name:     "arc repeats start at end",
			start:    start,
			segments: []decad.PathSegment{decad.ArcThrough{Through: r3.NewVec(0, 1, 0), End: start}},
			want:     decad.ErrDegenerate,
		},
		{
			name:     "arc repeats through at end",
			start:    start,
			segments: []decad.PathSegment{decad.ArcThrough{Through: r3.NewVec(0, 1, 0), End: r3.NewVec(0, 1, 0)}},
			want:     decad.ErrDegenerate,
		},
		{
			name:     "collinear arc",
			start:    start,
			segments: []decad.PathSegment{decad.ArcThrough{Through: r3.NewVec(1, 1, 1), End: r3.NewVec(2, 2, 2)}},
			want:     decad.ErrDegenerate,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			_, err := decad.NewPath(test.start, test.segments...)
			require.ErrorIs(t, err, test.want)
		})
	}
}

func TestNewPathUsesExactArcCollinearity(t *testing.T) {
	start := r3.NewVec(math.MaxFloat64, math.MaxFloat64, math.MaxFloat64)
	through := r3.NewVec(math.MaxFloat64, math.Nextafter(math.MaxFloat64, 0), math.MaxFloat64)
	end := r3.NewVec(math.Nextafter(math.MaxFloat64, 0), math.MaxFloat64, math.MaxFloat64)

	path, err := decad.NewPath(start, decad.ArcThrough{Through: through, End: end})
	require.NoError(t, err)
	require.Equal(t, end, path.End())
}
