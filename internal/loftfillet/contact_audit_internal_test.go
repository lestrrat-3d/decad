package loftfillet

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestFitArcOutside(t *testing.T) {
	t.Parallel()
	// The first-quadrant short arc: its wedge is the first quadrant.
	arc := sectionrecord.ArcSeg{
		Center: sectionrecord.Point2{U: 0, V: 0},
		Start:  sectionrecord.Point2{U: 1, V: 0},
		End:    sectionrecord.Point2{U: 0, V: 1},
		TStart: 0, TEnd: 1,
	}
	half := func(n int64) *big.Rat { return big.NewRat(n, 2) }
	span := func(u0, v0, u1, v1 int64) freeform.BezierSpan {
		return freeform.BezierSpan{
			{U: half(u0), V: half(v0)},
			{U: half(u1), V: half(v1)},
		}
	}
	for _, tc := range []struct {
		name   string
		span   freeform.BezierSpan
		lo, hi *big.Rat
		want   bool
	}{
		{name: "left of end ray", span: span(-2, 1, -1, 2), lo: big.NewRat(0, 1), hi: big.NewRat(1, 1), want: true},
		{name: "below start ray", span: span(1, -2, 2, -1), lo: big.NewRat(0, 1), hi: big.NewRat(1, 1), want: true},
		{name: "inside wedge", span: span(1, 1, 2, 2), lo: big.NewRat(0, 1), hi: big.NewRat(1, 1)},
		{name: "crossing full span", span: span(-2, 1, 2, 1), lo: big.NewRat(0, 1), hi: big.NewRat(1, 1)},
		{name: "crossing restricted span", span: span(-2, 1, 2, 1), lo: big.NewRat(0, 1), hi: big.NewRat(1, 4), want: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, fitArcOutside(tc.span, arc, tc.lo, tc.hi))
		})
	}
}
