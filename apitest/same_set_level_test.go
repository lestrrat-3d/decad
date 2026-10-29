package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestVerifySameSetDenotedLevels(t *testing.T) {
	for _, tc := range []struct {
		name                  string
		cup, firstInch, equal bool
	}{
		{"prism/mm-first", false, false, false}, {"prism/inch-first", false, true, false},
		{"cup/mm-first", true, false, false}, {"cup/inch-first", true, true, false},
		{"prism/mm-equal", false, false, true}, {"prism/inch-equal", false, true, true},
		{"cup/mm-equal", true, false, true}, {"cup/inch-equal", true, true, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			makeBody := func(d decad.Distance) *decad.Body {
				w := sketch.NewWorld()
				s, err := w.CreateSketch(w.XY())
				require.NoError(t, err)
				side := 1.0
				if tc.cup {
					side = 3
				}
				rect := s.CreateRectangle(0, 0, side, side)
				s.Fix(rect.A)
				_, err = s.Solve(t.Context())
				require.NoError(t, err)
				require.Len(t, s.Profiles(), 1)
				b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], d)
				require.NoError(t, err)
				if tc.cup {
					b, err = decad.Shell(t.Context(), b, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b))), units.Millimeters(1))
					require.NoError(t, err)
				}
				return b
			}
			mm := decad.Distance{D: units.Millimeters(5.08), Dir: decad.Along}
			in := decad.Distance{D: units.Inches(0.2), Dir: decad.Along}
			first, second := mm, in
			if tc.firstInch {
				first, second = in, mm
			}
			if tc.equal {
				second = first
			}
			a := makeBody(first)
			b := makeBody(second)
			inch := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.2), new(big.Rat).SetFloat64(25.4))
			require.Negative(t, inch.Cmp(new(big.Rat).SetFloat64(5.08)))
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Len(t, doc.Bodies(), 2)
			require.True(t, doc.Bodies()[0] == a && doc.Bodies()[1] == b)
			require.LessOrEqual(t, len(report.Interferences), 1)
			if tc.equal {
				require.Len(t, report.Interferences, 1)
				row := report.Interferences[0]
				require.True(t, row.A == a && row.B == b)
				volume, err := a.Volume()
				require.NoError(t, err)
				require.Equal(t, volume.Value.Base(), row.Volume.Value.Base())
				require.Equal(t, volume.Bound.Base(), row.Volume.Bound.Base())
				require.Equal(t, volume.Exactness, row.Volume.Exactness)
				return
			}
			if len(report.Interferences) == 0 {
				for _, d := range report.Diagnostics {
					t.Logf("diagnostic code=%v status=%v reading=%v pair=%v", d.Code, d.Status, d.Reading, d.Pair != nil)
				}
				require.Equal(t, decad.Suspect, report.Status)
				found := false
				for _, d := range report.Diagnostics {
					if d.Pair != nil && d.Pair.A == a && d.Pair.B == b && d.Status == decad.Suspect && d.Reading == decad.ReadingNone {
						switch d.Code {
						case decad.DiagUnsupportedPairContact:
							found = true
						}
					}
				}
				require.True(t, found)
				return
			}
			row := report.Interferences[0]
			require.True(t, row.A == a && row.B == b)
			value, bound := row.Volume.Value.Base(), row.Volume.Bound.Base()
			t.Logf("value=%g bound=%g exactness=%v", value, bound, row.Volume.Exactness)
			require.False(t, math.IsNaN(value) || math.IsInf(value, 0))
			require.False(t, math.IsNaN(bound) || bound < 0)
			if math.IsInf(bound, 1) {
				found := false
				for _, d := range report.Diagnostics {
					if d.Pair != nil && d.Pair.A == a && d.Pair.B == b && d.Code == decad.DiagMeasurementBeyondTolerance {
						found = true
					}
				}
				require.True(t, found)
				return
			}
			want := new(big.Rat).Set(inch)
			if tc.cup {
				want.Mul(want, big.NewRat(8, 1))
				want.Add(want, big.NewRat(1, 1))
			}
			residual := new(big.Rat).Sub(new(big.Rat).SetFloat64(value), want)
			residual.Abs(residual)
			covered := residual.Cmp(new(big.Rat).SetFloat64(bound)) <= 0
			require.True(t, covered, "value=%g bound=%g exactness=%v covered=%v", value, bound, row.Volume.Exactness, covered)
		})
	}
}
