package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sweepCircleProfile is a circle of radius r about the sketch origin.
func sweepCircleProfile(t *testing.T, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), r), r))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// TestSweepCompositeTubeUnionAuditsCandidates joins a round tube swept along a
// line-arc-line path with a box that encloses a stretch of one straight leg.
// The pair's chord tolerance meshes each tube finely enough that the number of
// all its facet pairs passes the fixed facet-pair ceiling, while the pairs
// that can touch stay far below it, so the composite sweep's cross-span audit
// must count only the candidates it lists.
//
// The tube occupies pi r^2 times its path length (Pappus), the box encloses
// the whole cross section over the stretch d, and the Union's volume bound
// must contain tube + box - pi r^2 d.
//
// Leg shown to fail: with CrossSpanContactAudit refusing whenever the triangle
// count's pair total passes the ceiling, both cases refuse with
// "the composite sweep's facet-pair audit exceeds its fixed work ceiling".
func TestSweepCompositeTubeUnionAuditsCandidates(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name         string
		r, bend      float64
		legs         [3]float64
		boxLeg       int
		boxLo, boxHi float64
		boxSide      float64
	}{
		{name: "U path box on first leg", r: 1, bend: 5, legs: [3]float64{30, 30, 30},
			boxLeg: 0, boxLo: 10, boxHi: 14, boxSide: 4},
		{name: "U path box on last leg", r: 0.75, bend: 4, legs: [3]float64{24, 40, 28},
			boxLeg: 2, boxLo: 6, boxHi: 11, boxSide: 3},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s, profile := sweepCircleProfile(t, tc.r)
			a, b, c, rr := tc.legs[0], tc.legs[1], tc.legs[2], tc.bend
			// Up the Z axis, a quarter bend into -Y, along -Y, a quarter bend
			// back down, then down to the start height: two parallel legs.
			path, err := decad.NewPath(r3.Vec{},
				decad.LineTo{End: r3.NewVec(0, 0, a)},
				decad.ArcAround{Center: r3.NewVec(0, -rr, a), Axis: r3.NewVec(1, 0, 0),
					End: r3.NewVec(0, -rr, a+rr)},
				decad.LineTo{End: r3.NewVec(0, -rr-b, a+rr)},
				decad.ArcAround{Center: r3.NewVec(0, -rr-b, a), Axis: r3.NewVec(1, 0, 0),
					End: r3.NewVec(0, -2*rr-b, a)},
				decad.LineTo{End: r3.NewVec(0, -2*rr-b, a-c)},
			)
			require.NoError(t, err)
			tube, err := decad.Sweep(t.Context(), doc, s, profile, path)
			require.NoError(t, err)

			// The box stands on the chosen leg's axis: Z from the leg's start.
			centre := r3.NewVec(0, 0, 0)
			lo, hi := tc.boxLo, tc.boxHi
			if tc.boxLeg == 2 {
				centre = r3.NewVec(0, -2*rr-b, 0)
				lo, hi = a-tc.boxHi, a-tc.boxLo
			}
			h := tc.boxSide / 2
			w := sketch.NewWorld()
			bs, bp := meshPolygonSketch(t, w, w.XY(), [][2]float64{{-h, -h}, {h, -h}, {h, h}, {-h, h}})
			box, err := decad.Extrude(t.Context(), doc, bs, bp,
				decad.Distance{D: units.Millimeters(hi - lo), Dir: decad.Along})
			require.NoError(t, err)
			move, err := r3.Translation(centre.Add(r3.NewVec(0, 0, lo)))
			require.NoError(t, err)
			box, err = decad.Transform(t.Context(), box, move)
			require.NoError(t, err)

			joined, err := decad.Union(t.Context(), tube, box)
			require.NoError(t, err)
			require.True(t, joined.IsSolid())
			volume, err := joined.Volume()
			require.NoError(t, err)
			area := math.Pi * tc.r * tc.r
			length := a + b + c + math.Pi*rr
			want := area*length + tc.boxSide*tc.boxSide*(hi-lo) - area*(hi-lo)
			require.LessOrEqual(t, volume.Value.Base()-volume.Bound.Base(), want)
			require.GreaterOrEqual(t, volume.Value.Base()+volume.Bound.Base(), want)
			// The bound must be tight enough to tell the overlap apart.
			require.Less(t, volume.Bound.Base(), area*(hi-lo)/4)
		})
	}
}
