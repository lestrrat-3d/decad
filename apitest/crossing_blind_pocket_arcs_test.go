package apitest_test

import (
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// arcPocketBody extrudes the profile draw makes on the plane z = planeZ by
// depth in dir.
func arcPocketBody(t *testing.T, doc *decad.Document, planeZ float64, depth units.Value, dir decad.Direction,
	draw func(*sketch.Sketch)) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), planeZ)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: depth, Dir: dir})
	require.NoError(t, err)
	return body
}

func drawRect(x0, y0, x1, y1 float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
	}
}

// drawCrossSlot is a slot whose semicircle centres sit at (±3, 0), radius 2:
// two lines joined by two tangent arcs.
func drawCrossSlot(s *sketch.Sketch) {
	slot, err := s.CreateSlot(-3, 0, 3, 0, 2)
	if err != nil {
		panic(err)
	}
	s.Fix(slot.C1)
	s.Fix(slot.C2)
}

// arcVolume is k + p*pi + q*sqrt(3) mm^3 at volumeRefPrec, each coefficient a
// decimal string.
func arcVolume(t *testing.T, k, p, q string) *big.Float {
	t.Helper()
	parse := func(s string) *big.Float {
		f, ok := new(big.Float).SetPrec(volumeRefPrec).SetString(s)
		require.True(t, ok, s)
		return f
	}
	sqrt3 := new(big.Float).SetPrec(volumeRefPrec).Sqrt(parse("3"))
	v := parse(k)
	v.Add(v, new(big.Float).SetPrec(volumeRefPrec).Mul(parse(p), parse(piRef)))
	return v.Add(v, new(big.Float).SetPrec(volumeRefPrec).Mul(parse(q), sqrt3))
}

// TestCrossingBlindPocketArcLoops cuts blind pockets across through holes
// whose plan loops hold circles and arcs, on a 20x20x10 plate (or a disc).
// Every line crossing a circle meets it at an irrational point, so the
// volume is Approximate, and its bound must hold the closed form, computed
// at 400 bits.
//
// The closed forms. A rectangle x -2..6, y -1.5..1.5 against the disc r=3
// keeps x from -2 to sqrt(9-y^2) inside it, an area of
// 6 + integral sqrt(9-y^2) over |y| <= 1.5 = 6 + 9*sqrt(3)/4 + 3*pi/2, so a
// pocket of depth d removes d*(18 - 9*sqrt(3)/4 - 3*pi/2) beyond the
// 90*pi through hole: 3928 - 84*pi + 9*sqrt(3) at d = 4. A disc r=2 centred
// at (1, 0) overlaps the 4x4 hole x -4..0 in the segment beyond x = 0, of area
// 4*pi/3 - sqrt(3), so depth 4 removes 4*(8*pi/3 + sqrt(3)) beyond the 160
// hole. The slot (24 + 4*pi) meets the rectangle x 2..8, y -1..1 in
// 2 + sqrt(3) + 2*pi/3, so depth 4 removes 4*(10 - sqrt(3) - 2*pi/3). The L
// pocket adds x 4..6, y 1.5..5 to the first rectangle, 28 mm^3 more at depth
// 4. The disc plate of radius 10 has the 4x4 hole and a rectangle x -1..3,
// y -1..1 that overlaps the hole in 1x2, so depth 4 removes 24.
//
// Shown red: restoring the four-axis-line gate sends every case to the mesh
// path. Not witnessed here: the face displacement the BRep build charges (the
// merges' cut charge plus the canonical vertices' allowance). Zeroing it, or
// either term alone, leaves every case green, because each vertex sits within
// about 1e-15 mm of its crossing, which moves the volume by about 1e-14 mm^3,
// while the measurement's own rounding bound is about 1e-11 mm^3.
func TestCrossingBlindPocketArcLoops(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	roundHole := func(t *testing.T, d *decad.Document) *decad.Body { return circleBodyAtZ(t, d, 0, 3, 0, 10) }
	squareHole := func(t *testing.T, d *decad.Document) *decad.Body { return boxBodyAtZ(t, d, -4, -2, 0, 2, 0, 10) }
	plate := func(t *testing.T, d *decad.Document) *decad.Body { return boxBody(t, d, -10, -10, 10, 10, 10) }
	pocketTop := func(draw func(*sketch.Sketch)) func(*testing.T, *decad.Document) *decad.Body {
		return func(t *testing.T, d *decad.Document) *decad.Body {
			return arcPocketBody(t, d, 6, mm(4), decad.Along, draw)
		}
	}
	for _, tc := range []struct {
		name              string
		plate, hole, tool func(*testing.T, *decad.Document) *decad.Body
		k, p, q           string
		radii             []float64
		displaced         bool
	}{
		{name: "round-hole/top", plate: plate, hole: roundHole, tool: pocketTop(drawRect(-2, -1.5, 6, 1.5)),
			k: "3928", p: "-84", q: "9", radii: []float64{3}},
		{name: "round-hole/bottom", plate: plate, hole: roundHole,
			tool: func(t *testing.T, d *decad.Document) *decad.Body {
				return arcPocketBody(t, d, 0, mm(4), decad.Along, drawRect(-2, -1.5, 6, 1.5))
			}, k: "3928", p: "-84", q: "9", radii: []float64{3}},
		{name: "round-pocket/square-hole", plate: plate, hole: squareHole, radii: []float64{2},
			k: "3840", p: "-10.666666666666666666666666666666666666666666666666666666666666667", q: "-4",
			tool: func(t *testing.T, d *decad.Document) *decad.Body { return circleBodyAtZ(t, d, 1, 2, 6, 4) }},
		{name: "slot-hole", plate: plate,
			hole: func(t *testing.T, d *decad.Document) *decad.Body {
				return arcPocketBody(t, d, 0, mm(10), decad.Along, drawCrossSlot)
			}, tool: pocketTop(drawRect(2, -1, 8, 1)),
			k: "3720", p: "-37.333333333333333333333333333333333333333333333333333333333333333", q: "4",
			radii: []float64{2}},
		{name: "l-pocket", plate: plate, hole: roundHole, tool: pocketTop(drawPolygon(
			[2]float64{-2, -1.5}, [2]float64{6, -1.5}, [2]float64{6, 5}, [2]float64{4, 5},
			[2]float64{4, 1.5}, [2]float64{-2, 1.5})),
			k: "3900", p: "-84", q: "9", radii: []float64{3}},
		{name: "disc-plate", hole: squareHole, tool: pocketTop(drawRect(-1, -1, 3, 1)),
			k: "-184", p: "1000", q: "0", radii: []float64{10},
			plate: func(t *testing.T, d *decad.Document) *decad.Body { return circleBodyAtZ(t, d, 0, 10, 0, 10) }},
		// 0.2 in is 5.08 mm, whose level carries a displacement:
		// 4000 - 90*pi - 5.08*(18 - 9*sqrt(3)/4 - 3*pi/2).
		{name: "round-hole/inch-depth", plate: plate, hole: roundHole,
			tool: func(t *testing.T, d *decad.Document) *decad.Body {
				return arcPocketBody(t, d, 10, units.Inches(0.2), decad.Against, drawRect(-2, -1.5, 6, 1.5))
			}, k: "3908.56", p: "-82.38", q: "11.43", radii: []float64{3}, displaced: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			part, err := decad.Cut(t.Context(), tc.plate(t, doc), tc.hole(t, doc))
			require.NoError(t, err)
			require.Equal(t, "decad.prismPayload", decadtest.PayloadClass(part))
			part, err = decad.Cut(t.Context(), part, tc.tool(t, doc))
			require.NoError(t, err)
			require.Equal(t, "decad.brepPayload", decadtest.PayloadClass(part))
			requireEveryEdgeOnTwoFaces(t, part)
			// The slot's sketch-solved arc ends sit a few ulps off radius 2, so
			// a cylinder reads its source radius to within 1e-12.
			seen := make([]bool, len(tc.radii))
			for _, f := range part.Faces() {
				switch s := f.Surface().(type) {
				case decad.Plane:
				case decad.Cylinder:
					r, err := s.Radius.In(units.Millimeter)
					require.NoError(t, err)
					i := slices.IndexFunc(tc.radii, func(want float64) bool { return math.Abs(r-want) <= 1e-12 })
					require.GreaterOrEqual(t, i, 0, "cylinder radius %v", r)
					seen[i] = true
				default:
					require.Failf(t, "unexpected surface", "%T", s)
				}
			}
			require.NotContains(t, seen, false)

			volume, err := part.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, volume.Exactness)
			bound := boundMM3(t, volume)
			require.Greater(t, bound, 0.0)
			require.Less(t, bound, 1e-9)
			gap := new(big.Float).SetPrec(volumeRefPrec).SetFloat64(volumeMM(t, volume))
			gap.Sub(gap, arcVolume(t, tc.k, tc.p, tc.q)).Abs(gap)
			require.LessOrEqual(t, gap.Cmp(new(big.Float).SetFloat64(bound)), 0,
				"volume %v misses the closed form by %s, bound %v", volumeMM(t, volume), gap.Text('g', 6), bound)
			if !tc.displaced {
				require.Less(t, bound, 1e-10)
			}

			mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
			requireWatertight(t, mesh)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
		})
	}
}

// TestCrossingBlindPocketArcLoopsDecline keeps two crossings on the mesh path.
// A round pocket crossing a round hole meets it where two circles cross, which
// no exact record states. An oblique pocket wall crosses the round hole off
// every axis-aligned plane the BRep build can state exactly.
//
// Shown red: dropping the oblique-line refusal in momentinput.CrossingBlindLoops
// builds the oblique case as a brep. Dropping both of stackedbrep's two-circle
// misses (unequal walked ends, and a second point under one crossing key)
// builds the two-circle case; either one alone still misses.
func TestCrossingBlindPocketArcLoopsDecline(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		tool func(*testing.T, *decad.Document) *decad.Body
	}{
		{"two-circles", func(t *testing.T, d *decad.Document) *decad.Body { return circleBodyAtZ(t, d, 3, 2, 6, 4) }},
		{"oblique-wall", func(t *testing.T, d *decad.Document) *decad.Body {
			return arcPocketBody(t, d, 6, units.Millimeters(4), decad.Along, drawPolygon(
				[2]float64{-1, -1}, [2]float64{6, -2}, [2]float64{6, 2}, [2]float64{-1, 1}))
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			part, err := decad.Cut(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 0, 3, 0, 10))
			require.NoError(t, err)
			part, err = decad.Cut(t.Context(), part, tc.tool(t, doc))
			require.NoError(t, err)
			require.True(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
		})
	}
}
