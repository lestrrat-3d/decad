package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The class-B Union, Intersect, rooted and chained fixtures of
// docs/general-boolean-design.md §9 and §11 PR 7. The plate is 40×20×10 on XY;
// a pin is a Ø6 cylinder along x through (·, 10, 5), sketched on a YZ plane
// offset to x = x0 and extruded l along +x.

func internalPinAlongX(t *testing.T, doc *Document, x0, l float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), x0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(10, 5)
	s.Fix(c)
	s.CreateCircle(c, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(l), Dir: Along})
	require.NoError(t, err)
	return body
}

func requireBrep(t *testing.T, body *Body) brepPayload {
	t.Helper()
	bp, ok := body.payload.(brepPayload)
	require.True(t, ok, `the pair builds a brep body, got %T`, body.payload)
	requireClosedTopology(t, body)
	return bp
}

func requireVolumeCovers(t *testing.T, body *Body, a, b int64) {
	t.Helper()
	lo, hi := piEnclosed(big.NewRat(a, 1), big.NewRat(b, 1))
	requireCoversInterval(t, body.volume, lo, hi)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
}

// TestClassBRootedPin is the rooted boss across a perpendicular pair (§2's
// B4 row, read sideways): a pin entering the plate's wall x = 40 from inside
// at x = 30 and leaving to x = 50. Union keeps the plate and the 10 mm of pin
// outside it; Cut leaves a blind hole 10 mm deep with a floor; Intersect is
// the 10 mm of pin inside, a prism. Each runs the one-face reach: only the
// wall x = 40 meets the pin's tube.
//
// Legs shown to fail (each broken in classb.go, the fixture watched go red,
// then restored): the rooted arm's inside side (Union then keeps the pin's
// inside length), and the floor's outward flag (the blind hole's mesh then
// fails its volume proof).
func TestClassBRootedPin(t *testing.T) {
	t.Parallel()
	t.Run("union", func(t *testing.T) {
		t.Parallel()
		doc := New()
		plate := internalBoxBody(t, doc, 0, 0, 40, 20, 10)
		pin := internalPinAlongX(t, doc, 30, 20)
		got, err := Union(t.Context(), plate, pin)
		require.NoError(t, err)
		requireBrep(t, got)
		// Six plate faces, the pin's outside wall and its end cap.
		require.Len(t, got.Faces(), 8)
		requireVolumeCovers(t, got, 8000, 90)
		require.Equal(t, []*Body{got}, doc.Bodies())
	})
	t.Run("cut", func(t *testing.T) {
		t.Parallel()
		doc := New()
		plate := internalBoxBody(t, doc, 0, 0, 40, 20, 10)
		pin := internalPinAlongX(t, doc, 30, 20)
		got, err := Cut(t.Context(), plate, pin)
		require.NoError(t, err)
		requireBrep(t, got)
		// Six plate faces, the hole's wall and its floor.
		require.Len(t, got.Faces(), 8)
		requireVolumeCovers(t, got, 8000, -90)
	})
	t.Run("intersect", func(t *testing.T) {
		t.Parallel()
		doc := New()
		plate := internalBoxBody(t, doc, 0, 0, 40, 20, 10)
		pin := internalPinAlongX(t, doc, 30, 20)
		got, err := Intersect(t.Context(), plate, pin)
		require.NoError(t, err)
		_, ok := got.payload.(prismPayload)
		require.True(t, ok, `the inside of a rooted pin is a prism, got %T`, got.payload)
		requireVolumeCovers(t, got, 0, 90)
	})
}

// TestClassBThroughPin is the two-face reach for Union and Intersect: a pin
// from x = −5 to 45 passes through both of the plate's x walls. Union keeps
// both 5 mm stubs and both pin caps; Intersect is the 40 mm inside, a prism.
// Either operand may come first.
func TestClassBThroughPin(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		Name  string
		Op    func(*Body, *Body) (*Body, error)
		Swap  bool
		Faces int
		A, B  int64
	}{
		{Name: "union", Faces: 10, A: 8000, B: 90},
		{Name: "union, pin first", Swap: true, Faces: 10, A: 8000, B: 90},
		{Name: "intersect", A: 0, B: 360},
		{Name: "intersect, pin first", Swap: true, A: 0, B: 360},
	} {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			doc := New()
			plate := internalBoxBody(t, doc, 0, 0, 40, 20, 10)
			pin := internalPinAlongX(t, doc, -5, 50)
			a, b := plate, pin
			if tc.Swap {
				a, b = pin, plate
			}
			var got *Body
			var err error
			if tc.Faces > 0 {
				got, err = Union(t.Context(), a, b)
				require.NoError(t, err)
				requireBrep(t, got)
				require.Len(t, got.Faces(), tc.Faces)
			} else {
				got, err = Intersect(t.Context(), a, b)
				require.NoError(t, err)
				_, ok := got.payload.(prismPayload)
				require.True(t, ok)
			}
			requireVolumeCovers(t, got, tc.A, tc.B)
		})
	}
}

// internalBarHole drills a Ø4 hole along y through the 60×20×10 bar at x,
// at z, from y = −1 to 21.
func internalBarHole(t *testing.T, doc *Document, x, z float64) *Body {
	t.Helper()
	return internalDrillAlongY(t, doc, x, z, 2)
}

// TestClassBChain is §2's S11: three Ø4 cross holes through a 60×20×10 bar at
// x = 10, 30, 50, each cut into the previous brep result, so the bar ends one
// brep of nine faces whose y walls each carry three holes, volume
// 12000 − 3·π·4·20. A through hole down z at (20, 10) then cuts the brep's
// own caps (the brep takes Y along any reference axis), and a pin through
// the bar's x walls unions with the brep in either position. A fourth cross
// hole overlapping the first misses B7 and takes the mesh path, pinned.
func TestClassBChain(t *testing.T) {
	t.Parallel()
	doc := New()
	bar := internalBoxBody(t, doc, 0, 0, 60, 20, 10)
	for i, x := range []float64{10, 30, 50} {
		var err error
		bar, err = Cut(t.Context(), bar, internalBarHole(t, doc, x, 5))
		require.NoError(t, err)
		requireBrep(t, bar)
		require.Len(t, bar.Faces(), 7+i)
	}
	requireVolumeCovers(t, bar, 12000, -240)
	holed := 0
	for _, f := range bar.Faces() {
		if _, planar := f.Surface().(Plane); planar && len(f.Loops()) == 4 {
			holed++
		}
	}
	require.Equal(t, 2, holed, `each y wall carries all three holes`)

	t.Run("down the caps", func(t *testing.T) {
		doc := New()
		bar := internalBoxBody(t, doc, 0, 0, 60, 20, 10)
		for _, x := range []float64{10, 30, 50} {
			var err error
			bar, err = Cut(t.Context(), bar, internalBarHole(t, doc, x, 5))
			require.NoError(t, err)
		}
		w := sketch.NewWorld()
		mid, err := w.CreateOffsetPlane(w.XY(), 5)
		require.NoError(t, err)
		drill := internalClassBTool(t, doc, w, mid, 6, func(s *sketch.Sketch) {
			c := s.CreatePoint(20, 10)
			s.Fix(c)
			s.CreateCircle(c, 2)
		})
		got, err := Cut(t.Context(), bar, drill)
		require.NoError(t, err)
		requireBrep(t, got)
		requireVolumeCovers(t, got, 12000, -280)
	})
	t.Run("brep in either position", func(t *testing.T) {
		for _, swap := range []bool{false, true} {
			doc := New()
			bar := internalBoxBody(t, doc, 0, 0, 60, 20, 10)
			var err error
			bar, err = Cut(t.Context(), bar, internalBarHole(t, doc, 30, 5))
			require.NoError(t, err)
			// A pin rooted in the bar's wall x = 60, from x = 55 to 70: its
			// tube meets that wall alone, clear of the hole at x = 30.
			pin := internalPinAlongX(t, doc, 55, 15)
			a, b := bar, pin
			if swap {
				a, b = pin, bar
			}
			got, err := Union(t.Context(), a, b)
			require.NoError(t, err)
			requireBrep(t, got)
			// 12000 − π·4·20 + π·9·10.
			requireVolumeCovers(t, got, 12000, 10)
		}
	})
	t.Run("an overlapping fourth hole", func(t *testing.T) {
		doc := New()
		bar := internalBoxBody(t, doc, 0, 0, 60, 20, 10)
		for _, x := range []float64{10, 30, 50} {
			var err error
			bar, err = Cut(t.Context(), bar, internalBarHole(t, doc, x, 5))
			require.NoError(t, err)
		}
		fourth := internalBarHole(t, doc, 12, 5)
		_, ok, err := tryClassB(t.Context(), meshbool.OpCut, bar, fourth)
		require.NoError(t, err)
		require.False(t, ok, `the two holes' walls are not apart (B7)`)
		got, err := Cut(t.Context(), bar, fourth)
		if err != nil {
			var be *BooleanError
			require.ErrorAs(t, err, &be, `a miss reaches the mesh path's own result`)
			return
		}
		requireMeshPathResult(t, got)
	})
}

// TestClassBBreakingOutHole is §9's hole breaking out of the bar's top: a Ø4
// hole along y at z = 9 crosses the top face z = 10, so the pair misses the
// through reach and builds through the crossing reach: the top face splits
// in two, and the volume is the bar less the disc's part below the top,
// 20·(8π/3 + √3), within its bound.
func TestClassBBreakingOutHole(t *testing.T) {
	t.Parallel()
	doc := New()
	bar := internalBoxBody(t, doc, 0, 0, 60, 20, 10)
	hole := internalBarHole(t, doc, 30, 9)
	got, err := Cut(t.Context(), bar, hole)
	require.NoError(t, err)
	requireBrep(t, got)
	want := 12000 - 20*(8*math.Pi/3+math.Sqrt(3))
	require.InDelta(t, want, got.volume.Value.Base(), got.volume.Bound.Base()+1e-9*want)
	require.Less(t, got.volume.Bound.Base(), 1e-6)
}

// TestClassBStackedTarget cuts a cross hole under the pocket of a blind-cut
// plate: the stacked prism's face view is the target, the hole passes through
// its outer walls below the pocket floor, and the result is a brep of volume
// 4000 − 400 − π·4·20.
func TestClassBStackedTarget(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 20, 20, 10)
	pocket := internalBoxBodyAtZ(t, doc, 5, 5, 15, 15, 6, 4)
	part, err := Cut(t.Context(), plate, pocket)
	require.NoError(t, err)
	_, stacked := part.payload.(stackedPrismPayload)
	require.True(t, stacked, `the premise: a blind cut builds a stacked prism`)
	got, err := Cut(t.Context(), part, internalDrillAlongY(t, doc, 10, 3, 2))
	require.NoError(t, err)
	requireBrep(t, got)
	requireVolumeCovers(t, got, 3600, -80)
}

// TestClassBMissesAFaceInsideTheTube pins the reach's box test on its own: a
// bar with a vertical Ø6 hole at (30, 10), cut by a 4×4 square slot along y
// through x 28..32, z 3..7. The slot's tube meets the vertical hole's wall,
// which is not across y, so the pair misses even though both y walls'
// scenes would match the slot whole; the mesh path takes it.
//
// Shown to fail with faces not across d skipped instead of refused: the pair
// then builds a brep that ignores the vertical hole.
func TestClassBMissesAFaceInsideTheTube(t *testing.T) {
	t.Parallel()
	doc := New()
	bar := internalBoxBody(t, doc, 0, 0, 60, 20, 10)
	w := sketch.NewWorld()
	mid, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	vertical := internalClassBTool(t, doc, w, mid, 6, func(s *sketch.Sketch) {
		c := s.CreatePoint(30, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
	})
	holed, err := Cut(t.Context(), bar, vertical)
	require.NoError(t, err)
	_, prism := holed.payload.(prismPayload)
	require.True(t, prism, `the premise: the vertical hole is a co-directional cut`)
	w2 := sketch.NewWorld()
	plane, err := w2.CreateOffsetPlane(w2.XZ(), -10)
	require.NoError(t, err)
	slot := internalClassBTool(t, doc, w2, plane, 11, func(s *sketch.Sketch) {
		r := s.CreateRectangle(28, 3, 32, 7)
		s.Fix(r.A)
	})
	_, ok, err := tryClassB(t.Context(), meshbool.OpCut, holed, slot)
	require.NoError(t, err)
	require.False(t, ok)
	got, err := Cut(t.Context(), holed, slot)
	if err != nil {
		var be *BooleanError
		require.ErrorAs(t, err, &be, `a miss reaches the mesh path's own result`)
		return
	}
	requireMeshPathResult(t, got)
}
