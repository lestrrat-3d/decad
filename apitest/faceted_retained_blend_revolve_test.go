package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-edge-blend-design.md §2 for a revolve
// wall: a r=10, L=40 shaft along x with a D-flat at z >= 7 cut by a
// Transform-placed box. Built by Revolve, the wall and end planes carry a
// denoted patch whose point leaves state the tag exactly, so the retained
// routes read them as exact.

const (
	shaftR, shaftL, flatZ = 10.0, 40.0, 7.0
)

// revolvedShaft revolves the rectangle x in [0, 40], y in [0, 10] about the
// x axis, a solid shaft of radius shaftR.
func revolvedShaft(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, shaftL, shaftR)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// extrudedShaft is the Extrude twin of revolvedShaft, along x.
func extrudedShaft(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.YZ())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, shaftR)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(shaftL), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// flatTool is a box spanning the shaft's length whose floor is at z = flatZ,
// moved by Transform so the Cut takes the mesh path.
func flatTool(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(-5, -20, shaftL+5, 20)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: flatZ})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), box, move)
	require.NoError(t, err)
	return placed
}

// flatEdges selects the two straight edges where the flat meets the wall.
func flatEdges(t *testing.T, b *decad.Body) decad.EdgeSelector {
	t.Helper()
	x := r3.NewVec(1, 0, 0)
	var q *decad.EdgeQuery
	n := 0
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(decad.Line3); !ok {
			continue
		}
		a, c := e.Start().Position().Value, e.End().Position().Value
		if math.Abs(a.Z-flatZ) > 1e-6 || math.Abs(c.Z-flatZ) > 1e-6 || math.Abs(a.X-c.X) < 1 {
			continue
		}
		preds := []decad.EdgePredicate{decad.EndpointAt(a), decad.ParallelTo(x)}
		if q == nil {
			q = decad.Edges(preds...)
		} else {
			q.Or(preds...)
		}
		n++
	}
	require.Equal(t, 2, n, "the flat's two long edges")
	return q.Exactly(2)
}

// flatCorner is the section of one blended flat edge: the region the blend
// removes, in (y, z), for a ball of radius r or a bevel of distance d.
func flatCorner(blendFillet bool, size float64) [][2]float64 {
	k := [2]float64{math.Sqrt(shaftR*shaftR - flatZ*flatZ), flatZ}
	arc := func(from, to [2]float64, n int) [][2]float64 {
		a0 := math.Atan2(from[1], from[0])
		a1 := math.Atan2(to[1], to[0])
		out := make([][2]float64, 0, n+1)
		for i := 0; i <= n; i++ {
			a := a0 + (a1-a0)*float64(i)/float64(n)
			out = append(out, [2]float64{shaftR * math.Cos(a), shaftR * math.Sin(a)})
		}
		return out
	}
	if !blendFillet {
		ta := [2]float64{k[0] - size, flatZ}
		a := math.Atan2(k[1], k[0]) - size/shaftR
		tb := [2]float64{shaftR * math.Cos(a), shaftR * math.Sin(a)}
		return append([][2]float64{k, ta}, arc(tb, k, 2000)...)
	}
	// The ball rolls inside the material, size under the flat and
	// shaftR-size from the axis.
	centre := [2]float64{math.Sqrt((shaftR-size)*(shaftR-size) - (flatZ-size)*(flatZ-size)), flatZ - size}
	ta := [2]float64{centre[0], flatZ}
	scale := shaftR / math.Hypot(centre[0], centre[1])
	tb := [2]float64{centre[0] * scale, centre[1] * scale}
	region := [][2]float64{k, ta}
	a0 := math.Atan2(ta[1]-centre[1], ta[0]-centre[0])
	a1 := math.Atan2(tb[1]-centre[1], tb[0]-centre[0])
	for i := 1; i < 2000; i++ {
		a := a0 + (a1-a0)*float64(i)/2000
		region = append(region, [2]float64{centre[0] + size*math.Cos(a), centre[1] + size*math.Sin(a)})
	}
	region = append(region, tb)
	return append(region, arc(tb, k, 2000)...)
}

func TestFacetedRetainedBlendRevolvedShaft(t *testing.T) {
	t.Parallel()

	flatShaft := func(t *testing.T) *decad.Body {
		t.Helper()
		sc := newRetentionScene()
		cut, err := decad.Cut(t.Context(), revolvedShaft(t, sc), flatTool(t, sc))
		require.NoError(t, err)
		return cut
	}

	t.Run("normal on the trimmed wall", func(t *testing.T) {
		t.Parallel()
		cut := flatShaft(t)
		walls := 0
		for _, f := range cut.Faces() {
			c, ok := f.Surface().(decad.Cylinder)
			if !ok {
				continue
			}
			walls++
			// A point on the wall below the flat: angle 30 degrees from +y.
			require.InDelta(t, shaftR, c.Radius.Base(), 0)
			n, err := f.NormalAt(r3.NewVec(20, shaftR*math.Cos(math.Pi/6), shaftR*math.Sin(math.Pi/6)))
			require.NoError(t, err)
			want := r3.NewVec(0, math.Cos(math.Pi/6), math.Sin(math.Pi/6))
			require.Less(t, n.Value.Sub(want).Len(), 1e-9)
			require.Less(t, n.Bound.Mag(), 1.0, "the proven bound stays below 1 on the shrunk wall")
		}
		require.Equal(t, 1, walls)
	})

	t.Run("a rotated revolve still refuses", func(t *testing.T) {
		t.Parallel()
		// A rotated placement makes the denoted basis a held float, not an
		// exact frame, so the tag is not provably the record's surface.
		sc := newRetentionScene()
		turn := rotation(t, r3.Vec{}, r3.NewVec(0, 0, 1), 0.3)
		turned, err := decad.Transform(t.Context(), revolvedShaft(t, sc), turn)
		require.NoError(t, err)
		cut, err := decad.Cut(t.Context(), turned, flatTool(t, sc))
		require.NoError(t, err)
		var q *decad.EdgeQuery
		for _, e := range cut.Edges() {
			a, c := e.Start().Position().Value, e.End().Position().Value
			if math.Abs(a.Z-flatZ) > 1e-6 || math.Abs(c.Z-flatZ) > 1e-6 || a.Sub(c).Len() < shaftL/2 {
				continue
			}
			// The flat meets a wall whose tag is not provably its revolve
			// record, so the rim keeps its chord chain
			// (agent-docs/faceted-source-retention-design.md §10.1 E12).
			require.IsType(t, decad.FacetedCurve{}, e.Curve())
			preds := []decad.EdgePredicate{decad.EndpointAt(a), decad.EndpointAt(c)}
			if q == nil {
				q = decad.Edges(preds...)
			} else {
				q.Or(preds...)
			}
		}
		require.NotNil(t, q)
		_, err = decad.Fillet(t.Context(), cut, q, units.Millimeters(2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	for _, tc := range []struct {
		name   string
		fillet bool
		size   float64
	}{{"fillet", true, 2}, {"chamfer", false, 1}} {
		t.Run(tc.name+" of the D-flat", func(t *testing.T) {
			t.Parallel()
			cut := flatShaft(t)
			before, err := cut.Volume()
			require.NoError(t, err)
			size := tc.size
			var blended *decad.Body
			if tc.fillet {
				blended, err = decad.Fillet(t.Context(), cut, flatEdges(t, cut), units.Millimeters(size))
			} else {
				blended, err = decad.Chamfer(t.Context(), cut, flatEdges(t, cut), units.Millimeters(size))
			}
			require.NoError(t, err)
			after, err := blended.Volume()
			require.NoError(t, err)
			want := 2 * shaftL * shoelaceArea(flatCorner(tc.fillet, size))
			requireVolumeChange(t, before, after, want)
			if tc.fillet {
				blends := 0
				for _, f := range blended.Faces() {
					if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-size) < 1e-12 {
						blends++
					}
				}
				require.Equal(t, 2, blends)
			}
			mesh, err := blended.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())

			// The Extrude twin blends the same flat to the same volume.
			sc := newRetentionScene()
			twinCut, err := decad.Cut(t.Context(), extrudedShaft(t, sc), flatTool(t, sc))
			require.NoError(t, err)
			twinBefore, err := twinCut.Volume()
			require.NoError(t, err)
			var twin *decad.Body
			if tc.fillet {
				twin, err = decad.Fillet(t.Context(), twinCut, flatEdges(t, twinCut), units.Millimeters(size))
			} else {
				twin, err = decad.Chamfer(t.Context(), twinCut, flatEdges(t, twinCut), units.Millimeters(size))
			}
			require.NoError(t, err)
			twinAfter, err := twin.Volume()
			require.NoError(t, err)
			requireVolumeChange(t, twinBefore, twinAfter, want)
		})
	}
}

// slotTool is a box across the shaft at x in [15, 25] from z = 3 up, moved by
// Transform so the Cut takes the mesh path.
func slotTool(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(15, -20, 25, 20)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: 3})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), box, move)
	require.NoError(t, err)
	return placed
}

// endRim selects the circular rim of the shaft's end face at x = 0.
func endRim(t *testing.T, b *decad.Body) decad.EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		c, ok := e.Curve().(decad.Circle3)
		if !ok || math.Abs(c.Center.X) > 1e-6 || math.Abs(c.Radius.Base()-shaftR) > 1e-6 {
			continue
		}
		return decad.Edges(decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
	}
	require.Fail(t, "no end rim")
	return nil
}

func TestFacetedRetainedBlendRevolvedRim(t *testing.T) {
	t.Parallel()
	const R = shaftR

	for _, tc := range []struct {
		name   string
		fillet bool
		size   float64
	}{{"fillet", true, 2}, {"chamfer", false, 2}} {
		t.Run(tc.name+" of the end rim", func(t *testing.T) {
			t.Parallel()
			r := tc.size
			// Pappus over the corner the blend removes from the meridian.
			var want float64
			if tc.fillet {
				want = 2 * math.Pi * (r*r*(R-r/2) - math.Pi*r*r/4*(R-r+4*r/(3*math.Pi)))
			} else {
				want = 2 * math.Pi * r * r / 2 * (R - r/3)
			}
			blend := func(b *decad.Body) (*decad.Body, decad.Measurement, decad.Measurement) {
				before, err := b.Volume()
				require.NoError(t, err)
				var out *decad.Body
				if tc.fillet {
					out, err = decad.Fillet(t.Context(), b, endRim(t, b), units.Millimeters(r))
				} else {
					out, err = decad.Chamfer(t.Context(), b, endRim(t, b), units.Millimeters(r))
				}
				require.NoError(t, err)
				after, err := out.Volume()
				require.NoError(t, err)
				return out, before, after
			}

			sc := newRetentionScene()
			cut, err := decad.Cut(t.Context(), revolvedShaft(t, sc), slotTool(t, sc))
			require.NoError(t, err)
			blended, before, after := blend(cut)
			requireVolumeChange(t, before, after, want)
			mesh, err := blended.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())

			// The Extrude twin blends the same rim to the same volume.
			tsc := newRetentionScene()
			twinCut, err := decad.Cut(t.Context(), extrudedShaft(t, tsc), slotTool(t, tsc))
			require.NoError(t, err)
			_, twinBefore, twinAfter := blend(twinCut)
			requireVolumeChange(t, twinBefore, twinAfter, want)
		})
	}
}
