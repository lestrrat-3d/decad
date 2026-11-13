package parts_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	mvPlate  = "back plate"
	mvBridge = "bridge"
)

func mvArbor(k int) string { return fmt.Sprintf("arbor %d", k) }
func mvWheel(k int) string { return fmt.Sprintf("wheel %d", k) }
func mvScrew(k int) string { return fmt.Sprintf("screw %d", k) }

// mvArborX is the x of arbor k (1..3).
func mvArborX(k int) float64 { return float64(k-2) * 30 }

// mvScrewY is the y of screw k (1, 2).
func mvScrewY(k int) float64 { return float64(3-2*k) * 15 }

// movementSet mimics a flat clock movement (agent-docs/parts-design.md §5.9): a wide back plate with three
// bores, three arbors standing behind it, wheels and a bridge in front and two screws holding the bridge.
// Every part leaves along Z. The arbors reach tail mm behind the plate (10 in the fixture table). Coordinates
// are in mm.
func movementSet(t *testing.T, tail float64) (*parts.Set, map[string]*decad.Body) {
	t.Helper()
	doc := decad.New()
	cut := func(b, tool *decad.Body) *decad.Body { return must(decad.Cut(t.Context(), b, tool)) }
	union := func(b, o *decad.Body) *decad.Body { return must(decad.Union(t.Context(), b, o)) }
	cyl := func(x, y, r, z0, h float64) *decad.Body { return cylinderAt(t, doc, x, y, r, z0, h) }

	plate := block(t, doc, -60, -40, 60, 40, 0, 6)
	bridge := block(t, doc, -50, -20, 50, 20, 15, 3)
	bodies := map[string]*decad.Body{}
	for k := 1; k <= 3; k++ {
		x := mvArborX(k)
		plate = cut(plate, cyl(x, 0, 2.5, -1, 8))
		bridge = cut(bridge, cyl(x, 0, 2.5, 14, 5))
		bodies[mvArbor(k)] = union(cyl(x, 0, 2, -tail, 22+tail), cyl(x, 0, 4, -tail, 4))
		bodies[mvWheel(k)] = cut(cyl(x, 0, 12, 9, 3), cyl(x, 0, 2.5, 8, 5))
	}
	for k := 1; k <= 2; k++ {
		y := mvScrewY(k)
		bridge = cut(bridge, cyl(45, y, 1.6, 14, 5))
		bodies[mvScrew(k)] = union(cyl(45, y, 1.5, 6, 12), cyl(45, y, 3, 18, 2))
	}
	bodies[mvPlate], bodies[mvBridge] = plate, bridge

	s := parts.NewSet(doc)
	names := []string{mvPlate, mvArbor(1), mvArbor(2), mvArbor(3), mvWheel(1), mvWheel(2), mvWheel(3), mvBridge,
		mvScrew(1), mvScrew(2)}
	for _, name := range names {
		mustAdd(t, s, parts.Part{Name: name, Kind: parts.Manufactured, Body: bodies[name]})
	}
	return s, bodies
}

// movementOpts are the options every movement explode runs with.
//
// The wheels are fixed to +Z: once the arbors have left, nothing stands beside a wheel, so the search would
// also let it leave sideways (D3), and the fixture is a movement whose every part leaves along Z.
func movementOpts(extra ...parts.ExplodeOption) []parts.ExplodeOption {
	opts := []parts.ExplodeOption{parts.WithDerivedExplode(), parts.WithExplodeGround(mvPlate),
		parts.WithExplodeGap(units.Millimeters(5))}
	for k := 1; k <= 3; k++ {
		opts = append(opts, parts.WithRemoval(mvWheel(k), parts.Removal{Direction: r3.Vec{Z: 1}}))
	}
	return append(opts, extra...)
}

// mvWant is a part's row of the fixture table: layer, D7 offset (the opt-out run's) and slide.
type mvWant struct {
	layer int
	slide r3.Vec
	d7    r3.Vec
}

func movementTable() map[string]mvWant {
	table := map[string]mvWant{
		mvPlate:  {},
		mvBridge: {2, r3.Vec{X: 115, Y: -65}, r3.Vec{Z: 4}},
	}
	for k := 1; k <= 3; k++ {
		table[mvArbor(k)] = mvWant{-1, r3.Vec{X: -26, Y: -81}, r3.Vec{Z: -27}}
		table[mvWheel(k)] = mvWant{1, r3.Vec{X: -18, Y: 57}, r3.Vec{Z: 2}}
	}
	for k := 1; k <= 2; k++ {
		table[mvScrew(k)] = mvWant{3, r3.Vec{X: 23, Y: -22}, r3.Vec{Z: 21}}
	}
	return table
}

func TestSpreadMovement(t *testing.T) {
	s, bodies := movementSet(t, 10)
	table := movementTable()

	flat, err := s.Explode(t.Context(), movementOpts(parts.WithoutExplodeSpread())...)
	require.NoError(t, err)
	require.NotNil(t, flat.Spread)
	require.False(t, flat.Spread.Ran)
	require.InDelta(t, 32.0/120, flat.Spread.Thinness.Base(), 1e-6)
	require.False(t, flat.Spread.Coaxial)
	for name, want := range table {
		p := partByName(t, flat, name)
		requireVec(t, want.d7, p.Offset, name+" D7 offset")
		requireVec(t, r3.Vec{}, p.Slide, name+" slide")
	}
	hidden, err := flat.Hidden(r3.Vec{Z: -1})
	require.NoError(t, err)
	covers := func(front, back string) float64 {
		for _, h := range hidden {
			if h.Front == front && h.Back == back {
				return h.Covered.Base()
			}
		}
		return 0
	}
	for k := 1; k <= 3; k++ {
		require.InDelta(t, 1, covers(mvPlate, mvArbor(k)), 1e-9, "the plate hides arbor %d from above", k)
		require.InDelta(t, 1, covers(mvBridge, mvWheel(k)), 1e-9, "the bridge hides wheel %d from above", k)
	}

	ex, err := s.Explode(t.Context(), movementOpts()...)
	require.NoError(t, err)
	sp := ex.Spread
	require.NotNil(t, sp)
	require.True(t, sp.Ran)
	requireVec(t, r3.Vec{Z: 1}, sp.Axis, "axis")
	requireVec(t, r3.Vec{Y: 1}, sp.Across, "across")
	requireVec(t, r3.Vec{X: 1}, sp.Rows, "rows")
	require.Equal(t, 5, sp.Layers)
	require.Equal(t, 3, sp.Columns)
	require.InDelta(t, 154, sp.Width.Base(), 1e-6)
	require.InDelta(t, 225, sp.Depth.Base(), 1e-6)
	require.Equal(t, units.Length, sp.Width.Kind())
	require.Equal(t, units.Length, sp.Depth.Kind())
	require.Equal(t, units.Dimensionless, sp.Aspect.Kind())
	require.InDelta(t, 225.0/154, sp.Aspect.Base(), 1e-6)
	requireVec(t, r3.Vec{X: 1, Y: 1, Z: -1}.Scale(1/math.Sqrt(3)), sp.View, "view")
	require.InDelta(t, 1, sp.View.Len(), 1e-12)

	for name, want := range table {
		p, base := partByName(t, ex, name), partByName(t, flat, name)
		require.Equal(t, want.layer, p.Layer, name)
		requireVec(t, want.slide, p.Slide, name+" slide")
		requireVec(t, want.d7.Add(want.slide), p.Offset, name+" final offset")
		requireVec(t, base.Offset, p.Offset.Sub(p.Slide), name+" offset less slide")
		require.Equal(t, [2]int{base.Order, base.Round}, [2]int{p.Order, p.Round}, name)
		require.Equal(t, base.Stage, p.Stage, name)
		requireVec(t, base.Dir, p.Dir, name+" dir")
	}
	require.Empty(t, ex.Overlaps)

	// Parts of different layers keep the gap along Z and along X or Y.
	final := func(name string) decad.Box {
		b, err := bodies[name].Bounds()
		require.NoError(t, err)
		off := partByName(t, ex, name).Offset
		return decad.Box{Min: b.Min.Add(off), Max: b.Max.Add(off)}
	}
	gapAlong := func(a, b decad.Box, lo, hi func(decad.Box) float64) float64 {
		return math.Max(lo(a)-hi(b), lo(b)-hi(a))
	}
	zLo, zHi := func(b decad.Box) float64 { return b.Min.Z }, func(b decad.Box) float64 { return b.Max.Z }
	xLo, xHi := func(b decad.Box) float64 { return b.Min.X }, func(b decad.Box) float64 { return b.Max.X }
	yLo, yHi := func(b decad.Box) float64 { return b.Min.Y }, func(b decad.Box) float64 { return b.Max.Y }
	for _, a := range ex.Parts {
		for _, b := range ex.Parts {
			if a.Name >= b.Name || a.Layer == b.Layer {
				continue
			}
			fa, fb := final(a.Name), final(b.Name)
			require.GreaterOrEqual(t, gapAlong(fa, fb, zLo, zHi), 5-1e-9, a.Name+" and "+b.Name+" along Z")
			across := math.Max(gapAlong(fa, fb, xLo, xHi), gapAlong(fa, fb, yLo, yHi))
			require.GreaterOrEqual(t, across, 5-1e-9, a.Name+" and "+b.Name+" across Z")
		}
	}
	for _, view := range []r3.Vec{sp.View, {Z: -1}} {
		hidden, err := ex.Hidden(view)
		require.NoError(t, err)
		require.Empty(t, hidden, "no part hides another seen along %v", view)
	}

	again, err := s.Explode(t.Context(), movementOpts()...)
	require.NoError(t, err)
	for i, p := range ex.Parts {
		require.Equal(t, p.Offset, again.Parts[i].Offset, p.Name)
	}
}

func TestSpreadGate(t *testing.T) {
	requireUnchanged := func(t *testing.T, ex *parts.Exploded, thinness float64) {
		t.Helper()
		require.NotNil(t, ex.Spread)
		require.False(t, ex.Spread.Ran)
		require.InDelta(t, thinness, ex.Spread.Thinness.Base(), 1e-6)
		for _, p := range ex.Parts {
			requireVec(t, r3.Vec{}, p.Slide, p.Name+" slide")
		}
	}
	t.Run("tall stack", func(t *testing.T) {
		ex, err := collarSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(nameCarrier))
		require.NoError(t, err)
		requireUnchanged(t, ex, 1.5)
	})
	t.Run("gearbox", func(t *testing.T) {
		ex, err := gearboxSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(gbRing))
		require.NoError(t, err)
		requireUnchanged(t, ex, 0.95)
	})
	t.Run("flat and coaxial", func(t *testing.T) {
		ex, err := boltRingSet(t, 8).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(namePlate))
		require.NoError(t, err)
		requireUnchanged(t, ex, 13.0/60)
		require.True(t, ex.Spread.Coaxial)
		for k := range 8 {
			require.InDelta(t, 0, partByName(t, ex, boltName(k)).Offset.X, 1e-9, boltName(k))
		}
	})
	t.Run("thick and off the axis", func(t *testing.T) {
		// Arbors reaching 100 mm behind the plate make the assembly as tall as it is wide: the layers are
		// not coaxial, but the thinness gate keeps them stacked.
		s, _ := movementSet(t, 100)
		ex, err := s.Explode(t.Context(), movementOpts()...)
		require.NoError(t, err)
		requireUnchanged(t, ex, 122.0/120)
		require.False(t, ex.Spread.Coaxial)
	})
	t.Run("not derived", func(t *testing.T) {
		ex, err := housing(t).Explode(t.Context(), parts.WithExplodeGround(nameBottom))
		require.NoError(t, err)
		require.Nil(t, ex.Spread)
		for _, p := range ex.Parts {
			require.Zero(t, p.Layer, p.Name)
		}
	})
	t.Run("opt-out needs derived", func(t *testing.T) {
		_, err := housing(t).Explode(t.Context(), parts.WithoutExplodeSpread())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "WithDerivedExplode")
	})
}

func TestSpreadGuides(t *testing.T) {
	s, _ := movementSet(t, 10)
	ex, err := s.Explode(t.Context(), movementOpts()...)
	require.NoError(t, err)
	require.Len(t, ex.Guides, 9)
	guide := func(ex *parts.Exploded, name string) parts.Guide {
		for _, g := range ex.Guides {
			if g.Part == name {
				return g
			}
		}
		require.Failf(t, "no guide", "part %q has none", name)
		return parts.Guide{}
	}
	for _, g := range ex.Guides {
		require.NotEqual(t, mvPlate, g.Part)
		require.Equal(t, partByName(t, ex, g.Part).Anchor, g.Points[len(g.Points)-1], g.Part+" ends at its anchor")
	}
	requirePath := func(g parts.Guide, want ...r3.Vec) {
		t.Helper()
		require.Len(t, g.Points, len(want), g.Part)
		for i, w := range want {
			requireVec(t, w, g.Points[i], fmt.Sprintf("%s point %d", g.Part, i))
		}
	}
	requirePath(guide(ex, mvWheel(2)), r3.Vec{Z: 10.5}, r3.Vec{Z: 12.5}, r3.Vec{X: -18, Y: 57, Z: 12.5})
	requirePath(guide(ex, mvScrew(1)), r3.Vec{X: 45, Y: 15, Z: 13}, r3.Vec{X: 45, Y: 15, Z: 34}, r3.Vec{X: 68, Y: -7, Z: 34})
	requirePath(guide(ex, mvArbor(2)), r3.Vec{Z: 6}, r3.Vec{Z: -21}, r3.Vec{X: -26, Y: -81, Z: -21})

	doubled, err := s.Explode(t.Context(), movementOpts(parts.WithExplodeScale(units.Scalar(2)))...)
	require.NoError(t, err)
	requirePath(guide(doubled, mvWheel(2)), r3.Vec{Z: 10.5}, r3.Vec{Z: 14.5}, r3.Vec{X: -36, Y: 114, Z: 14.5})
	require.Equal(t, partByName(t, doubled, mvWheel(2)).Anchor, guide(doubled, mvWheel(2)).Points[2])
}
