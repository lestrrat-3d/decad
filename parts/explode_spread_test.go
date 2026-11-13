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
	mvStage  = "screw" // the explicit stage of the pressed movement
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
// Every part leaves along Z. The arbors reach tail mm behind the plate (10 in the fixture table). Each part is
// joined with a PinInBore whose declared axis runs along Z through the part's centre. When pressed, arbor 2 has
// no collar behind the plate and a collar under and one above wheel 2, so the two block each other along +Z.
// Coordinates are in mm.
func movementSet(t *testing.T, tail float64, pressed bool) (*parts.Set, map[string]*decad.Body) {
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
		if pressed && k == 2 {
			bodies[mvArbor(k)] = union(union(cyl(x, 0, 2, -tail, 22+tail), cyl(x, 0, 4, 7, 2)), cyl(x, 0, 4, 12, 1.5))
		}
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
	zAxis := func(x, y float64) *parts.Axis { return &parts.Axis{Point: r3.Vec{X: x, Y: y}, Dir: r3.Vec{Z: 1}} }
	for k := 1; k <= 3; k++ {
		x := mvArborX(k)
		mustJoin(t, s, parts.Joint{Name: mvArbor(k) + " in the plate", Kind: parts.PinInBore, A: mvArbor(k), B: mvPlate,
			Axis: zAxis(x, 0)})
	}
	for k := 1; k <= 3; k++ {
		mustJoin(t, s, parts.Joint{Name: mvWheel(k) + " on " + mvArbor(k), Kind: parts.PinInBore, A: mvWheel(k), B: mvArbor(k),
			Axis: zAxis(mvArborX(k), 0)})
	}
	for k := 1; k <= 3; k++ {
		mustJoin(t, s, parts.Joint{Name: mvArbor(k) + " in the bridge", Kind: parts.PinInBore, A: mvArbor(k), B: mvBridge,
			Axis: zAxis(mvArborX(k), 0)})
	}
	for k := 1; k <= 2; k++ {
		mustJoin(t, s, parts.Joint{Name: mvScrew(k) + " in the bridge", Kind: parts.PinInBore, A: mvScrew(k), B: mvBridge,
			Axis: zAxis(45, mvScrewY(k))})
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
	s, bodies := movementSet(t, 10, false)
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
		s, _ := movementSet(t, 100, false)
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

// mvGuide returns the guide of the named part.
func mvGuide(t *testing.T, ex *parts.Exploded, name string) parts.Guide {
	t.Helper()
	for _, g := range ex.Guides {
		if g.Part == name {
			return g
		}
	}
	require.Failf(t, "no guide", "part %q has none", name)
	return parts.Guide{}
}

func TestSpreadGuides(t *testing.T) {
	s, _ := movementSet(t, 10, false)
	ex, err := s.Explode(t.Context(), movementOpts()...)
	require.NoError(t, err)
	require.Len(t, ex.Guides, 9)
	want := []string{mvArbor(1), mvArbor(2), mvArbor(3), mvWheel(1), mvWheel(2), mvWheel(3), mvBridge, mvScrew(1), mvScrew(2)}
	for i, g := range ex.Guides {
		require.Equal(t, want[i], g.Part, "guides are in Parts order")
	}
	leaders := map[string]parts.Leader{}
	for _, l := range ex.Leaders {
		leaders[l.Joint] = l
	}
	for _, g := range ex.Guides {
		part, partner := partByName(t, ex, g.Part), partByName(t, ex, g.Partner)
		require.Equal(t, part.Anchor, g.To, g.Part+" ends at its anchor")
		requireVec(t, part.Offset.Sub(partner.Offset), g.To.Sub(g.From), g.Part+" runs by the offset less its partner's")
		require.Equal(t, units.Length, g.Size.Kind(), g.Part)
		l, ok := leaders[g.Joint]
		require.True(t, ok, "%s: the leader of joint %q", g.Part, g.Joint)
		if g.Part == mvBridge { // the bridge is B of its joint
			requireVec(t, l.To.Sub(l.From), g.To.Sub(g.From), g.Part+" against its leader")
		} else {
			requireVec(t, l.From.Sub(l.To), g.To.Sub(g.From), g.Part+" against its leader")
		}
	}
	for _, g := range []struct {
		part, partner, joint string
		from, to             r3.Vec
		size                 float64
	}{
		{mvWheel(2), mvArbor(2), "wheel 2 on arbor 2", r3.Vec{X: -26, Y: -81, Z: -16.5}, r3.Vec{X: -18, Y: 57, Z: 12.5}, 24},
		{mvArbor(2), mvPlate, "arbor 2 in the plate", r3.Vec{Z: 6}, r3.Vec{X: -26, Y: -81, Z: -21}, 32},
		{mvBridge, mvArbor(1), "arbor 1 in the bridge", r3.Vec{X: -26, Y: -81, Z: -10.5}, r3.Vec{X: 115, Y: -65, Z: 20.5}, 100},
		{mvScrew(1), mvBridge, "screw 1 in the bridge", r3.Vec{X: 160, Y: -50, Z: 17}, r3.Vec{X: 68, Y: -7, Z: 34}, 14},
	} {
		got := mvGuide(t, ex, g.part)
		require.Equal(t, g.partner, got.Partner, g.part)
		require.Equal(t, g.joint, got.Joint, g.part)
		requireVec(t, g.from, got.From, g.part+" from")
		requireVec(t, g.to, got.To, g.part+" to")
		require.InDelta(t, g.size, got.Size.Base(), 1e-6, g.part+" size")
	}
	require.Equal(t, mvPlate, ex.Ground)

	doubled, err := s.Explode(t.Context(), movementOpts(parts.WithExplodeScale(units.Scalar(2)))...)
	require.NoError(t, err)
	g := mvGuide(t, doubled, mvWheel(2))
	requireVec(t, r3.Vec{X: -52, Y: -162, Z: -43.5}, g.From, "scaled from")
	requireVec(t, r3.Vec{X: -36, Y: 114, Z: 14.5}, g.To, "scaled to")
	require.Equal(t, partByName(t, doubled, mvWheel(2)).Anchor, g.To)
}

// pressedOpts are the options of the pressed movement: arbor 2 and wheel 2 block each other, and screw 2
// leaves in an explicit stage.
func pressedOpts(extra ...parts.ExplodeOption) []parts.ExplodeOption {
	return movementOpts(append([]parts.ExplodeOption{parts.WithExplodeStages(
		parts.Stage{Name: mvStage, Parts: []string{mvScrew(2)}, Direction: r3.Vec{Z: 1}})}, extra...)...)
}

// pressedRow is a part's row of the pressed table in agent-docs/parts-design.md §5.9.
type pressedRow struct {
	stage        string
	layer, stack int
	d7, slide    r3.Vec
}

func pressedTable() map[string]pressedRow {
	return map[string]pressedRow{
		mvPlate:    {"", 0, 0, r3.Vec{}, r3.Vec{}},
		mvScrew(2): {mvStage, 1, 0, r3.Vec{Z: 17}, r3.Vec{X: -102, Y: 63}},
		mvWheel(2): {"removal 4.2", 2, 1, r3.Vec{Z: 21}, r3.Vec{X: -48, Y: 68, Z: 12}},
		mvArbor(2): {"removal 4.2", 2, 2, r3.Vec{Z: 21}, r3.Vec{X: -48, Y: 68, Z: 39}},
		mvArbor(1): {"removal 4", -1, 0, r3.Vec{Z: -27}, r3.Vec{X: -26, Y: -81}},
		mvArbor(3): {"removal 4", -1, 0, r3.Vec{Z: -27}, r3.Vec{X: -26, Y: -81}},
		mvWheel(1): {"removal 3", 3, 0, r3.Vec{Z: 2}, r3.Vec{X: -18, Y: 97, Z: 76}},
		mvWheel(3): {"removal 3", 3, 0, r3.Vec{Z: 33}, r3.Vec{X: -18, Y: 97, Z: 76}},
		mvBridge:   {"removal 2", 4, 0, r3.Vec{Z: 35}, r3.Vec{X: 115, Y: -65, Z: 76}},
		mvScrew(1): {"removal 1", 5, 0, r3.Vec{Z: 52}, r3.Vec{X: 23, Y: -52, Z: 76}},
	}
}

func TestSpreadStackedGroup(t *testing.T) {
	s, bodies := movementSet(t, 10, true)
	flat, err := s.Explode(t.Context(), pressedOpts(parts.WithoutExplodeSpread())...)
	require.NoError(t, err)
	a, w := partByName(t, flat, mvArbor(2)), partByName(t, flat, mvWheel(2))
	require.Equal(t, "removal 4.2", a.Stage)
	require.Equal(t, a.Stage, w.Stage)
	require.Equal(t, 4, a.Round)
	require.Equal(t, 4, w.Round)
	requireVec(t, r3.Vec{Z: 21}, a.Offset, "arbor 2 without the spread")
	requireVec(t, r3.Vec{Z: 21}, w.Offset, "wheel 2 without the spread")
	require.Equal(t, [][2]string{{mvArbor(2), mvWheel(2)}}, flat.Overlaps)
	for _, p := range flat.Parts {
		require.Zero(t, p.Stack, p.Name)
	}

	ex, err := s.Explode(t.Context(), pressedOpts()...)
	require.NoError(t, err)
	sp := ex.Spread
	require.NotNil(t, sp)
	require.True(t, sp.Ran)
	require.Equal(t, 7, sp.Layers)
	require.Equal(t, 5, sp.Columns)
	require.InDelta(t, 194, sp.Width.Base(), 1e-6)
	require.InDelta(t, 225, sp.Depth.Base(), 1e-6)
	for name, row := range pressedTable() {
		p, base := partByName(t, ex, name), partByName(t, flat, name)
		require.Equal(t, row.stage, p.Stage, name)
		require.Equal(t, row.layer, p.Layer, name+" layer")
		require.Equal(t, row.stack, p.Stack, name+" stack")
		requireVec(t, row.slide, p.Slide, name+" slide")
		requireVec(t, row.d7.Add(row.slide), p.Offset, name+" offset")
		requireVec(t, base.Offset, p.Offset.Sub(p.Slide), name+" offset less slide")
	}

	box := func(name string) decad.Box {
		b, err := bodies[name].Bounds()
		require.NoError(t, err)
		off := partByName(t, ex, name).Offset
		return decad.Box{Min: b.Min.Add(off), Max: b.Max.Add(off)}
	}
	require.InDelta(t, 5, box(mvArbor(2)).Min.Z-box(mvWheel(2)).Max.Z, 1e-6, "arbor 2 starts 5 mm above wheel 2 ends")
	aa, wa := partByName(t, ex, mvArbor(2)).Anchor, partByName(t, ex, mvWheel(2)).Anchor
	require.InDelta(t, wa.X, aa.X, 1e-9)
	require.InDelta(t, wa.Y, aa.Y, 1e-9)
	require.InDelta(t, -48, aa.X, 1e-6)
	require.InDelta(t, 68, aa.Y, 1e-6)
	require.Empty(t, ex.Overlaps)

	hidden, err := ex.Hidden(sp.View)
	require.NoError(t, err)
	require.Len(t, hidden, 1)
	require.Equal(t, mvArbor(2), hidden[0].Front)
	require.Equal(t, mvWheel(2), hidden[0].Back)
	require.InDelta(t, 473.0/2025, hidden[0].Covered.Base(), 1e-9)

	g := mvGuide(t, ex, mvWheel(2))
	require.Equal(t, mvArbor(2), g.Partner)
	requireVec(t, r3.Vec{X: -48, Y: 68, Z: 70.5}, g.From, "wheel 2 guide from")
	requireVec(t, r3.Vec{X: -48, Y: 68, Z: 43.5}, g.To, "wheel 2 guide to")
}

func TestSpreadExplicitLayer(t *testing.T) {
	s, _ := movementSet(t, 10, true)
	ex, err := s.Explode(t.Context(), pressedOpts()...)
	require.NoError(t, err)
	screw := partByName(t, ex, mvScrew(2))
	require.Equal(t, mvStage, screw.Stage)
	require.Zero(t, screw.Order)
	require.Equal(t, 1, screw.Layer)
	requireVec(t, r3.Vec{X: -102, Y: 63}, screw.Slide, "screw 2 slide")
	requireVec(t, r3.Vec{X: -102, Y: 63, Z: 17}, screw.Offset, "screw 2 offset")
	for _, p := range ex.Parts {
		if p.Name != mvPlate {
			require.NotZero(t, p.Layer, p.Name+" shares no cell with the plate")
		}
	}
	require.Zero(t, partByName(t, ex, mvPlate).Layer)
	for name, layer := range map[string]int{mvArbor(1): -1, mvArbor(3): -1, mvWheel(1): 3, mvWheel(3): 3, mvBridge: 4, mvScrew(1): 5} {
		require.Equal(t, layer, partByName(t, ex, name).Layer, name)
	}
	hidden, err := ex.Hidden(ex.Spread.View)
	require.NoError(t, err)
	for _, h := range hidden {
		require.NotEqual(t, mvPlate, h.Front)
		require.NotEqual(t, mvPlate, h.Back)
	}
}
