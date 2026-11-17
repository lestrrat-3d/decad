package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers a closed chamfer chain on walls that state a displacement
// (agent-docs/charged-window-chamfer-design.md §3): the bottom-face windows of the spoked hub disc after its
// Ø14 boss with a root fillet. The boss fillet charges every circular wall and leaves the windows' oblique
// sides as extent planes (§2), which only a closed chamfer chain reads.

const (
	chargedChamfer = 0.5
	// windowInner is the radius of a window's inner arc, the one wall whose material lies inside its circle.
	windowInner = 7.0
)

// filletedHub is the spoked hub disc with a Ø14 boss whose root is filleted by 0.5, and its bottom face.
func filletedHub(t *testing.T) (*decad.Body, *decad.Face) {
	t.Helper()
	_, disc := spokedHubDisc(t)
	part, err := bossOnTop(t, disc, 14, 0.5)
	require.NoError(t, err)
	return part, bottomPlane(t, part)
}

// bottomPlane is the planar face of a body whose outward normal is -z.
func bottomPlane(t *testing.T, b *decad.Body) *decad.Face {
	t.Helper()
	var found *decad.Face
	for _, f := range b.Faces() {
		if p, ok := f.Surface().(decad.Plane); ok && p.Frame.N().Z < -0.5 {
			require.Nil(t, found, "one bottom face")
			found = f
		}
	}
	require.NotNil(t, found)
	return found
}

// windowLoopEdges are the edges of the face's loops that hold eight edges: the four window loops.
func windowLoopEdges(f *decad.Face) [][]*decad.Edge {
	var out [][]*decad.Edge
	for _, l := range f.Loops() {
		if es := l.Edges(); len(es) == 8 {
			out = append(out, es)
		}
	}
	return out
}

// namedEdges selects exactly the given edges by their end points.
func namedEdges(edges []*decad.Edge) decad.EdgeSelector {
	var q *decad.EdgeQuery
	for _, e := range edges {
		pr := []decad.EdgePredicate{decad.EndpointAt(e.Start().Position().Value), decad.EndpointAt(e.End().Position().Value)}
		if q == nil {
			q = decad.Edges(pr...)
			continue
		}
		q.Or(pr...)
	}
	return q.Exactly(len(edges))
}

// chamferOfLoop is the exact change a chamfer of size t makes along edges whose material lies on the stated side,
// each edge a line or an arc of a circle about the z axis (agent-docs/charged-window-chamfer-design.md §3.1): the
// removed volume and the change in surface area. A line of length L removes L·t²/2 and an arc of span φ and radius
// r, whose material lies outside the circle (k = +1) or inside it (k = -1), removes φ·t²/2·(r + k·t/3): Pappus on
// the triangle of the section. The area changes by the bevel less the base and wall bands, φ·t·(√2·ρ − ρ − r) with
// ρ = r + k·t/2 for an arc and L·t·(√2 − 2) for a line.
func chamferOfLoop(t *testing.T, edges []*decad.Edge) (volume, area float64) {
	t.Helper()
	const size = chargedChamfer
	for _, e := range edges {
		a, b := e.Start().Position().Value, e.End().Position().Value
		switch c := e.Curve().(type) {
		case decad.Line3:
			l := a.Sub(b).Len()
			volume += l * size * size / 2
			area += l * size * (math.Sqrt2 - 2)
		case decad.Arc3:
			r := c.Radius.Base()
			k := 1.0
			if math.Abs(r-windowInner) < 1e-9 {
				k = -1
			}
			u, v := a.Sub(c.Center), b.Sub(c.Center)
			span := math.Atan2(u.Cross(v).Len(), u.Dot(v))
			volume += span * size * size / 2 * (r + k*size/3)
			rho := r + k*size/2
			area += span * size * (math.Sqrt2*rho - rho - r)
		case decad.Circle3:
			r := c.Radius.Base()
			volume += 2 * math.Pi * size * size / 2 * (r - size/3)
			rho := r - size/2
			area += 2 * math.Pi * size * (math.Sqrt2*rho - rho - r)
		default:
			require.Failf(t, "edge", "unexpected curve %T", c)
		}
	}
	return volume, area
}

// bottomRim is the bottom face's outer rim circle.
func bottomRim(t *testing.T, f *decad.Face) *decad.Edge {
	t.Helper()
	for _, l := range f.Loops() {
		if es := l.Edges(); len(es) == 1 {
			return es[0]
		}
	}
	require.Fail(t, "no rim loop")
	return nil
}

func TestChargedChainChamferWindowLoop(t *testing.T) {
	t.Parallel()
	part, bottom := filletedHub(t)
	windows := windowLoopEdges(bottom)
	require.Len(t, windows, 4)
	vol, err := part.Volume()
	require.NoError(t, err)
	area, err := part.Area()
	require.NoError(t, err)
	wantVol, wantArea := chamferOfLoop(t, windows[0])

	// The twin is the same disc before the boss: its tags state no displacement and its window loop chamfers
	// through the exact route.
	_, disc := spokedHubDisc(t)
	twinWindows := windowLoopEdges(bottomPlane(t, disc))
	twinVol, err := disc.Volume()
	require.NoError(t, err)
	twin, err := decad.Chamfer(t.Context(), disc, namedEdges(twinWindows[0]), units.Millimeters(chargedChamfer))
	require.NoError(t, err)
	twinAfter, err := twin.Volume()
	require.NoError(t, err)
	require.InDelta(t, wantVol, twinVol.Value.Mag()-twinAfter.Value.Mag(), 1e-6,
		"the exact formula agrees with the uncharged twin")

	facetedBefore := 0
	for _, f := range part.Faces() {
		if _, ok := f.Surface().(decad.Faceted); ok {
			facetedBefore++
		}
	}
	out, err := decad.Chamfer(t.Context(), part, namedEdges(windows[0]), units.Millimeters(chargedChamfer))
	require.NoError(t, err)
	after, err := out.Volume()
	require.NoError(t, err)
	areaAfter, err := out.Area()
	require.NoError(t, err)
	requireVolumeChange(t, vol, after, wantVol)
	requireAreaChange(t, area, areaAfter, wantArea)
	requireVerifiedMesh(t, out)

	t.Run("the bounds carry the junction vertices' own bound", func(t *testing.T) {
		t.Parallel()
		// Each junction row's vertices lie up to the held junction vertex's bound from the true section
		// (§3.3), so the volume bound grows by at least that bound times the exact bevel area.
		beta := 0.0
		for _, e := range windows[0] {
			beta = math.Max(beta, e.Start().Position().Bound.Mag())
		}
		require.Positive(t, beta)
		bevel := 0.0
		for _, e := range windows[0] {
			a, b := e.Start().Position().Value, e.End().Position().Value
			switch c := e.Curve().(type) {
			case decad.Line3:
				bevel += a.Sub(b).Len() * chargedChamfer * math.Sqrt2
			case decad.Arc3:
				r := c.Radius.Base()
				k := 1.0
				if math.Abs(r-windowInner) < 1e-9 {
					k = -1
				}
				u, v := a.Sub(c.Center), b.Sub(c.Center)
				span := math.Atan2(u.Cross(v).Len(), u.Dot(v))
				bevel += span * (r + k*chargedChamfer/2) * chargedChamfer * math.Sqrt2
			}
		}
		growth := after.Bound.Mag() - vol.Bound.Mag()
		require.GreaterOrEqual(t, growth, beta*bevel)
		// The uncharged twin states only the grid's own tolerance.
		require.Less(t, twinAfter.Bound.Mag()-twinVol.Bound.Mag(), 1e-6)
	})

	t.Run("a strip beside an oblique wall or a charged cylinder stays faceted", func(t *testing.T) {
		t.Parallel()
		facetedAfter := 0
		for _, f := range out.Faces() {
			if _, ok := f.Surface().(decad.Faceted); ok {
				facetedAfter++
			}
		}
		// The window's eight strips are new faces and none states a tag: the two beside an oblique wall
		// (no plane AxialDelta states their tilt) and the six cones beside a charged cylinder.
		require.Equal(t, facetedBefore+8, facetedAfter)
		for _, f := range out.Faces() {
			if c, ok := f.Surface().(decad.Cone); ok {
				require.Failf(t, "cone", "a strip beside a charged cylinder published a cone of radius %v", c.Radius)
			}
		}
	})

	t.Run("each window builds alone", func(t *testing.T) {
		t.Parallel()
		part, bottom := filletedHub(t)
		w := windowLoopEdges(bottom)
		got, err := decad.Chamfer(t.Context(), part, namedEdges(w[3]), units.Millimeters(chargedChamfer))
		require.NoError(t, err)
		after, err := got.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, vol, after, wantVol)
	})
}

func TestChargedChainChamferMixed(t *testing.T) {
	t.Parallel()
	// Each case builds its own body: a successful Chamfer retires its receiver.
	fresh := func(t *testing.T) (*decad.Body, [][]*decad.Edge, *decad.Edge, decad.Measurement, decad.Measurement) {
		t.Helper()
		part, bottom := filletedHub(t)
		vol, err := part.Volume()
		require.NoError(t, err)
		area, err := part.Area()
		require.NoError(t, err)
		return part, windowLoopEdges(bottom), bottomRim(t, bottom), vol, area
	}

	t.Run("a window loop and the outer rim circle in one selection", func(t *testing.T) {
		t.Parallel()
		part, windows, rim, vol, area := fresh(t)
		sel := append(append([]*decad.Edge(nil), windows[0]...), rim)
		wantVol, wantArea := chamferOfLoop(t, sel)
		out, err := decad.Chamfer(t.Context(), part, namedEdges(sel), units.Millimeters(chargedChamfer))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		areaAfter, err := out.Area()
		require.NoError(t, err)
		requireVolumeChange(t, vol, after, wantVol)
		requireAreaChange(t, area, areaAfter, wantArea)
		requireVerifiedMesh(t, out)
	})

	t.Run("two window loops and the rim", func(t *testing.T) {
		t.Parallel()
		part, windows, rim, vol, area := fresh(t)
		sel := append(append(append([]*decad.Edge(nil), windows[0]...), windows[2]...), rim)
		wantVol, wantArea := chamferOfLoop(t, sel)
		out, err := decad.Chamfer(t.Context(), part, namedEdges(sel), units.Millimeters(chargedChamfer))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		areaAfter, err := out.Area()
		require.NoError(t, err)
		requireVolumeChange(t, vol, after, wantVol)
		requireAreaChange(t, area, areaAfter, wantArea)
	})
}

func TestChargedChainChamferEveryRimAndWindow(t *testing.T) {
	t.Parallel()
	// The balance wheel's selection: the four bottom windows and the outer rims of both caps.
	part, bottom := filletedHub(t)
	var sel []*decad.Edge
	for _, w := range windowLoopEdges(bottom) {
		sel = append(sel, w...)
	}
	sel = append(sel, bottomRim(t, bottom))
	var topRim *decad.Edge
	for _, f := range part.Faces() {
		if p, ok := f.Surface().(decad.Plane); ok && p.Frame.N().Z > 0.5 && math.Abs(p.Frame.Origin().Z-5) < 1e-9 {
			for _, l := range f.Loops() {
				if es := l.Edges(); len(es) == 1 {
					topRim = es[0]
				}
			}
		}
	}
	require.NotNil(t, topRim)
	sel = append(sel, topRim)
	vol, err := part.Volume()
	require.NoError(t, err)
	area, err := part.Area()
	require.NoError(t, err)
	wantVol, wantArea := chamferOfLoop(t, sel)
	out, err := decad.Chamfer(t.Context(), part, namedEdges(sel), units.Millimeters(chargedChamfer))
	require.NoError(t, err)
	after, err := out.Volume()
	require.NoError(t, err)
	areaAfter, err := out.Area()
	require.NoError(t, err)
	requireVolumeChange(t, vol, after, wantVol)
	requireAreaChange(t, area, areaAfter, wantArea)
}

func TestChargedChainChamferRefusals(t *testing.T) {
	t.Parallel()
	part, bottom := filletedHub(t)
	windows := windowLoopEdges(bottom)

	t.Run("an open window run on the top face", func(t *testing.T) {
		t.Parallel()
		var top *decad.Face
		for _, f := range part.Faces() {
			if p, ok := f.Surface().(decad.Plane); ok && p.Frame.N().Z > 0.5 && math.Abs(p.Frame.Origin().Z-5) < 1e-9 {
				top = f
			}
		}
		require.NotNil(t, top)
		// The +x window's run of seven edges, which ends where a corner round meets the boss's root fillet.
		var run []*decad.Edge
		for _, l := range top.Loops() {
			for _, e := range l.Edges() {
				a, b := e.Start().Position().Value, e.End().Position().Value
				if _, rim := e.Curve().(decad.Circle3); rim || math.Min(a.X, b.X) < 6.5 ||
					math.Max(math.Abs(a.Y), math.Abs(b.Y)) > 9.6 || false {
					continue
				}
				run = append(run, e)
			}
		}
		require.Len(t, run, 7)
		_, err := decad.Chamfer(t.Context(), part, namedEdges(run), units.Millimeters(chargedChamfer))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "an open tangent chain on walls that state a displacement is not built")
	})

	t.Run("a fillet of a bottom window loop", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Fillet(t.Context(), part, namedEdges(windows[0]), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a tangent chain fillet on walls that state a displacement is not built")
	})

	t.Run("one window edge alone", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Chamfer(t.Context(), part, namedEdges(windows[0][:1]), units.Millimeters(chargedChamfer))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "whose continuation is not selected")
	})

	t.Run("a window loop missing one edge", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Chamfer(t.Context(), part, namedEdges(windows[0][:7]), units.Millimeters(chargedChamfer))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("a setback inside the inner arc by less than the chain's slack", func(t *testing.T) {
		t.Parallel()
		// R - t is positive but below the chain's slack, which the true wall's displacement and the junction shifts set.
		_, err := decad.Chamfer(t.Context(), part, namedEdges(windows[0]), units.Millimeters(windowInner-1e-9))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "the blend reaches the rim's axis")
	})

	t.Run("a setback that reaches the inner arc's axis", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Chamfer(t.Context(), part, namedEdges(windows[0]), units.Millimeters(windowInner))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
}

// TestChainWithTouchingEdgeKeepsItsRefusal selects the cross-drilled rounded box's top outline together with the
// vertical seam below one of its junctions. The seam shares a vertex with a chain edge and has no link of its own, so
// the selection keeps the refusal of a selection that names a chain and other edges.
func TestChainWithTouchingEdgeKeepsItsRefusal(t *testing.T) {
	t.Parallel()
	drilled := crossDrilled(t)
	seam := decad.Edges(decad.EndpointAt(r3.NewVec(3, 0, 20)), decad.ParallelTo(r3.NewVec(0, 0, 1)))
	n := 1
	for _, e := range drilled.Edges() {
		p, q := e.Start().Position().Value, e.End().Position().Value
		if math.Abs(p.Z-20) < 1e-9 && math.Abs(q.Z-20) < 1e-9 {
			seam.Or(decad.EndpointAt(p), decad.EndpointAt(q))
			n++
		}
	}
	require.Equal(t, 9, n)
	_, err := decad.Chamfer(t.Context(), drilled, seam.Exactly(9), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "a selection that holds a tangent chain names only chain edges")
}
