package retainedblend_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file builds a closed chamfer chain on walls whose tags really lie off the walls the held mesh follows
// (agent-docs/charged-window-chamfer-design.md §3). The held mesh is a rounded plate. Its tags are displaced:
// each corner cylinder's radius differs from the plate's by up to its tag displacement, which a stated AxialDelta covers; the
// line between two corners of different radii is their common tangent, which tilts, and is stated as an oblique
// plane with its TagDeparture and ExtentDelta; the line between two corners of equal radius is a parallel offset
// with a stated AxialDelta. The exact chamfer of the plate, unmoved, is the ground truth.

const (
	twinW, twinD, twinH = 60.0, 40.0, 20.0
	twinR               = 12.0
	// twinArc is the chord count of one corner arc.
	twinArc = 128
)

// twinCfg is one fixture: tag is the displacement of every tag, size the chamfer's setback and beta the bound the
// held mesh states for every outline vertex.
type twinCfg struct{ tag, size, beta float64 }

// twinCoarse displaces the tags by 2e-4. The chain's slack sums planeAmp times each charged face's displacement,
// about 8 mm, inside the corner circles' R - t, and the setback is small so that the grid's own tolerance, 2^-10 of
// it, lies below the displacement.
var twinCoarse = twinCfg{tag: 2e-4, size: 0.05, beta: 8e-4}

// twinLoose barely displaces the tags and states a large vertex bound, as a stacked view's vertices do: the junction
// shift e_J of §3.3 then exceeds every other term.
var twinLoose = twinCfg{tag: 1e-9, size: 0.5, beta: 2e-3}

// radii are the corner tags' radius changes, corners 0 to 3: the lines between corners 0 and 1 and between 1 and 2
// tilt, and the lines between corners 2 and 3 and between 3 and 0 are offset.
func (c twinCfg) radii() [4]float64 { return [4]float64{c.tag, -c.tag, c.tag, c.tag} }

// twinCentre is corner c's centre and the unit direction from it to the corner's first point.
func twinCentre(c int) (centre, first r3.Vec) {
	switch c {
	case 0:
		return r3.NewVec(twinW-twinR, twinR, 0), r3.NewVec(0, -1, 0)
	case 1:
		return r3.NewVec(twinW-twinR, twinD-twinR, 0), r3.NewVec(1, 0, 0)
	case 2:
		return r3.NewVec(twinR, twinD-twinR, 0), r3.NewVec(0, 1, 0)
	}
	return r3.NewVec(twinR, twinR, 0), r3.NewVec(-1, 0, 0)
}

// twinPoint is point k of corner c's arc at height z; the end points are exact.
func twinPoint(c, k int, z float64) r3.Vec {
	centre, first := twinCentre(c)
	if k == 0 {
		return centre.Add(first.Scale(twinR)).Add(r3.NewVec(0, 0, z))
	}
	// The quarter turn counter-clockwise takes first to its left normal.
	left := r3.NewVec(-first.Y, first.X, 0)
	if k == twinArc {
		return centre.Add(left.Scale(twinR)).Add(r3.NewVec(0, 0, z))
	}
	a := float64(k) * math.Pi / 2 / twinArc
	return centre.Add(first.Scale(twinR * math.Cos(a))).Add(left.Scale(twinR * math.Sin(a))).Add(r3.NewVec(0, 0, z))
}

// twinLineTag is the tag plane of the line wall between corners a and b, whose true wall faces the unit vector v:
// the outer common tangent of the two corner tags, as a frame whose normal is the plane's. It also returns the
// distance from the tag plane to the true wall's two ends.
func (c twinCfg) lineTag(t *testing.T, a int, v r3.Vec) (frame r3.Frame, tilt, extent float64) {
	t.Helper()
	b := (a + 1) % 4
	ca, _ := twinCentre(a)
	cb, _ := twinCentre(b)
	rs := c.radii()
	ra, rb := twinR+rs[a], twinR+rs[b]
	l := cb.Sub(ca).Len()
	u := cb.Sub(ca).Scale(1 / l)
	// n . u = (ra - rb) / l, so the plane n . x = n . ca + ra touches both circles.
	s := (ra - rb) / l
	n := u.Scale(s).Add(v.Scale(math.Sqrt(1 - s*s)))
	o := ca.Add(n.Scale(ra))
	f, err := r3.NewFrame(o, r3.NewVec(0, 0, 1), n.Cross(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	h := n.Dot(o)
	ends := []r3.Vec{ca.Add(v.Scale(twinR)), cb.Add(v.Scale(twinR))}
	for _, p := range ends {
		extent = math.Max(extent, math.Abs(n.Dot(p)-h))
	}
	return f, n.Sub(v).Len(), extent
}

// twinPlate holds the rounded plate with displaced tags.
func (cfg twinCfg) plate(t *testing.T) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	per := 4 * (twinArc + 1)
	var fp facetedrecord.Payload
	for _, z := range []float64{0, twinH} {
		for c := range 4 {
			for k := 0; k <= twinArc; k++ {
				fp.Verts = append(fp.Verts, twinPoint(c, k, z))
				fp.VertexBound = append(fp.VertexBound, cfg.beta)
			}
		}
	}
	// groups: 0 top, 1 bottom, 2..5 the corner cylinders, 6..9 the lines.
	const top, bottom, cyl0, line0 = 0, 1, 2, 6
	tri := func(g, a, b, c int) {
		fp.Tris = append(fp.Tris, [3]int{a, b, c})
		fp.Src = append(fp.Src, g)
	}
	for i := 1; i+1 < per; i++ {
		tri(top, per, per+i, per+i+1)
		tri(bottom, 0, i+1, i)
	}
	for i := range per {
		j := (i + 1) % per
		g := cyl0 + i/(twinArc+1)
		if i%(twinArc+1) == twinArc {
			g = line0 + i/(twinArc+1)
		}
		tri(g, i, j, j+per)
		tri(g, i, j+per, i+per)
	}
	fp.Groups = []facetedrecord.Group{
		axialPlane(t, "top", r3.NewVec(0, 0, twinH), r3.NewVec(1, 0, twinH), r3.NewVec(0, 1, twinH)),
		axialPlane(t, "bottom", r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0)),
	}
	for c := range 4 {
		centre, _ := twinCentre(c)
		g := axialPlane(t, "corner", r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		g.Planar = false
		g.Surface = surfacegeom.Cylinder{Origin: centre, Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(twinR + cfg.radii()[c])}
		g.HasAxialDelta, g.AxialDelta = true, math.Abs(cfg.radii()[c])
		// The fixture stands for the stacked view, whose charge loop marks the wall it charges.
		g.ViewCharge = true
		fp.Groups = append(fp.Groups, g)
	}
	faces := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0), r3.NewVec(0, -1, 0)}
	for a := range 4 {
		frame, tilt, extent := cfg.lineTag(t, a, faces[a])
		g := axialPlane(t, "line", r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		g.Surface = surfacegeom.Plane{Frame: frame}
		if tilt > 1e-3*cfg.tag {
			// An oblique wall: its tilt and its extent bound, no AxialDelta (charged-window-chamfer-design.md §2).
			g.NormalBound = math.Nextafter(tilt*1.001, math.Inf(1))
			g.ExtentDelta = math.Nextafter(extent*1.001, math.Inf(1))
		} else {
			g.HasAxialDelta, g.AxialDelta = true, extent
		}
		fp.Groups = append(fp.Groups, g)
	}
	// The held mesh follows the chords of the true arcs: its volume differs from the plate's by the circular
	// segments.
	theta := math.Pi / 2 / twinArc
	segment := twinR * twinR / 2 * (theta - math.Sin(theta))
	fp.VolSymDiff = 1.001 * 4 * twinArc * segment * twinH
	fp.DPair = r3.NewVec(twinW, twinD, twinH).Len()
	b, err := facetedrecord.Build(t.Context(), axialEnv(), fp)
	require.NoError(t, err)
	return b, payloadOf(t, b)
}

// twinOutline are the edges of the top face's outer loop.
func twinOutline(t *testing.T, b *topo.Body) []*topo.Edge {
	t.Helper()
	var out []*topo.Edge
	for _, e := range topobuild.Edges(b) {
		if math.Abs(e.Start.Position.Z-twinH) > 1e-9 || math.Abs(e.End.Position.Z-twinH) > 1e-9 || e.Start == e.End {
			continue
		}
		onTop := false
		for _, f := range e.Faces {
			if p, ok := f.Surface.(surfacegeom.Plane); ok && p.Frame.N().Z > 0.5 {
				onTop = true
			}
		}
		if onTop {
			out = append(out, e)
		}
	}
	return out
}

// twinExact is the exact volume and area change of the unmoved plate's chamfer of the top outline.
func (cfg twinCfg) exact() (volume, area float64) {
	t := cfg.size
	straight := 2*(twinW-2*twinR) + 2*(twinD-2*twinR)
	volume = straight*t*t/2 + 2*math.Pi*t*t/2*(twinR-t/3)
	rho := twinR - t/2
	area = straight*t*(math.Sqrt2-2) + 2*math.Pi*t*(math.Sqrt2*rho-rho-twinR)
	return volume, area
}

// meshArea is the held area of the facets.
func meshArea(fp facetedrecord.Payload) float64 {
	a := 0.0
	for _, tri := range fp.Tris {
		p, q, r := fp.Verts[tri[0]], fp.Verts[tri[1]], fp.Verts[tri[2]]
		a += q.Sub(p).Cross(r.Sub(p)).Len() / 2
	}
	return a
}

// twinDistance is the distance from p to the true chamfer surface of the unmoved plate's top outline: the points
// whose inset from the plate's outline plus their depth below the top is the setback. Both grow at unit rate along
// orthogonal directions, so the distance is the excess over sqrt 2.
func (cfg twinCfg) distance(p r3.Vec) float64 {
	qx := math.Abs(p.X-twinW/2) - (twinW/2 - twinR)
	qy := math.Abs(p.Y-twinD/2) - (twinD/2 - twinR)
	sd := math.Hypot(math.Max(qx, 0), math.Max(qy, 0)) - twinR + math.Min(math.Max(qx, qy), 0)
	return math.Abs(-sd+(twinH-p.Z)-cfg.size) / math.Sqrt2
}

// twinRun chamfers the fixture's top outline.
func (cfg twinCfg) run(t *testing.T) (fp, got facetedrecord.Payload) {
	t.Helper()
	b, fp := cfg.plate(t)
	outline := twinOutline(t, b)
	require.Len(t, outline, 8)
	for _, e := range outline {
		require.True(t, e.RetainedCurve, "the fixture's outline edges are retained curves")
	}
	out, recognized, err := retainedblend.Chamfer(t.Context(), axialEnv(), b, fp, outline, cfg.size, 0, nil)
	require.True(t, recognized)
	require.NoError(t, err)
	return fp, payloadOf(t, out)
}

// junctionBounds are the bounds a result states at the junctions' base row (P_J) and wall cut (S_J), and the
// smallest bound at the same rows away from every junction.
type junctionBounds struct{ rowJ, rowInterior, cutJ, cutInterior float64 }

// junctionBoundsOf reads them from the strip's own vertices: P_J lies at the setback from a tangent point on the
// top face, S_J on the wall at the setback height.
func (cfg twinCfg) junctionBounds(fp, got facetedrecord.Payload) junctionBounds {
	var tangent []r3.Vec
	for c := range 4 {
		tangent = append(tangent, twinPoint(c, 0, twinH), twinPoint(c, twinArc, twinH))
	}
	near := func(p r3.Vec, height, radius float64) bool {
		if math.Abs(p.Z-height) > 1e-9 {
			return false
		}
		for _, q := range tangent {
			if math.Hypot(p.X-q.X, p.Y-q.Y) < radius {
				return true
			}
		}
		return false
	}
	jb := junctionBounds{rowInterior: math.Inf(1), cutInterior: math.Inf(1)}
	for i, tri := range got.Tris {
		if got.Src[i] < len(fp.Groups) {
			continue
		}
		for _, v := range tri {
			p, b := got.Verts[v], got.VertexBound[v]
			switch {
			case near(p, twinH, cfg.size+2e-3) && !near(p, twinH, cfg.size-2e-3):
				jb.rowJ = max(jb.rowJ, b)
			case math.Abs(p.Z-twinH) < 1e-9 && !near(p, twinH, 0.5):
				jb.rowInterior = min(jb.rowInterior, b)
			case near(p, twinH-cfg.size, 2e-3):
				jb.cutJ = max(jb.cutJ, b)
			case math.Abs(p.Z-(twinH-cfg.size)) < 1e-9 && !near(p, twinH-cfg.size, 0.5):
				jb.cutInterior = min(jb.cutInterior, b)
			}
		}
	}
	return jb
}

func TestChainChamferOnDisplacedTags(t *testing.T) {
	t.Parallel()
	cfg := twinCoarse
	fp, got := cfg.run(t)
	wantVol, wantArea := cfg.exact()

	t.Run("the held volume stays within the result's bound of the exact chamfer", func(t *testing.T) {
		t.Parallel()
		trueBefore := twinH * (twinW*twinD - (4-math.Pi)*twinR*twinR)
		require.InDelta(t, trueBefore, meshVolume(fp), fp.VolSymDiff)
		require.InDelta(t, trueBefore-wantVol, meshVolume(got), got.VolSymDiff)
	})

	t.Run("the held area change stays within the area bound's growth of the exact change", func(t *testing.T) {
		t.Parallel()
		growth := got.AreaSlack - fp.AreaSlack
		change := meshArea(got) - meshArea(fp)
		// The tags' setback contour lies off the held outline by the displacement along the whole outline, far
		// above the grid's own rounding, so the growth must carry the charge.
		require.Greater(t, math.Abs(wantArea-change), 1e-3)
		require.InDelta(t, wantArea, change, growth)
		// The growth holds at least the base band's boundary tube of §3.3, 2·P·σ with P the outline's length and
		// σ at least 3δ: the area term itself, apart from the grid's.
		perimeter := 2*(twinW-2*twinR) + 2*(twinD-2*twinR) + 2*math.Pi*twinR
		require.GreaterOrEqual(t, growth, 2*perimeter*3*cfg.tag)
	})

	t.Run("every strip vertex's bound covers its distance from the exact chamfer surface", func(t *testing.T) {
		t.Parallel()
		strip, worst := 0, 0.0
		for i, tri := range got.Tris {
			if got.Src[i] < len(fp.Groups) {
				continue
			}
			for _, v := range tri {
				strip++
				d := cfg.distance(got.Verts[v])
				worst = max(worst, d)
				require.LessOrEqual(t, d, got.VertexBound[v], "vertex %d at %v", v, got.Verts[v])
			}
		}
		require.Positive(t, strip)
		// The displacement puts the arcs' strip vertices off the exact surface by more than the grid tolerance, so
		// the bound's own slack is what covers them.
		require.Greater(t, worst, cfg.size/1024)
	})

	t.Run("the strips' bounds carry every stated charge and the crease", func(t *testing.T) {
		t.Parallel()
		// The chain's slack sums planeAmp = 4/minSine = 4000 times the displacement every charged face of the held
		// body states, its cylinders, displaced planes and oblique walls alike (§3.3, "Slack").
		stated := 0.0
		for _, g := range fp.Groups {
			stated += g.AxialDelta + g.ExtentDelta
		}
		require.GreaterOrEqual(t, stated, 4*cfg.tag, "the four corner cylinders state a displacement")
		// Each junction's true walls may turn by at least the cylinder's (e_J + 2δ)/R, with e_J at least the held
		// junction vertex's bound, so the slack also holds t·ψ²/4 of it (§3.2).
		psi := (cfg.beta + 2*cfg.tag) / (twinR + cfg.tag)
		crease := cfg.size * psi * psi / 4
		jb := cfg.junctionBounds(fp, got)
		require.False(t, math.IsInf(jb.rowInterior, 1))
		require.GreaterOrEqual(t, jb.rowInterior, cfg.size/1024+4000*stated+0.99*crease)
	})

	t.Run("the strips are published as the walls allow", func(t *testing.T) {
		t.Parallel()
		// Lines 0 and 1 stand on oblique walls: their strips state no tag. Lines 2 and 3 stand on displaced planes
		// and keep a plane tag that states a displacement; the arcs' strips are cones, which stay faceted beside a
		// charged cylinder.
		planes, untagged := 0, 0
		for _, g := range got.Groups[len(fp.Groups):] {
			switch s := g.Surface.(type) {
			case nil:
				untagged++
			case surfacegeom.Plane:
				planes++
				require.True(t, g.HasAxialDelta)
				require.Positive(t, g.AxialDelta)
			default:
				require.Failf(t, "strip", "a strip published %T", s)
			}
		}
		require.Equal(t, 2, planes)
		require.Equal(t, 6, untagged)
	})
}

// TestChainChamferJunctionShift states a large vertex bound beside barely displaced tags, so that the junction shift
// e_J = β(v) + |v − V| exceeds the slack: each junction row's base point P_J and its wall cut S_J then lie that much
// further from the exact section than the rest of their rows (charged-window-chamfer-design.md §3.3).
func TestChainChamferJunctionShift(t *testing.T) {
	t.Parallel()
	cfg := twinLoose
	fp, got := cfg.run(t)
	jb := cfg.junctionBounds(fp, got)
	require.False(t, math.IsInf(jb.rowInterior, 1))
	require.False(t, math.IsInf(jb.cutInterior, 1))
	require.GreaterOrEqual(t, jb.rowJ-jb.rowInterior, cfg.beta, "P_J")
	require.GreaterOrEqual(t, jb.cutJ-jb.cutInterior, cfg.beta, "S_J")
	// The slack stays far below the shift, so the shift is what separates them.
	require.Less(t, jb.rowInterior, cfg.size/1024+4000*(4*cfg.tag+6*cfg.tag)+1e-6)

	wantVol, wantArea := cfg.exact()
	trueBefore := twinH * (twinW*twinD - (4-math.Pi)*twinR*twinR)
	require.InDelta(t, trueBefore-wantVol, meshVolume(got), got.VolSymDiff)
	require.InDelta(t, wantArea, meshArea(got)-meshArea(fp), got.AreaSlack-fp.AreaSlack)
}
