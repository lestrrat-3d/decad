package retainedblend_test

import (
	"context"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests follow a charged cylinder wall down the producer chain: a retained fillet publishes the wall with its
// displacement (agent-docs/retained-edge-blend-design.md §6.1, "Publishing a charged wall"), and a later mesh Cut,
// placement, scale and chamfer read it again. Every stage runs a charged body and its uncharged twin (no
// displacement), so a displacement dropped anywhere turns a comparison red.

// chargedPair is the charged body and its uncharged twin at one stage of the chain.
type chargedPair struct {
	charged, twin *topo.Body
}

// rimBlend is the result of a top rim fillet of holedPlate with the wall displacement delta, and of its twin.
func rimBlend(t *testing.T, delta float64) chargedPair {
	t.Helper()
	fillet := func(wallDelta float64, hasDelta bool) *topo.Body {
		b, built := holedPlate(t, wallDelta, hasDelta)
		out, recognized, err := retainedblend.Fillet(t.Context(), axialEnv(), b, built,
			[]*topo.Edge{topRimEdge(t, b)}, 0.5, 0)
		require.NoError(t, err)
		require.True(t, recognized)
		require.NotNil(t, out)
		return out
	}
	return chargedPair{charged: fillet(delta, true), twin: fillet(0, false)}
}

// holeWall is the retained Cylinder group of the given radius, the plate's hole wall.
func holeWall(t *testing.T, fp facetedrecord.Payload, radius float64) facetedrecord.Group {
	t.Helper()
	var found []facetedrecord.Group
	for _, g := range fp.Groups {
		cyl, ok := g.Surface.(surfacegeom.Cylinder)
		if !ok {
			continue
		}
		r, err := cyl.Radius.In(units.Millimeter)
		require.NoError(t, err)
		if math.Abs(r-radius) < 1e-9 {
			found = append(found, g)
		}
	}
	require.Len(t, found, 1, "the hole wall is one retained cylinder group")
	require.True(t, found[0].Retained)
	return found[0]
}

// holeWallFace is the body's face tagged as the hole's Cylinder of the given radius.
func holeWallFace(t *testing.T, b *topo.Body, radius float64) *topo.Face {
	t.Helper()
	var found []*topo.Face
	for _, f := range topobuild.Faces(b) {
		if cyl, ok := f.Surface.(surfacegeom.Cylinder); ok {
			r, err := cyl.Radius.In(units.Millimeter)
			require.NoError(t, err)
			if math.Abs(r-radius) < 1e-9 {
				found = append(found, f)
			}
		}
	}
	require.NotEmpty(t, found, "the hole wall face")
	return found[0]
}

// requireWallStates checks that the charged body's hole wall states delta, unmarked, on its group and every wall face,
// and that the twin's wall states none.
func requireWallStates(t *testing.T, p chargedPair, delta float64) {
	t.Helper()
	cw := holeWall(t, payloadOf(t, p.charged), holeRadius)
	require.True(t, cw.HasAxialDelta)
	require.Equal(t, delta, cw.AxialDelta)
	require.False(t, cw.ViewCharge, "the view's mark never leaves the blend")
	tw := holeWall(t, payloadOf(t, p.twin), holeRadius)
	require.False(t, tw.HasAxialDelta)
	require.Zero(t, tw.AxialDelta)

	cf, tf := holeWallFace(t, p.charged, holeRadius), holeWallFace(t, p.twin, holeRadius)
	require.True(t, cf.RetainedTag)
	require.True(t, cf.HasAxialDelta)
	require.Equal(t, delta, cf.AxialDelta)
	require.False(t, cf.ViewCharge)
	require.False(t, tf.HasAxialDelta)
}

// requireWallNormalCharge checks the charged wall face's NormalBound against the twin's and against the held term of
// a charged wall, HeldNormalTerm at 2(delta_f + delta), where delta_f is the largest bound of the wall's own facet
// vertices. The twin's term is at 2 delta_f only, so a delta dropped anywhere on the chain halves the charged term
// and turns this red.
func requireWallNormalCharge(t *testing.T, p chargedPair, radius float64) {
	t.Helper()
	cf, tf := holeWallFace(t, p.charged, radius), holeWallFace(t, p.twin, radius)
	fp := payloadOf(t, p.charged)
	gi := slices.IndexFunc(fp.Groups, func(g facetedrecord.Group) bool {
		cyl, ok := g.Surface.(surfacegeom.Cylinder)
		if !ok {
			return false
		}
		r, err := cyl.Radius.In(units.Millimeter)
		require.NoError(t, err)
		return math.Abs(r-radius) < 1e-9
	})
	require.GreaterOrEqual(t, gi, 0)
	g := fp.Groups[gi]
	require.Positive(t, g.AxialDelta)
	deltaF := 0.0
	for i, tri := range fp.Tris {
		if fp.Src[i] != gi {
			continue
		}
		for _, v := range tri {
			deltaF = max(deltaF, fp.VertexBound[v])
		}
	}
	want, ok := facetedrecord.HeldNormalTerm(g.Surface, 2*(deltaF+g.AxialDelta), 0)
	require.True(t, ok)
	require.Positive(t, tf.NormalBound)
	require.GreaterOrEqual(t, cf.NormalBound, want*(1-1e-9), "the charged wall carries 4(delta_f + delta)/R")
	require.Greater(t, cf.NormalBound, tf.NormalBound)
}

func TestFilletPublishesChargedCylinderWall(t *testing.T) {
	t.Parallel()
	for name, delta := range map[string]float64{"a rounding-sized": 1e-14, "a stated": 1e-6} {
		t.Run(name+" displacement", func(t *testing.T) {
			t.Parallel()
			p := rimBlend(t, delta)
			requireWallStates(t, p, delta)
			requireWallNormalCharge(t, p, holeRadius)
			require.GreaterOrEqual(t, p.charged.Volume.Bound.Base(), p.twin.Volume.Bound.Base())
		})
	}
}

// cutMesher meshes an operand through the tessellation dispatch, as the package's evaluator tests do.
type cutMesher struct{}

func (cutMesher) TessellateOperand(ctx context.Context, body *topo.Body, chord float64) (*topo.Mesh, bool, error) {
	mesh, err := tessdispatch.Body(ctx, body, chord, tessdispatch.All, tessdispatch.Options{Alloc: topotest.Alloc{}})
	if err != nil {
		return nil, false, err
	}
	if err := topobuild.RequireShellWinding(ctx, mesh, body.Lumps); err != nil {
		return nil, false, err
	}
	mesh.BoundaryOK = true
	return mesh, false, nil
}

// boxCut is a real mesh Cut of target by a 2 x 2 box through the plate's full height, away from the hole.
func boxCut(t *testing.T, target *topo.Body) *topo.Body {
	t.Helper()
	ctx := t.Context()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	pts := []sectionrecord.Point2{{U: 5, V: 5}, {U: 7, V: 5}, {U: 7, V: 7}, {U: 5, V: 7}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	env := topo.Env{Doc: &topo.Document{}, Ref: 31, Alloc: topotest.Alloc{}}
	tool, err := prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}},
		Frame:   frame, Z0: -1, Z1: plateHeight + 1, Xform: r3.Identity(),
	}, freeform.NewFreeformWork())
	require.NoError(t, err)

	ev, err := booleaneval.Evaluate(ctx, meshbool.OpCut, target, tool, cutMesher{}, 100, nil, nil)
	require.NoError(t, err)
	volume := measurement.Measurement{Value: units.CubicMillimeters(ev.Volume),
		Bound: units.CubicMillimeters(ev.VolumeBound), Exactness: measurement.ExactnessOf(ev.VolumeBound)}
	out, err := facetedrecord.BuildWithProof(ctx, topo.Env{Doc: &topo.Document{}, Ref: 32, Alloc: topotest.Alloc{}},
		ev.Payload, ev.Audit, volume, ev.VolumeRat)
	require.NoError(t, err)
	return out
}

func TestMeshCutKeepsChargedCylinderWall(t *testing.T) {
	t.Parallel()
	for name, delta := range map[string]float64{"a rounding-sized": 1e-14, "a stated": 1e-6} {
		t.Run(name+" displacement", func(t *testing.T) {
			t.Parallel()
			blend := rimBlend(t, delta)
			cut := chargedPair{charged: boxCut(t, blend.charged), twin: boxCut(t, blend.twin)}
			requireWallStates(t, cut, delta)
			requireWallNormalCharge(t, cut, holeRadius)
			// The cut removed the 2 x 2 x 5 box from both.
			require.InDelta(t, meshVolume(payloadOf(t, blend.twin))-20, meshVolume(payloadOf(t, cut.twin)), 1e-6)
			require.InDelta(t, meshVolume(payloadOf(t, blend.charged))-20, meshVolume(payloadOf(t, cut.charged)), 1e-6)
		})
	}
}

// bottomRimEdge is the edge of the bottom plane and the hole wall, whatever curve it carries.
func bottomRimEdge(t *testing.T, b *topo.Body) *topo.Edge {
	t.Helper()
	for _, e := range topobuild.Edges(b) {
		p := e.Start.Position
		if math.Abs(p.Z) > 1e-9 || math.Abs(math.Hypot(p.X, p.Y)-holeRadius) > 1e-9 || len(e.Faces) != 2 {
			continue
		}
		planes, cyls := 0, 0
		for _, f := range e.Faces {
			switch f.Surface.(type) {
			case surfacegeom.Plane:
				planes++
			case surfacegeom.Cylinder:
				cyls++
			}
		}
		if planes == 1 && cyls == 1 {
			return e
		}
	}
	require.FailNow(t, "no bottom rim edge")
	return nil
}

func TestChamferOfPublishedChargedCylinderWall(t *testing.T) {
	t.Parallel()
	// 1e-14 is below the 64-ulp slack of the chain's coordinates, so the unmarked wall's rim keeps its circle.
	const delta = 1e-14
	blend := rimBlend(t, delta)
	cut := chargedPair{charged: boxCut(t, blend.charged), twin: boxCut(t, blend.twin)}
	chamfer := func(b *topo.Body) *topo.Body {
		out, recognized, err := retainedblend.Chamfer(t.Context(), axialEnv(), b, payloadOf(t, b),
			[]*topo.Edge{bottomRimEdge(t, b)}, 0.5, 0, nil)
		require.True(t, recognized, "the bottom rim of the published wall is recognized")
		require.NoError(t, err)
		require.NotNil(t, out)
		return out
	}
	twin := chamfer(cut.twin)
	out := chamfer(cut.charged)
	done := chargedPair{charged: out, twin: twin}
	requireWallStates(t, done, delta)

	require.GreaterOrEqual(t, out.Area.Bound.Base(), twin.Area.Bound.Base()+0.99*22.44*0.5*delta)
	// The volume bound carries the wall's displacement through the rim's blend: strictly more than the twin's.
	require.Greater(t, out.Volume.Bound.Base(), twin.Volume.Bound.Base())
	// The exact volume is the same to the mesh's own rounding.
	require.InDelta(t, meshVolume(payloadOf(t, twin)), meshVolume(payloadOf(t, out)), 1e-9)
}

// requireChargedWallKept checks the stage's charged wall states want and its face's normal bound keeps the charge
// beyond the twin's.
func requireChargedWallKept(t *testing.T, p chargedPair, want, radius float64) {
	t.Helper()
	cw := holeWall(t, payloadOf(t, p.charged), radius)
	require.True(t, cw.HasAxialDelta)
	require.GreaterOrEqual(t, cw.AxialDelta, want)
	require.LessOrEqual(t, cw.AxialDelta, want*(1+1e-15))
	tw := holeWall(t, payloadOf(t, p.twin), radius)
	require.False(t, tw.HasAxialDelta)
	cf, tf := holeWallFace(t, p.charged, radius), holeWallFace(t, p.twin, radius)
	require.True(t, cf.HasAxialDelta)
	require.False(t, tf.HasAxialDelta)
	require.Greater(t, cf.NormalBound, tf.NormalBound, "the charged wall's normal bound keeps its term")
	requireWallNormalCharge(t, p, radius)
}

func TestPlacementKeepsChargedCylinderWall(t *testing.T) {
	t.Parallel()
	const delta = 1e-6
	blend := rimBlend(t, delta)
	move, err := r3.Translation(r3.NewVec(3, -4, 2.5))
	require.NoError(t, err)
	place := func(b *topo.Body) *topo.Body {
		fp := payloadOf(t, b)
		fp.Xform = r3.Identity() // the fixture is hand built; a Copy places a payload that carries its pose
		out, err := fp.Placed(t.Context(), axialEnv(), move)
		require.NoError(t, err)
		return out
	}
	placed := chargedPair{charged: place(blend.charged), twin: place(blend.twin)}
	requireChargedWallKept(t, placed, delta, holeRadius)
	require.Equal(t, delta, holeWall(t, payloadOf(t, placed.charged), holeRadius).AxialDelta)
}

// An exact rotation (the path of an exactly denoted Copy) keeps the wall's displacement too: the true wall moves
// rigidly with its tag.
func TestExactPlacementKeepsChargedCylinderWall(t *testing.T) {
	t.Parallel()
	const delta = 1e-6
	blend := rimBlend(t, delta)
	q, composed, err := exactrotation.New(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(90))
	require.NoError(t, err)
	place := func(b *topo.Body) *topo.Body {
		fp := payloadOf(t, b)
		fp.Xform = r3.Identity()
		out, err := fp.PlacedWith(t.Context(), axialEnv(), composed, q)
		require.NoError(t, err)
		return out
	}
	placed := chargedPair{charged: place(blend.charged), twin: place(blend.twin)}
	requireChargedWallKept(t, placed, delta, holeRadius)
}

func TestScaleMultipliesChargedCylinderWall(t *testing.T) {
	t.Parallel()
	const delta, factor = 1e-6, 2.0
	blend := rimBlend(t, delta)
	scale := func(b *topo.Body) *topo.Body {
		src := payloadOf(t, b)
		src.Xform = r3.Identity() // the fixture is hand built; a scaled body carries its pose
		fp, err := facetedrecord.ScaleImage(t.Context(), b, src, r3.Vec{}, factor)
		require.NoError(t, err)
		out, err := facetedrecord.Build(t.Context(), axialEnv(), fp)
		require.NoError(t, err)
		return out
	}
	scaled := chargedPair{charged: scale(blend.charged), twin: scale(blend.twin)}
	requireChargedWallKept(t, scaled, factor*delta, factor*holeRadius)
}

// A wall displacement above the chain's slack leaves the published wall's rim without a retained circle, so its
// chamfer is not recognized, where the uncharged twin's is.
func TestChamferDeclinesPublishedWallBeyondSlack(t *testing.T) {
	t.Parallel()
	blend := rimBlend(t, 1e-6)
	chamfer := func(b *topo.Body) bool {
		_, recognized, err := retainedblend.Chamfer(t.Context(), axialEnv(), b, payloadOf(t, b),
			[]*topo.Edge{bottomRimEdge(t, b)}, 0.5, 0, nil)
		return recognized && err == nil
	}
	require.True(t, chamfer(blend.twin))
	require.False(t, chamfer(blend.charged))
}
