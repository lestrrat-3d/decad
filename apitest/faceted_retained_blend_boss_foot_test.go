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

// The boss-foot rows of agent-docs/retained-edge-blend-design.md §9: a boss's
// side edge is convex, and at its foot it ends on the plate's top face, which
// wraps the boss's footprint and so is reflex at the corner. The plate's top
// face takes the blend's end cap there. A through hole placed by a Transform
// keeps every body on the mesh path.

// bossFoot describes a plate 80×60×10 with a 15 mm boss over the rectangle
// rect = (x0, y0, x1, y1) standing on it, a through hole of radius hole[2] at
// (hole[0], hole[1]), and, when roofed, a second 10 mm plate lying on the
// boss's top. A nonzero lean cuts away the boss's side beyond the vertical
// plane through (x0 + 2, y0) that advances lean in x per unit of y, so the
// boss's corner there is π/2 − atan(lean). The plane misses the rectangle's
// corners, which leaves no sliver of the cut side. The cut is sketched, not
// rotated into place: a rotated tool's tags state a departure, which a
// retained blend refuses (agent-docs/faceted-source-retention-design.md §3.5).
type bossFoot struct {
	rect   [4]float64
	lean   float64
	hole   [3]float64
	roofed bool
}

// build makes the body.
func (bf bossFoot) build(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	x0, y0, x1, y1 := bf.rect[0], bf.rect[1], bf.rect[2], bf.rect[3]
	body := rectBox(t, sc.w, sc.doc, 0, 0, 80, 60, 10)
	// The boss stands on the plate's top, so the Union shifts it clear
	// (agent-docs/gap-tolerance-design.md §5.2). A leaning boss instead sinks
	// 1 mm into the plate: the shift re-normalises the slanted plane's frame
	// (surfaceplace.TransformSurface), which states a departure a retained
	// blend refuses.
	boss := placed(t, rectBox(t, sc.w, sc.doc, x0, y0, x1, y1, 15), 10)
	if bf.lean != 0 {
		boss = placed(t, rectBox(t, sc.w, sc.doc, x0, y0, x1, y1, 16), 9)
		px, py := x0+2, y0
		s, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		corners := [][2]float64{{px - 60*bf.lean, py - 60}, {px + 60*bf.lean, py + 60}, {px - 60*bf.lean, py + 60}}
		pts := make([]*sketch.Point, len(corners))
		for i, c := range corners {
			pts[i] = s.CreatePoint(c[0], c[1])
			s.Fix(pts[i])
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		tool, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(25), Dir: decad.Along})
		require.NoError(t, err)
		boss, err = decad.Cut(t.Context(), boss, placed(t, tool, 4))
		require.NoError(t, err)
	}
	body, err := decad.Union(t.Context(), body, boss)
	require.NoError(t, err)
	if bf.roofed {
		roof := rectBox(t, sc.w, sc.doc, 0, 0, 80, 60, 10)
		body, err = decad.Union(t.Context(), body, placed(t, roof, 25))
		require.NoError(t, err)
	}
	body, err = decad.Cut(t.Context(), body, zCylinderAt(t, sc, bf.hole[0], bf.hole[1], bf.hole[2], -5, 40))
	require.NoError(t, err)
	return body
}

// volume is the exact volume of the body build makes.
func (bf bossFoot) volume() float64 {
	w, h := bf.rect[2]-bf.rect[0], bf.rect[3]-bf.rect[1]
	slabs := 10.0
	if bf.roofed {
		slabs = 20
	}
	area := w * h
	if bf.lean != 0 {
		area -= 2*h + h*h*bf.lean/2
	}
	return 80*60*slabs + 15*area - math.Pi*bf.hole[2]*bf.hole[2]*slabs
}

// corners are the boss's four corners, in the order the rectangle lists
// them: (x0, y0), (x1, y0), (x1, y1), (x0, y1).
func (bf bossFoot) corners() [][2]float64 {
	return [][2]float64{{bf.rect[0], bf.rect[1]}, {bf.rect[2], bf.rect[1]}, {bf.rect[2], bf.rect[3]},
		{bf.rect[0], bf.rect[3]}}
}

// boxBoss is the 20×20 boss over (30..50, 20..40) with the r = 3 hole at
// (10, 10).
var boxBoss = bossFoot{rect: [4]float64{30, 20, 50, 40}, hole: [3]float64{10, 10, 3}}

// bossSides selects b's vertical edges with a top end within a micrometre of
// one of the corners at height 25. The mesh Boolean holds the ends as floats
// near the exact corners, so each is matched at its held position.
func bossSides(t *testing.T, b *decad.Body, corners [][2]float64) *decad.EdgeQuery {
	t.Helper()
	var q *decad.EdgeQuery
	for _, c := range corners {
		p := r3.NewVec(c[0], c[1], 25)
		var at *r3.Vec
		for _, v := range b.Vertices() {
			if pos := v.Position().Value; pos.Sub(p).Len() < 1e-3 {
				at = &pos
				break
			}
		}
		require.NotNil(t, at, "no vertex near %v", p)
		preds := []decad.EdgePredicate{decad.EndpointAt(*at), decad.ParallelTo(zAxis)}
		if q == nil {
			q = decad.Edges(preds...)
			continue
		}
		q = q.Or(preds...)
	}
	return q.Exactly(len(corners))
}

// requireBossVolume checks the built body's volume against the exact one.
func requireBossVolume(t *testing.T, b *decad.Body, want float64) decad.Measurement {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	t.Logf("volume %.6f ± %.3g, exact %.6f", v.Value.Mag(), v.Bound.Mag(), want)
	require.InDelta(t, want, v.Value.Mag(), v.Bound.Mag())
	return v
}

// requireCapsTaken checks the surface area after a blend of side edges whose
// ends take caps. The caps' total area is caps; charging them as corners the
// end faces give up would put the exact area change 2·caps off the held one.
// The area slack carries that gap, so the bound's growth stays below it only
// when the caps are charged the right way.
func requireCapsTaken(t *testing.T, before, after *decad.Body, want, caps float64) {
	t.Helper()
	beforeArea, err := before.Area()
	require.NoError(t, err)
	afterArea, err := after.Area()
	require.NoError(t, err)
	requireAreaChange(t, beforeArea, afterArea, want)
	require.Less(t, afterArea.Bound.Mag()-beforeArea.Bound.Mag(), 2*caps)
}

func TestFacetedRetainedBlendBossFoot(t *testing.T) {
	t.Parallel()

	t.Run("fillet of a box boss's sides", func(t *testing.T) {
		t.Parallel()
		// Each side edge loses r²(1 − π/4) of section over its 15 mm. The
		// boss's top gives up the corner and the plate's top takes the same
		// corner as the cap, so the faces' corner areas cancel: the area
		// changes by the quarter cylinder less the two 3 mm strips.
		const r = 3.0
		body := boxBoss.build(t)
		before := requireBossVolume(t, body, boxBoss.volume())
		filleted, err := decad.Fillet(t.Context(), body, bossSides(t, body, boxBoss.corners()), units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 4*15*r*r*(1-math.Pi/4))
		requireCapsTaken(t, body, filleted, -4*15*r*(2-math.Pi/2), 4*r*r*(1-math.Pi/4))
		cylinders := 0
		for _, f := range filleted.Faces() {
			// The hole's wall shares the radius; its axis is at x = 10.
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-r) < 1e-12 && c.Origin.X > 20 {
				cylinders++
				require.InDelta(t, 1, math.Abs(c.Axis.Z), 1e-12)
			}
		}
		require.Equal(t, 4, cylinders)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("chamfer of a box boss's sides", func(t *testing.T) {
		t.Parallel()
		const d = 2.0
		body := boxBoss.build(t)
		before := requireBossVolume(t, body, boxBoss.volume())
		chamfered, err := decad.Chamfer(t.Context(), body, bossSides(t, body, boxBoss.corners()), units.Millimeters(d))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 4*15*d*d/2)
		requireCapsTaken(t, body, chamfered, 4*15*(d*math.Sqrt2-2*d), 4*d*d/2)
	})

	t.Run("fillet at a corner that is not square", func(t *testing.T) {
		t.Parallel()
		// The slanted side meets the side along y = 20 at 45° at (32, 20), so
		// the faces' normals turn by φ = 3π/4 and the section loses
		// r²(tan(φ/2) − φ/2). The setback is r·tan(φ/2) on each side, so the
		// area gains the arc r·φ and loses two strips over the 15 mm; the
		// boss's top gives up the corner region the plate's top takes. The
		// boss reaches x = 60 so the slanted side leaves through its top side.
		const r = 2.0
		slanted := boxBoss
		slanted.rect[2] = 60
		slanted.lean = 1
		body := slanted.build(t)
		before := requireBossVolume(t, body, slanted.volume())
		filleted, err := decad.Fillet(t.Context(), body, bossSides(t, body, [][2]float64{{32, 20}}), units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		const phi = 3 * math.Pi / 4
		corner := r * r * (math.Tan(phi/2) - phi/2)
		requireVolumeChange(t, before, after, 15*corner)
		requireCapsTaken(t, body, filleted, 15*r*(phi-2*math.Tan(phi/2)), corner)
	})

	t.Run("fillet of a pillar between two plates", func(t *testing.T) {
		t.Parallel()
		// Both ends of the side edge lie on plate faces reflex at the corner,
		// and both take a cap.
		const r = 3.0
		pillar := boxBoss
		pillar.roofed = true
		body := pillar.build(t)
		before := requireBossVolume(t, body, pillar.volume())
		filleted, err := decad.Fillet(t.Context(), body, bossSides(t, body, pillar.corners()[:1]), units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 15*r*r*(1-math.Pi/4))
		caps := 2 * r * r * (1 - math.Pi/4)
		requireCapsTaken(t, body, filleted, 15*r*(math.Pi/2-2)+caps, caps)
	})

	t.Run("a cavity crossing the cap", func(t *testing.T) {
		t.Parallel()
		// The cavity straddles the plate's top under the boss's corner at
		// (30, 20). Its corner nearest the fillet's axis at (33, 23) lies 3.18
		// mm from it, outside the 3 mm blend, so it meets no strip facet; its
		// upper half lies in the corner the fillet removes, and it crosses the
		// cap the plate's top takes.
		body := withCavity(t, boxBoss.build(t), 30.25, 20.25, 30.75, 20.75, 9, 11)
		_, err := decad.Fillet(t.Context(), body, bossSides(t, body, boxBoss.corners()[:1]), units.Millimeters(3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the cap an end face takes meets another facet of the body at")
	})

	t.Run("refusals", func(t *testing.T) {
		t.Parallel()
		// A hole whose rim passes 1.83 mm from the corner lies inside the
		// 3 mm the fillet's rows reach on the plate's top.
		near := boxBoss
		near.hole = [3]float64{28, 18, 1}
		body := near.build(t)
		_, err := decad.Fillet(t.Context(), body, bossSides(t, body, near.corners()[:1]), units.Millimeters(3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "another part of an end face comes within the blend's reach of the corner")

		// An L section extruded 20 mm along y: a 10 mm wall over x = 10..20
		// rises from z = 10 to 30 on a plate 40 mm long. The wall's convex
		// edge along z at (20, 0) ends at the step z = 10, where the front
		// face y = 0 carries on below the corner: that face is reflex there.
		sc := newRetentionScene()
		s, err := sc.w.CreateSketch(sc.w.XZ())
		require.NoError(t, err)
		outline := [][2]float64{{0, 0}, {40, 0}, {40, 10}, {20, 10}, {20, 30}, {10, 30}, {10, 10}, {0, 10}}
		pts := make([]*sketch.Point, len(outline))
		for i, c := range outline {
			pts[i] = s.CreatePoint(c[0], c[1])
			s.Fix(pts[i])
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		prism, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(10)})
		require.NoError(t, err)
		shift, err := r3.Translation(r3.Vec{Y: 10})
		require.NoError(t, err)
		prism, err = decad.Transform(t.Context(), prism, shift)
		require.NoError(t, err)
		stepped, err := decad.Cut(t.Context(), prism, zCylinderAt(t, sc, 30, 10, 3, -30, 30))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), stepped, straightEdge(t, stepped, r3.NewVec(20, 0, 30), zAxis),
			units.Millimeters(2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a convex edge ends where a face is reflex")
	})
}
