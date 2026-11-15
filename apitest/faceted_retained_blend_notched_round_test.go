package apitest_test

import (
	"fmt"
	"maps"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-modify-reach-design.md §5.5 step 5 for a
// concave open tangent chain whose walls continue past its end planes: a box
// ear is unioned under a block whose bottom edges are rounded, and the ear's
// front face crosses the round between its two tangent lines. The ear's root
// on its side face (or back face) runs as a straight edge on the floor, through
// a tangent junction, and as an arc on the round. The straight end lies on the
// ear's back face, past which the floor continues; the arc ends on its front
// face, past which the round continues. Both end faces are convex at the corner
// and take the blend's end cap.

const (
	notchLongR  = 3.0 // radius of the bottom long edges' rounds (along x)
	notchShortR = 2.0 // radius of the bottom short edges' rounds (along y)
)

// notchedBlock is a 40 x 20 x 20 block (corner at the origin) whose bottom
// long edges are rounded at notchLongR and short edges at notchShortR, with a
// pocket cut in its top so that it is a mesh Boolean result; ears is joined
// under it. ear boxes are given as x0, y0, x1, y1, z0, z1.
func notchedBlock(t *testing.T, w *sketch.World, doc *decad.Document, ears ...[6]float64) *decad.Body {
	t.Helper()
	x := decad.ParallelTo(r3.NewVec(1, 0, 0))
	sec, err := w.CreateSketch(w.YZ())
	require.NoError(t, err)
	r := sec.CreateRectangle(0, 0, 20, 20)
	sec.Fix(r.A)
	_, err = sec.Solve(t.Context())
	require.NoError(t, err)
	block, err := decad.Extrude(t.Context(), doc, sec, sec.Profiles()[0],
		decad.Distance{D: units.Millimeters(40), Dir: decad.Along})
	require.NoError(t, err)
	block, err = decad.Fillet(t.Context(), block, decad.Edges(x, decad.EndpointAt(r3.NewVec(0, 0, 0))).
		Or(x, decad.EndpointAt(r3.NewVec(0, 20, 0))).Exactly(2), units.Millimeters(notchLongR))
	require.NoError(t, err)
	loops := decad.Edges(decad.OuterLoopOf(decad.CapStart(block))).Or(decad.OuterLoopOf(decad.CapEnd(block)))
	block, err = decad.Fillet(t.Context(), block, loops, units.Millimeters(notchShortR))
	require.NoError(t, err)
	block, err = decad.Cut(t.Context(), block, placed(t, rectBox(t, w, doc, 12, 6, 28, 14, 10), 12))
	require.NoError(t, err)
	for _, e := range ears {
		ear := placed(t, rectBox(t, w, doc, e[0], e[1], e[2], e[3], e[5]-e[4]), e[4])
		block, err = decad.Union(t.Context(), block, ear)
		require.NoError(t, err)
	}
	return block
}

// notchedRoot is one fan-shaped root of an ear unioned under notchedBlock: the
// ear's side or back face P carries a concave root that runs as a straight
// edge on the floor, through a tangent junction, and into an arc on a round.
// The straight end lies on the ear's back face G0, past which the floor
// continues; the arc ends on its front face G1, past which the round
// continues.
type notchedRoot struct {
	name string
	ear  [6]float64
	at   r3.Vec // the root's end on G0
	dir  r3.Vec // the root's straight direction
	// lineLen is the straight part's length, radius the round's radius and
	// offset the distance of G1's plane from the round's axis.
	lineLen, radius, offset float64
	// g0 and g1 name the end faces' planes: the axis of their normal and their
	// offset along it.
	g0, g1 [2]float64
	// round says whether p lies on the round, and zone whether it lies where
	// the blend rewrites the held mesh.
	round, zone func(p r3.Vec) bool
	// keptMin is how many vertices of the round lie outside the zone.
	keptMin int
}

// onLateralRound says whether p lies on the block's r 3 round along x, and
// inSideRootZone whether it lies within the x = 18 root's reach, past its base.
func onLateralRound(p r3.Vec) bool {
	return p.X > 4 && p.X < 36 && p.Y < 3.2 && p.Z < 3.2 && math.Abs(math.Hypot(p.Y-3, p.Z-3)-3) < 0.05
}

func inSideRootZone(p r3.Vec) bool { return p.X > 17.5 && p.X < 19.5 && p.Y > 1.4 }

var notchedRoots = []notchedRoot{
	{
		name: "N1 side root", ear: [6]float64{10, 1.5, 18, 13.5, -6, 1},
		at: r3.NewVec(18, 13.5, 0), dir: r3.NewVec(0, 1, 0),
		lineLen: 10.5, radius: notchLongR, offset: 1.5, g0: [2]float64{1, 13.5}, g1: [2]float64{1, 1.5}, keptMin: 20,
		round: onLateralRound, zone: inSideRootZone,
	},
	{
		name: "N2 back root", ear: [6]float64{0.8, 5, 12.8, 13, -6, 1},
		at: r3.NewVec(12.8, 13, 0), dir: r3.NewVec(1, 0, 0),
		lineLen: 10.8, radius: notchShortR, offset: 1.2, g0: [2]float64{0, 12.8}, g1: [2]float64{0, 0.8}, keptMin: 20,
		round: func(p r3.Vec) bool {
			return p.Y > 3.5 && p.Y < 16.5 && p.X < 2.2 && p.Z < 2.2 && math.Abs(math.Hypot(p.X-2, p.Z-2)-2) < 0.05
		},
		zone: func(p r3.Vec) bool { return p.Y > 12.5 && p.Y < 14.5 && p.X > 0.7 },
	},
	{
		name: "N3 ear on the round alone", ear: [6]float64{10, 1.5, 18, 4.2, -6, 1},
		at: r3.NewVec(18, 4.2, 0), dir: r3.NewVec(0, 1, 0),
		lineLen: 1.2, radius: notchLongR, offset: 1.5, g0: [2]float64{1, 4.2}, g1: [2]float64{1, 1.5}, keptMin: 20,
		round: onLateralRound, zone: inSideRootZone,
	},
}

// notchedBody builds the fixture of c in a document that takes the unions.
func notchedBody(t *testing.T, ears ...[6]float64) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	return notchedBlock(t, w, doc, ears...)
}

func (c notchedRoot) seed() decad.EdgeSelector {
	return decad.Edges(decad.Concave(), decad.ParallelTo(c.dir), decad.EndpointAt(c.at))
}

// requireRoundKept checks that every vertex of the round outside the blend's
// zone is a vertex of the rewritten mesh and the other way round: the blend
// rewrites only the facets it reaches. It reports counts and the first few
// mismatches, never the meshes.
func requireRoundKept(t *testing.T, c notchedRoot, held, out *decad.Mesh) {
	t.Helper()
	keys := func(m *decad.Mesh) map[string]r3.Vec {
		set := map[string]r3.Vec{}
		for _, p := range m.Vertices() {
			if c.round(p) && !c.zone(p) {
				set[fmt.Sprintf("%.9f,%.9f,%.9f", p.X, p.Y, p.Z)] = p
			}
		}
		return set
	}
	before, after := keys(held), keys(out)
	t.Logf("%s: %d round vertices outside the zone", c.name, len(before))
	require.GreaterOrEqual(t, len(before), c.keptMin, "the round holds vertices outside the blend's zone")
	var missing, added []string
	for k := range before {
		if _, ok := after[k]; !ok && len(missing) < 3 {
			missing = append(missing, k)
		}
	}
	for k := range after {
		if _, ok := before[k]; !ok && len(added) < 3 {
			added = append(added, k)
		}
	}
	require.Empty(t, missing, "round vertices outside the zone the blend dropped (of %d)", len(before))
	require.Empty(t, added, "round vertices outside the zone the blend made (of %d)", len(after))
}

func TestFacetedRetainedBlendNotchedRound(t *testing.T) {
	t.Parallel()

	for _, c := range notchedRoots {
		for _, fillet := range []bool{true, false} {
			if !fillet && c.name != "N1 side root" {
				continue
			}
			name := c.name + ", fillet"
			if !fillet {
				name = c.name + ", chamfer"
			}
			t.Run(name, func(t *testing.T) {
				t.Parallel()
				const size = 1.0
				body := notchedBody(t, c.ear)
				volBefore, err := body.Volume()
				require.NoError(t, err)
				g0Before := earFaceArea(t, body, int(c.g0[0]), c.g0[1])
				g1Before := earFaceArea(t, body, int(c.g1[0]), c.g1[1])
				held, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
				require.NoError(t, err)

				var out *decad.Body
				if fillet {
					out, err = decad.Fillet(t.Context(), body, c.seed(), units.Millimeters(size), decad.WithTangentChain())
				} else {
					out, err = decad.Chamfer(t.Context(), body, c.seed(), units.Millimeters(size), decad.WithTangentChain())
				}
				require.NoError(t, err)

				// The blend fills the root's corner along the line, A_f·L, and along
				// the arc where a row at radius rho sweeps asin(offset/rho) of it
				// from the junction's meridian to G1: ∫∫ rho·Δφ.
				af := size * size / 2
				if fillet {
					af = size * size * (1 - math.Pi/4)
				}
				added := af*c.lineLen + earRegion(c.radius, 1, size, fillet, func(rho float64) float64 {
					return rho * math.Asin(c.offset/rho)
				})
				volAfter, err := out.Volume()
				require.NoError(t, err)
				requireVolumeChange(t, volBefore, volAfter, -added)

				// G0 takes the section, G1 the section of the torus (or cone) by its
				// plane.
				requireAreaChange(t, g0Before, earFaceArea(t, out, int(c.g0[0]), c.g0[1]), af)
				capArea := earRegion(c.radius, 1, size, fillet, func(rho float64) float64 {
					return rho / math.Sqrt(rho*rho-c.offset*c.offset)
				})
				requireAreaChange(t, g1Before, earFaceArea(t, out, int(c.g1[0]), c.g1[1]), capArea)

				if fillet {
					cylinders, tori, majors := blendKinds(out, size)
					require.Equal(t, 1, cylinders)
					require.Equal(t, 1, tori)
					require.InDelta(t, c.radius+size, majors[0], 1e-9)
				}

				// The bound grows by the strip's own terms past the held mesh's.
				mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.LessOrEqual(t, revolveMeshBound(t, mesh), revolveMeshBound(t, held)+2*math.Ldexp(size, -10))
				require.True(t, mesh.BoundaryVerified())
				require.True(t, mesh.VolumeVerified())
				rep, err := out.Document().Verify(t.Context())
				require.NoError(t, err)
				br, err := rep.ForBody(out)
				require.NoError(t, err)
				require.Equal(t, decad.Sound, br.Status)

				// The round behind the base and past the end face is not touched.
				requireRoundKept(t, c, held, mesh)
				requireNoCrossingTriangles(t, out)
			})
		}
	}
}

func TestFacetedRetainedBlendNotchedRoundRefusals(t *testing.T) {
	t.Parallel()

	n1 := notchedRoots[0]
	// refused fillets the root of n1's fixture (c names the seed) and checks the
	// refusal's text.
	refused := func(t *testing.T, body *decad.Body, c notchedRoot, text string) {
		t.Helper()
		_, err := decad.Fillet(t.Context(), body, c.seed(), units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, text)
	}

	t.Run("a hole in the round inside the band is refused", func(t *testing.T) {
		t.Parallel()
		w, doc := sketch.NewWorld(), decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
		body, err := decad.Cut(t.Context(), notchedBlock(t, w, doc, n1.ear),
			zCylinderAt(t, &retentionScene{w: w, doc: doc}, 18.5, 2.2, 0.3, -1, 5))
		require.NoError(t, err)
		refused(t, body, n1, "another part of the wall lies within the blend's reach of the edge")
	})

	t.Run("a back root whose band reaches the corner patch of two rounds is refused", func(t *testing.T) {
		t.Parallel()
		// The ear's back face lies 0.5 from where the end round meets the
		// corner patch, inside the fillet's reach.
		c := notchedRoots[1]
		c.ear = [6]float64{0.8, 5, 12.8, 16.5, -6, 1}
		c.at = r3.NewVec(12.8, 16.5, 0)
		refused(t, notchedBody(t, c.ear), c, "another part of the wall lies within the blend's reach of the edge")
	})

	t.Run("a solid across the cap an end face takes is refused", func(t *testing.T) {
		t.Parallel()
		w, doc := sketch.NewWorld(), decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
		// A 0.15 box clear of the round, across the plane y = 1.5 where the ear's
		// front face takes the cap, inside the fillet's corner and clear of its
		// strip.
		body, err := decad.Union(t.Context(), notchedBlock(t, w, doc, n1.ear),
			placed(t, rectBox(t, w, doc, 18.05, 1.4, 18.2, 1.6, 0.2), 0.1))
		require.NoError(t, err)
		refused(t, body, n1, "the cap an end face takes meets another facet of the body")
	})
}

// Several ears whose back faces lie in one plane G (the back faces of two
// boxes unioned under the floor, y = 13.5) take one blend after another: the
// earlier blend leaves a run of cut vertices along G, and the later band's
// planes put them outside it (agent-docs/retained-edge-blend-design.md §4.3).
// Each side root is notchedRoots[0]'s: a straight edge on the floor and an arc
// on the round, so each root fills what N1's does.

// earBoxA and earBoxB are two ears sharing the back plane y = 13.5. Their front
// faces lie in different planes, y = 1.5 and y = 2: a front plane an unrelated
// ear's corner lies on is a vertex on the blend's own end plane, which the
// band reads as undecided.
var (
	earBoxA = [6]float64{4, 1.5, 12, 13.5, -6, 1}
	earBoxB = [6]float64{22, 2, 30, 13.5, -6, 1}
)

// sharedBackSeed selects the side roots at each x in one query.
func sharedBackSeed(xs ...float64) *decad.EdgeQuery {
	var q *decad.EdgeQuery
	for _, x := range xs {
		preds := []decad.EdgePredicate{decad.Concave(), decad.ParallelTo(r3.NewVec(0, 1, 0)),
			decad.EndpointAt(r3.NewVec(x, 13.5, 0))}
		if q == nil {
			q = decad.Edges(preds...)
			continue
		}
		q.Or(preds...)
	}
	return q
}

// sharedRootFill is the volume a fillet of the given size fills at the side
// root at x: A_f·L along the root's line and the arc's region of the round,
// where a row at radius rho sweeps asin(offset/rho) up to the ear's front face.
// Ear A's front face is 1.5 from the round's axis and ear B's 1.
func sharedRootFill(size, x float64) float64 {
	offset := 1.5
	if x > 20 {
		offset = 1
	}
	return rootFill(size, offset)
}

// rootFill is the fill of a fillet of the given size at a side root whose ear's
// front face lies offset from the round's axis.
func rootFill(size, offset float64) float64 {
	n1 := notchedRoots[0]
	af := size * size * (1 - math.Pi/4)
	return af*n1.lineLen + earRegion(n1.radius, 1, size, true, func(rho float64) float64 {
		return rho * math.Asin(offset/rho)
	})
}

// filletSharedRoots fillets each group of roots in one call, group after group,
// and checks the result: a sound solid whose volume lost each root's fill and
// whose held bound grew by the strips' own terms alone.
func filletSharedRoots(t *testing.T, ears [][6]float64, groups ...[]float64) {
	t.Helper()
	filletRoots(t, ears, sharedRootFill, groups...)
}

// filletRoots is filletSharedRoots with the fill of each root given by fill.
func filletRoots(t *testing.T, ears [][6]float64, fill func(size, x float64) float64, groups ...[]float64) {
	t.Helper()
	filletChargedRoots(t, ears, fill, 0, groups...)
}

// filletChargedRoots is filletRoots for roots whose band reads a vertex within its
// bound of an end plane by the sign of its offset (retained-edge-blend §4.3): the
// offset is charged into that vertex's bound, so the published bound grows past
// the strips' own terms by at most charge. A charge above zero says the charge
// shows: the bound exceeds the strips' own terms.
func filletChargedRoots(t *testing.T, ears [][6]float64, fill func(size, x float64) float64, charge float64,
	groups ...[]float64) {
	t.Helper()
	const size = 1.0
	body := notchedBody(t, ears...)
	volBefore, err := body.Volume()
	require.NoError(t, err)
	held, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)

	roots := 0
	filled := 0.0
	var heldRun map[string]struct{}
	for k, group := range groups {
		seed := sharedBackSeed(group...)
		body, err = decad.Fillet(t.Context(), body, seed, units.Millimeters(size), decad.WithTangentChain())
		require.NoError(t, err, "group %d", k)
		roots += len(group)
		for _, x := range group {
			filled += fill(size, x)
		}
		if k == 0 {
			heldRun = backPlaneVertices(t, body)
		}
	}

	n1 := notchedRoots[0]
	volAfter, err := body.Volume()
	require.NoError(t, err)
	requireVolumeChange(t, volBefore, volAfter, -filled)

	cylinders, tori, majors := blendKinds(body, size)
	require.Equal(t, roots, cylinders)
	require.Equal(t, roots, tori)
	for _, m := range majors {
		require.InDelta(t, n1.radius+size, m, 1e-9)
	}

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	own := revolveMeshBound(t, held) + float64(roots)*2*math.Ldexp(size, -10)
	require.LessOrEqual(t, revolveMeshBound(t, mesh), own+charge)
	if charge > 0 {
		require.Greater(t, revolveMeshBound(t, mesh), own, "the charge shows in the bound")
	}
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	rep, err := body.Document().Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, br.Status)
	requireNoCrossingTriangles(t, body)

	// The first group's cut vertices along the back plane are still vertices of
	// the result: a later blend clipped none of their facets.
	if heldRun != nil {
		kept := backPlaneVertices(t, body)
		var lost []string
		for k := range heldRun {
			if _, ok := kept[k]; !ok && len(lost) < 3 {
				lost = append(lost, k)
			}
		}
		require.Empty(t, lost, "vertices on the back plane the later blends dropped")
	}
}

// backPlaneVertices keys the vertices of the body's mesh that lie in the shared
// back plane y = 13.5 on the floor z = 0.
func backPlaneVertices(t *testing.T, b *decad.Body) map[string]struct{} {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	set := map[string]struct{}{}
	for _, p := range mesh.Vertices() {
		if math.Abs(p.Y-13.5) < 1e-9 && math.Abs(p.Z) < 1e-9 {
			set[fmt.Sprintf("%.9f,%.9f,%.9f", p.X, p.Y, p.Z)] = struct{}{}
		}
	}
	return set
}

func TestFacetedRetainedBlendSharedBackPlane(t *testing.T) {
	t.Parallel()

	both := [][6]float64{earBoxA, earBoxB}
	t.Run("one ear's two side roots in two calls", func(t *testing.T) {
		t.Parallel()
		filletSharedRoots(t, both, []float64{4}, []float64{12})
	})
	t.Run("one ear's second root before its first", func(t *testing.T) {
		t.Parallel()
		filletSharedRoots(t, both, []float64{12}, []float64{4})
	})
	t.Run("both ears' four side roots in one call", func(t *testing.T) {
		t.Parallel()
		filletSharedRoots(t, both, []float64{4, 12, 22, 30})
	})
	t.Run("ear B after ear A", func(t *testing.T) {
		t.Parallel()
		filletSharedRoots(t, both, []float64{4, 12}, []float64{22, 30})
	})
	t.Run("every root in its own call", func(t *testing.T) {
		t.Parallel()
		filletSharedRoots(t, both, []float64{4}, []float64{12}, []float64{22}, []float64{30})
	})
	t.Run("ears 1.5 apart on one plane", func(t *testing.T) {
		t.Parallel()
		// The facing sides lie 1.5 apart. The mesh joins the blend's corner on the
		// back plane to the other ear's corner by a facet edge along the plane, and
		// the band's generator cuts that edge: the cut lies on the plane because
		// both ends of the edge do, by the tags of their faces. The band ends 0.5
		// short of the other ear.
		filletSharedRoots(t, [][6]float64{earBoxA, {13.5, 2, 21.5, 13.5, -6, 1}}, []float64{12})
	})
	t.Run("ears 1.5 apart, the second's back face one float step off the plane", func(t *testing.T) {
		t.Parallel()
		// The second ear's back face lies one float step off y = 13.5: its tag is not
		// the blend's end plane, so its corner on the plane's edge is not on the plane
		// by provenance. The band reads the cut of that edge by the sign of its offset
		// and charges the offset, a float step, instead of reading it as exact.
		for _, y := range []float64{math.Nextafter(13.5, 14), math.Nextafter(13.5, 13)} {
			filletSharedRoots(t, [][6]float64{earBoxA, {13.5, 2, 21.5, y, -6, 1}}, []float64{12})
		}
	})
}

func TestFacetedRetainedBlendSharedBackPlaneRefusals(t *testing.T) {
	t.Parallel()

	t.Run("the same root twice", func(t *testing.T) {
		t.Parallel()
		body := notchedBody(t, earBoxA, earBoxB)
		body, err := decad.Fillet(t.Context(), body, sharedBackSeed(12), units.Millimeters(1),
			decad.WithTangentChain())
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), body, sharedBackSeed(12), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrNoMatch)
		require.ErrorContains(t, err, "matched 0 edges")
	})
}

// Ears whose front faces and back faces both share planes: a facet edge of the
// round runs along an end plane from one ear's corner to the next ear's, and the
// band's setback plane cuts that edge. Each ear's corners lie on the end plane
// by their own faces' tags, which equal the blend's end plane exactly, and the
// cut inherits the plane from the edge's two ends
// (agent-docs/retained-edge-blend-design.md §4.3).
var (
	// earBoxFlush is earBoxA's twin, whose front face lies in the plane y = 1.5
	// too, 10 away.
	earBoxFlush = [6]float64{22, 1.5, 30, 13.5, -6, 1}
	flushEars   = [][6]float64{earBoxA, earBoxFlush}
)

// flushRootFill is the fill of a root whose ear's front face lies 1.5 from the
// round's axis.
func flushRootFill(size, _ float64) float64 { return rootFill(size, 1.5) }

func TestFacetedRetainedBlendSharedEndPlanes(t *testing.T) {
	t.Parallel()

	for _, c := range []struct {
		name   string
		ears   [][6]float64
		groups [][]float64
		// charge is the offset of the vertex the band reads by sign, when there is one.
		charge float64
	}{
		{name: "one ear's side root", ears: flushEars, groups: [][]float64{{12}}},
		{name: "the other ear's side root", ears: flushEars, groups: [][]float64{{22}}},
		{name: "each outer side root", ears: flushEars, groups: [][]float64{{4}}},
		{name: "all four side roots in one call", ears: flushEars, groups: [][]float64{{4, 12, 22, 30}}},
		{name: "facing side roots 3.5 apart in one call", ears: [][6]float64{earBoxA, {15.5, 1.5, 23.5, 13.5, -6, 1}},
			groups: [][]float64{{12, 15.5}}},
		// The first blend's cut of the round lies a few thousandths off the front plane,
		// joined to the next ear's corner on the plane by a long facet edge along it:
		// the later band reads that cut by the sign of its offset and charges the offset.
		{name: "facing side roots in turn", ears: flushEars, groups: [][]float64{{12}, {22}}},
		{name: "facing side roots in turn, the other way", ears: flushEars, groups: [][]float64{{22}, {12}}},
		// A vertex of the round beside the other ear's corner lies 0.003 from the front
		// plane, and the band's cut of the edge to it lies on the plane within that bound.
		{name: "facing side roots 1.5 apart", ears: [][6]float64{earBoxA, {13.5, 1.5, 21.5, 13.5, -6, 1}},
			groups: [][]float64{{12}}, charge: 0.003},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			filletChargedRoots(t, c.ears, flushRootFill, c.charge, c.groups...)
		})
	}
}

func TestFacetedRetainedBlendSharedEndPlanesRefusals(t *testing.T) {
	t.Parallel()

	const wall = "another part of the wall lies within the blend's reach of the edge"
	// refuses fillets the groups of roots in turn and requires the last call to
	// refuse with text, the earlier ones to build.
	refuses := func(t *testing.T, body *decad.Body, text string, groups ...[]float64) {
		t.Helper()
		var err error
		for k, group := range groups {
			body, err = decad.Fillet(t.Context(), body, sharedBackSeed(group...), units.Millimeters(1),
				decad.WithTangentChain())
			if k < len(groups)-1 {
				require.NoError(t, err, "group %d", k)
			}
		}
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, text)
	}
	// Facing roots whose bands overlap are refused by the wall gate, not by an end
	// plane's reading: the first blend's setback line is a boundary chord of the
	// wall, and it meets the later band.
	t.Run("facing roots whose bands overlap", func(t *testing.T) {
		t.Parallel()
		// The second root's band starts 0.5 inside the first blend's setback, on
		// the back plane both ears share.
		ears := [][6]float64{earBoxA, {13.5, 2, 21.5, 13.5, -6, 1}}
		refuses(t, notchedBody(t, ears...), wall, []float64{12}, []float64{13.5})
	})
	t.Run("facing roots whose bands overlap on shared planes in one call", func(t *testing.T) {
		t.Parallel()
		refuses(t, notchedBody(t, earBoxA, [6]float64{13.5, 1.5, 21.5, 13.5, -6, 1}), wall, []float64{12, 13.5})
	})
	t.Run("facing roots 0.8 apart in one call", func(t *testing.T) {
		t.Parallel()
		refuses(t, notchedBody(t, flushEars[0], [6]float64{12.8, 1.5, 20.8, 13.5, -6, 1}), wall,
			[]float64{12, 12.8})
	})
	t.Run("a hole in the round inside the band on shared planes", func(t *testing.T) {
		t.Parallel()
		w, doc := sketch.NewWorld(), decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
		body, err := decad.Cut(t.Context(), notchedBlock(t, w, doc, flushEars...),
			zCylinderAt(t, &retentionScene{w: w, doc: doc}, 12.5, 2.2, 0.3, -1, 5))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), body, sharedBackSeed(12), units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "another part of the wall lies within the blend's reach of the edge")
	})
}

// endRoundRows reads the x of each row of vertices the held round along y at the
// block's end carries between x = 0.5 and 1: the rows run the whole tray's length,
// and an ear's outer side that lies within a few thousandths of one has the row's
// vertices within their bound of its plane.
func endRoundRows(t *testing.T) []float64 {
	t.Helper()
	mesh, err := notchedBody(t).Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	seen := map[float64]struct{}{}
	for _, p := range mesh.Vertices() {
		if p.X > 0.5 && p.X < 1 && p.Z < 2 {
			seen[p.X] = struct{}{}
		}
	}
	rows := slices.Sorted(maps.Keys(seen))
	require.GreaterOrEqual(t, len(rows), 2)
	return rows
}

// TestFacetedRetainedBlendEndRoundChordRow places the outer side of the back
// root's ear (the end plane where the arc ends) a few thousandths from a row of
// the end round's vertices, on either side of it: the row's vertices and the cuts
// of the band on its edges lie within their bounds of that plane, and the band
// reads them by the sign of their offset (retained-edge-blend §4.3).
func TestFacetedRetainedBlendEndRoundChordRow(t *testing.T) {
	t.Parallel()

	rows := endRoundRows(t)
	for _, row := range []float64{rows[0], rows[len(rows)-1]} {
		for _, d := range []float64{-0.002, -0.0005, 0.0005, 0.002} {
			t.Run(fmt.Sprintf("row %.4f%+.4f", row, d), func(t *testing.T) {
				t.Parallel()
				const size = 1.0
				c := notchedRoots[1]
				c.ear[0] = row + d
				offset := notchShortR - c.ear[0]
				body := notchedBody(t, c.ear)
				volBefore, err := body.Volume()
				require.NoError(t, err)
				held, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
				require.NoError(t, err)

				out, err := decad.Fillet(t.Context(), body, c.seed(), units.Millimeters(size), decad.WithTangentChain())
				require.NoError(t, err)

				af := size * size * (1 - math.Pi/4)
				added := af*c.lineLen + earRegion(c.radius, 1, size, true, func(rho float64) float64 {
					return rho * math.Asin(offset/rho)
				})
				volAfter, err := out.Volume()
				require.NoError(t, err)
				requireVolumeChange(t, volBefore, volAfter, -added)

				// The bound grows by the strip's own terms and the charge: a vertex within
				// held + |d| of the plane has its bound raised to that over the sine s of
				// the round's slant to the plane, and the cuts made from it carry it, the
				// walls of two bands reading it once each: at most four times that.
				mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				own := revolveMeshBound(t, held) + 2*math.Ldexp(size, -10)
				sine := math.Sqrt(1 - (offset/c.radius)*(offset/c.radius))
				require.LessOrEqual(t, revolveMeshBound(t, mesh),
					own+4*(revolveMeshBound(t, held)+math.Abs(d))/sine)
				require.True(t, mesh.BoundaryVerified())
				require.True(t, mesh.VolumeVerified())
				rep, err := out.Document().Verify(t.Context())
				require.NoError(t, err)
				br, err := rep.ForBody(out)
				require.NoError(t, err)
				require.Equal(t, decad.Sound, br.Status)
				requireNoCrossingTriangles(t, out)
			})
		}
	}
}

// TestFacetedRetainedBlendSharedEndPlanesBoundsCoverRound samples the result of
// facing roots filleted in turn against the exact cylinder of the block's round
// (axis y = z = 3, r = 3) near the end plane y = 1.5, where the second band reads
// the first blend's cut by sign and charges it. Each vertex of the held round that
// the result keeps, and each centroid of a facet of three such vertices, lies
// within the published mesh bound of the cylinder. The held vertices lie on the
// cylinder, so the vertex samples are a floor; the facet centroids lie inside it
// by the chord's sagitta, which the bound must cover. The sign-read vertices' own
// bounds are checked in the unit tests against every world the held claims allow.
func TestFacetedRetainedBlendSharedEndPlanesBoundsCoverRound(t *testing.T) {
	t.Parallel()

	for name, groups := range map[string][][]float64{
		"in turn":           {{12}, {22}},
		"in turn, reversed": {{22}, {12}},
		"in one call":       {{12, 22}},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			body := notchedBody(t, flushEars...)
			heldMesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			held := map[r3.Vec]struct{}{}
			for _, p := range heldMesh.Vertices() {
				held[p] = struct{}{}
			}
			for _, g := range groups {
				body, err = decad.Fillet(t.Context(), body, sharedBackSeed(g...), units.Millimeters(1), decad.WithTangentChain())
				require.NoError(t, err)
			}
			mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			bound := revolveMeshBound(t, mesh)
			off := func(p r3.Vec) float64 { return math.Abs(math.Hypot(p.Y-3, p.Z-3) - 3) }
			near := func(p r3.Vec) bool {
				_, ok := held[p]
				return ok && p.X > 4 && p.X < 36 && math.Abs(p.Y-1.5) < 0.2 && p.Z < 3.2 && off(p) < 0.01
			}
			verts, samples := mesh.Vertices(), 0
			for _, p := range verts {
				if near(p) {
					samples++
					require.LessOrEqual(t, off(p), bound, "vertex %v", p)
				}
			}
			for _, tri := range mesh.Triangles() {
				a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
				if !near(a) || !near(b) || !near(c) {
					continue
				}
				samples++
				mid := a.Add(b).Add(c).Scale(1.0 / 3)
				require.LessOrEqual(t, off(mid), bound, "facet centroid %v", mid)
			}
			require.Positive(t, samples, "no kept vertex of the round near the end plane")
		})
	}
}
