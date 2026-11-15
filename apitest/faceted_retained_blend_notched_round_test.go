package apitest_test

import (
	"fmt"
	"math"
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
