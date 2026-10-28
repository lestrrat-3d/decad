package apitest_test

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The concave fixtures of agent-docs/retained-modify-reach-design.md §12 for
// increment M7: Fillet and Chamfer of a closed concave outline of a mesh
// Boolean result. The pocket is roundedPocket's: a 40x40x10 plate with a
// 20x20, r = 3 pocket whose floor at z = 5 is bounded by four lines of 14 and
// four quarter arcs. The boss is a 20x20, r = 3 boss standing on the plate's
// top at z = 10. A concave blend fills the corner, so the volume grows.

const (
	concaveR     = 3.0
	concaveLines = 4 * 14
)

// concaveFilletR is the radius of every concave chain fillet below.
const concaveFilletR = 1.0

// concaveFilletFill is the volume a fillet of radius concaveFilletR adds along
// the closed concave outline: the corner area times the path length at the
// corner's centroid, which lies inside the arcs' circle on the pocket floor
// (kappa −1) and outside it around the boss (kappa +1).
func concaveFilletFill(kappa float64) float64 {
	const r = concaveFilletR
	af := r * r * (1 - math.Pi/4)
	xbar := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
	return af * (concaveLines + 2*math.Pi*(concaveR+kappa*xbar))
}

// concaveChamferFill is the same for a chamfer of setback tp on the base and
// tw on the walls, on the pocket floor.
func concaveChamferFill(tp, tw float64) float64 {
	return tp * tw / 2 * (concaveLines + 2*math.Pi*(concaveR-tp/3))
}

// floorFront selects the pocket floor's front edge from (13,10,floor) to
// (27,10,floor).
func floorFront(floor float64) decad.EdgeSelector {
	return decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(r3.NewVec(13, 10, floor))).Exactly(1)
}

// roundedBoss is the 40x40x10 plate with the rounded 20x20 boss placed over
// z = 8 … 20 and joined to it, so the plate's top holds the boss's root as a
// closed concave outline.
func roundedBoss(t *testing.T) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	plate := rectBox(t, w, doc, 0, 0, 40, 40, 10)
	boss := rectBox(t, w, doc, 10, 10, 30, 30, 12)
	boss, err := decad.Fillet(t.Context(), boss, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4),
		units.Millimeters(concaveR))
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), plate, placed(t, boss, 8))
	require.NoError(t, err)
	return out
}

// pocketWithCube is roundedPocket joined with a box that floats over the
// floor: x from 15 to 17, and a side by side square from (y0, z0) in y and z.
func pocketWithCube(t *testing.T, y0, z0, side float64) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	plate := rectBox(t, w, doc, 0, 0, 40, 40, 10)
	tool, err := decad.Fillet(t.Context(), rectBox(t, w, doc, 10, 10, 30, 30, 20),
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(concaveR))
	require.NoError(t, err)
	pocket, err := decad.Cut(t.Context(), plate, placed(t, tool, 5))
	require.NoError(t, err)
	cube := rectBox(t, w, doc, 15, y0, 17, y0+side, side)
	joined, err := decad.Union(t.Context(), pocket, placed(t, cube, z0))
	require.NoError(t, err)
	return joined
}

// requireBlendsFaceVoid checks that each of the n blend faces of radius
// concaveFilletR points into the void at a vertex: toward a cylinder's axis,
// toward a torus's tube centre.
func requireBlendsFaceVoid(t *testing.T, b *decad.Body, n int) {
	t.Helper()
	const r = concaveFilletR
	checked := 0
	for _, f := range b.Faces() {
		var centre func(p r3.Vec) r3.Vec
		switch s := f.Surface().(type) {
		case decad.Cylinder:
			if math.Abs(s.Radius.Base()-r) > 1e-12 {
				continue
			}
			centre = func(p r3.Vec) r3.Vec {
				q := p.Sub(s.Origin)
				return s.Origin.Add(s.Axis.Scale(q.Dot(s.Axis)))
			}
		case decad.Torus:
			if math.Abs(s.Minor.Base()-r) > 1e-12 {
				continue
			}
			centre = func(p r3.Vec) r3.Vec {
				q := p.Sub(s.Center)
				radial, ok := q.Sub(s.Axis.Scale(q.Dot(s.Axis))).Normalize()
				require.True(t, ok)
				return s.Center.Add(radial.Scale(s.Major.Base()))
			}
		default:
			continue
		}
		p := firstVertex(t, f)
		n, err := f.NormalAt(p)
		require.NoError(t, err)
		require.Positive(t, n.Value.Dot(centre(p).Sub(p)), "a blend at %v faces the material", p)
		checked++
	}
	require.Equal(t, n, checked)
}

// requireContactX checks that a strip clearance refusal names a contact point
// whose x lies in [lo, hi], where the crossing solid is.
func requireContactX(t *testing.T, err error, lo, hi float64) {
	t.Helper()
	const at = "the blend meets another facet of the body at {"
	msg := err.Error()
	i := strings.Index(msg, at)
	require.GreaterOrEqual(t, i, 0, msg)
	var p r3.Vec
	_, scanErr := fmt.Sscanf(msg[i+len(at):], "%g %g %g}", &p.X, &p.Y, &p.Z)
	require.NoError(t, scanErr, msg)
	require.GreaterOrEqual(t, p.X, lo, msg)
	require.LessOrEqual(t, p.X, hi, msg)
}

// requireBevelsFaceVoid checks that every bevel of a pocket floor chamfer,
// the 4 tilted planes and the 4 cones, points into the void at a vertex: up
// from the floor and in toward the pocket's axis through (20, 20).
func requireBevelsFaceVoid(t *testing.T, b *decad.Body) {
	t.Helper()
	checked := 0
	for _, f := range b.Faces() {
		switch s := f.Surface().(type) {
		case decad.Plane:
			if n := s.Frame.N(); math.Abs(n.Z) < 1e-9 || math.Abs(math.Abs(n.Z)-1) < 1e-9 {
				continue
			}
		case decad.Cone:
		default:
			continue
		}
		p := firstVertex(t, f)
		n, err := f.NormalAt(p)
		require.NoError(t, err)
		require.Positive(t, n.Value.Z, "a bevel at %v faces down into the floor", p)
		inward := r3.NewVec(20-p.X, 20-p.Y, 0)
		require.Positive(t, n.Value.Dot(inward), "a bevel at %v faces the wall", p)
		checked++
	}
	require.Equal(t, 8, checked)
}

func TestFacetedRetainedBlendConcaveChain(t *testing.T) {
	t.Parallel()

	t.Run("a pocket floor's outline fillets into the void", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocket(t, true)
		before, err := pocket.Volume()
		require.NoError(t, err)
		const r = concaveFilletR
		out, err := decad.Fillet(t.Context(), pocket, floorFront(5), units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveFilletFill(-1))
		cylinders, tori, majors := blendKinds(out, r)
		require.Equal(t, 4, cylinders)
		require.Equal(t, 4, tori)
		for _, m := range majors {
			// The ball lies inside the corner cylinder: the tube centre is R − r.
			require.InDelta(t, concaveR-r, m, 1e-12)
		}
		requireBlendsFaceVoid(t, out, 8)
		requireVerifiedMesh(t, out)
	})

	t.Run("a chamfer of the pocket floor's outline, equal setbacks", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocket(t, true)
		before, err := pocket.Volume()
		require.NoError(t, err)
		out, err := decad.Chamfer(t.Context(), pocket, floorFront(5), units.Millimeters(1), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveChamferFill(1, 1))
		requireBevelsFaceVoid(t, out)
		requireVerifiedMesh(t, out)
	})

	t.Run("a chamfer with the floor as reference takes both distances", func(t *testing.T) {
		t.Parallel()
		// Facing(+z) matches the plate's top as well, so the floor is named
		// by the placed tool's start cap, which the Cut keeps.
		w, doc := sketch.NewWorld(), decad.New()
		plate := rectBox(t, w, doc, 0, 0, 40, 40, 10)
		tool, err := decad.Fillet(t.Context(), rectBox(t, w, doc, 10, 10, 30, 30, 20),
			decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(concaveR))
		require.NoError(t, err)
		tool = placed(t, tool, 5)
		pocket, err := decad.Cut(t.Context(), plate, tool)
		require.NoError(t, err)
		before, err := pocket.Volume()
		require.NoError(t, err)
		floor := decad.Faces(decad.FaceCreatedBy(decad.CapStart(tool))).Exactly(1)
		out, err := decad.Chamfer(t.Context(), pocket, floorFront(5), units.Millimeters(2), decad.WithTangentChain(),
			decad.WithAsymmetricChamfer(floor, units.Millimeters(1)))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveChamferFill(2, 1))
		requireBevelsFaceVoid(t, out)
		requireVerifiedMesh(t, out)
	})

	t.Run("a boss's root fillets around the outside of its arcs", func(t *testing.T) {
		t.Parallel()
		joined := roundedBoss(t)
		before, err := joined.Volume()
		require.NoError(t, err)
		const r = concaveFilletR
		out, err := decad.Fillet(t.Context(), joined, floorFront(10), units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveFilletFill(1))
		cylinders, tori, majors := blendKinds(out, r)
		require.Equal(t, 4, cylinders)
		require.Equal(t, 4, tori)
		for _, m := range majors {
			// The ball lies outside the corner cylinder: the tube centre is R + r.
			require.InDelta(t, concaveR+r, m, 1e-12)
		}
		requireBlendsFaceVoid(t, out, 8)
		requireVerifiedMesh(t, out)
	})

	t.Run("all eight floor edges named build without the option", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocket(t, true)
		before, err := pocket.Volume()
		require.NoError(t, err)
		var outline *decad.EdgeQuery
		named := 0
		for _, e := range pocket.Edges() {
			p, q := e.Start().Position().Value, e.End().Position().Value
			if math.Abs(p.Z-5) > 1e-9 || math.Abs(q.Z-5) > 1e-9 {
				continue
			}
			named++
			if outline == nil {
				outline = decad.Edges(decad.EndpointAt(p), decad.EndpointAt(q))
				continue
			}
			outline = outline.Or(decad.EndpointAt(p), decad.EndpointAt(q))
		}
		require.Equal(t, 8, named)
		out, err := decad.Fillet(t.Context(), pocket, outline.Exactly(8), units.Millimeters(concaveFilletR))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveFilletFill(-1))
		cylinders, tori, majors := blendKinds(out, concaveFilletR)
		require.Equal(t, 4, cylinders)
		require.Equal(t, 4, tori)
		for _, m := range majors {
			require.InDelta(t, concaveR-concaveFilletR, m, 1e-12)
		}
		requireVerifiedMesh(t, out)
	})

	t.Run("a pocket shallower than the radius is refused", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocketAt(t, true, 9.2)
		_, err := decad.Fillet(t.Context(), pocket, floorFront(9.2), units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "the blend consumes the wall")
	})

	t.Run("a radius that reaches the arcs' axes is refused", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocket(t, true)
		_, err := decad.Fillet(t.Context(), pocket, floorFront(5), units.Millimeters(3), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "the blend reaches the rim's axis")
	})

	t.Run("a solid floating inside the fill is refused", func(t *testing.T) {
		t.Parallel()
		// The cube lies wholly between the floor's front corner and the
		// fillet's arc, clear of the floor and the wall.
		_, err := decad.Fillet(t.Context(), pocketWithCube(t, 10.05, 5.05, 0.1), floorFront(5), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the blended mesh fails its held audit")
		require.ErrorContains(t, err, "a result shell has inconsistent orientation")
	})

	t.Run("a solid floating clear of the fill is kept", func(t *testing.T) {
		t.Parallel()
		joined := pocketWithCube(t, 20, 5.05, 0.1)
		before, err := joined.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), joined, floorFront(5), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveFilletFill(-1))
		requireVerifiedMesh(t, out)
	})

	t.Run("a solid crossing the fill is refused", func(t *testing.T) {
		t.Parallel()
		// Only the cube's lower front edge reaches into the fill: its corner
		// (y, z) = (10.25, 5.25) lies between the floor's front corner and the
		// fillet's arc, and the rest of the cube lies in the void beyond it.
		_, err := decad.Fillet(t.Context(), pocketWithCube(t, 10.25, 5.25, 1), floorFront(5),
			units.Millimeters(concaveFilletR), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the blend meets another facet of the body")
		requireContactX(t, err, 15, 17)
	})
}
