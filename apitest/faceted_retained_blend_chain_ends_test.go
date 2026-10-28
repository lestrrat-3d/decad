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

// The chain-end fixtures of agent-docs/retained-modify-reach-design.md §12 for
// increment M8: open tangent chains of a mesh Boolean result whose two ends
// lie on one end face G. G gives up the blend's end cap where it is convex at
// the corner and takes it where it is reflex. Each outline runs line, quarter
// arc, line of 14, quarter arc, line, with r = 3 arcs.

// chainEndCap is a fillet's end cap of radius concaveFilletR.
const chainEndCap = concaveFilletR * concaveFilletR * (1 - math.Pi/4)

// chainEndFillet is the volume a fillet of radius concaveFilletR moves along
// an outline whose lines sum to lines and whose two quarter arcs have their
// material inside the arcs' circle.
func chainEndFillet(lines float64) float64 {
	const r = concaveFilletR
	xbar := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
	return chainEndCap * (lines + math.Pi*(concaveR-xbar))
}

// planeFaceAtX returns the one plane face of b that lies in the plane x = x0.
func planeFaceAtX(t *testing.T, b *decad.Body, x0 float64) *decad.Face {
	t.Helper()
	var found *decad.Face
	for _, f := range b.Faces() {
		s, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(math.Abs(s.Frame.N().X)-1) > 1e-9 || math.Abs(firstVertex(t, f).X-x0) > 1e-9 {
			continue
		}
		require.Nil(t, found, "two plane faces at x = %g", x0)
		found = f
	}
	require.NotNil(t, found, "no plane face at x = %g", x0)
	return found
}

// requireFaceAreaChange checks that the plane face at x = x0 grew by want
// from before to after, within the two faces' own area bounds.
func requireFaceAreaChange(t *testing.T, before, after *decad.Body, x0, want float64) {
	t.Helper()
	a0, err := planeFaceAtX(t, before, x0).Area()
	require.NoError(t, err)
	a1, err := planeFaceAtX(t, after, x0).Area()
	require.NoError(t, err)
	got := a1.Value.Mag() - a0.Value.Mag()
	bound := a0.Bound.Mag() + a1.Bound.Mag()
	t.Logf("face x = %g area change %.9f, want %.9f, bound %.3g", x0, got, want, bound)
	require.InDelta(t, want, got, bound)
}

// xEdgeAt selects the one x-parallel edge that ends at p.
func xEdgeAt(p r3.Vec) decad.EdgeSelector {
	return decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(p)).Exactly(1)
}

// blindPocket is the 40x40x10 plate with a 20x20 pocket over x, y = 10 … 30
// whose x = 30 corners are rounded at r = 3 and whose x = 10 corners stay
// sharp, floor at z = 5. The floor's outline ends at (10,10,5) and (10,30,5)
// on the x = 10 wall, which is convex at both.
func blindPocket(t *testing.T) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	plate := rectBox(t, w, doc, 0, 0, 40, 40, 10)
	up := decad.ParallelTo(r3.NewVec(0, 0, 1))
	tool, err := decad.Fillet(t.Context(), rectBox(t, w, doc, 10, 10, 30, 30, 20),
		decad.Edges(up, decad.EndpointAt(r3.NewVec(30, 10, 0))).Or(up, decad.EndpointAt(r3.NewVec(30, 30, 0))).Exactly(2),
		units.Millimeters(concaveR))
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), plate, placed(t, tool, 5))
	require.NoError(t, err)
	return out
}

// ledgeOnWall is a 10x40x30 wall joined with a ledge over x = x0 … 30,
// y = 10 … 30, z = 4 … 10. corners names the ledge's vertical edges to round
// at r = 3. The ledge's top outline ends on the wall's x = 10 face, which is
// reflex where the ledge stands out of it.
func ledgeOnWall(t *testing.T, x0 float64, corners int) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	wall := rectBox(t, w, doc, 0, 0, 10, 40, 30)
	up := decad.ParallelTo(r3.NewVec(0, 0, 1))
	vertical := decad.Edges(up, decad.EndpointAt(r3.NewVec(30, 10, 0))).Or(up, decad.EndpointAt(r3.NewVec(30, 30, 0)))
	if corners == 4 {
		vertical = vertical.Or(up, decad.EndpointAt(r3.NewVec(x0, 10, 0))).Or(up, decad.EndpointAt(r3.NewVec(x0, 30, 0)))
	}
	ledge, err := decad.Fillet(t.Context(), rectBox(t, w, doc, x0, 10, 30, 30, 6), vertical.Exactly(corners),
		units.Millimeters(concaveR))
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), wall, placed(t, ledge, 4))
	require.NoError(t, err)
	return out
}

// tongueBox is the 40x20x20 box with its x = 40 corners rounded at r = 3 and
// a Ø8 bore along y. Its top outline runs from (0,0,20) to (0,20,20), both on
// the x = 0 face, and is the whole of the top face's loop but that face's
// edge.
func tongueBox(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	up := decad.ParallelTo(r3.NewVec(0, 0, 1))
	box, err := decad.Fillet(t.Context(), rectBox(t, sc.w, sc.doc, 0, 0, 40, 20, 20),
		decad.Edges(up, decad.EndpointAt(r3.NewVec(40, 0, 0))).Or(up, decad.EndpointAt(r3.NewVec(40, 20, 0))).Exactly(2),
		units.Millimeters(concaveR))
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), box, sc.tool(t, 20))
	require.NoError(t, err)
	return out
}

// stepBlock is a 40x20x30 block with its x = 40 corners rounded at r = 3,
// cut down to z = 10 for x > 10. The z = 10 outline ends on the riser at
// x = 10, where the front and back faces wrap the corner.
func stepBlock(t *testing.T) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	up := decad.ParallelTo(r3.NewVec(0, 0, 1))
	block, err := decad.Fillet(t.Context(), rectBox(t, w, doc, 0, 0, 40, 20, 30),
		decad.Edges(up, decad.EndpointAt(r3.NewVec(40, 0, 0))).Or(up, decad.EndpointAt(r3.NewVec(40, 20, 0))).Exactly(2),
		units.Millimeters(concaveR))
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), block, placed(t, rectBox(t, w, doc, 10, -5, 50, 25, 30), 10))
	require.NoError(t, err)
	return out
}

// arcEndSlot is the 40x40x10 plate with a slot over x = −2 … 30,
// y = 10 … 30, all four corners rounded at r = 3, floor at z = 5. The x = 0
// face cuts the slot's x = −2 arcs, so the floor's outline ends on arcs.
func arcEndSlot(t *testing.T) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	plate := rectBox(t, w, doc, 0, 0, 40, 40, 10)
	tool, err := decad.Fillet(t.Context(), rectBox(t, w, doc, -2, 10, 30, 30, 20),
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(concaveR))
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), plate, placed(t, tool, 5))
	require.NoError(t, err)
	return out
}

func TestFacetedRetainedBlendChainEnds(t *testing.T) {
	t.Parallel()

	t.Run("a reflex foot fillets and its wall takes both caps", func(t *testing.T) {
		t.Parallel()
		joined := ledgeOnWall(t, 5, 2)
		before, err := joined.Volume()
		require.NoError(t, err)
		const r = concaveFilletR
		out, err := decad.Fillet(t.Context(), joined, xEdgeAt(r3.NewVec(27, 10, 10)), units.Millimeters(r),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, chainEndFillet(17+14+17))
		requireFaceAreaChange(t, joined, out, 10, 2*chainEndCap)
		cylinders, tori, majors := blendKinds(out, r)
		require.Equal(t, 3, cylinders)
		require.Equal(t, 2, tori)
		for _, m := range majors {
			require.InDelta(t, concaveR-r, m, 1e-12)
		}
		requireVerifiedMesh(t, out)
	})

	t.Run("a slot's floor fillets out through the face it leaves by", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocket(t, false)
		before, err := pocket.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), pocket, xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -chainEndFillet(27+14+27))
		requireFaceAreaChange(t, pocket, out, 0, 2*chainEndCap)
		requireBlendsFaceVoid(t, out, 5)
		requireVerifiedMesh(t, out)
	})

	t.Run("a chamfer of a slot's floor", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocket(t, false)
		before, err := pocket.Volume()
		require.NoError(t, err)
		out, err := decad.Chamfer(t.Context(), pocket, xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(1),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -(27+14+27+math.Pi*(concaveR-1.0/3))/2)
		requireFaceAreaChange(t, pocket, out, 0, 2*0.5)
		requireVerifiedMesh(t, out)
	})

	t.Run("a blind pocket's floor fillets and the sharp wall gives up both caps", func(t *testing.T) {
		t.Parallel()
		pocket := blindPocket(t)
		before, err := pocket.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), pocket, xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -chainEndFillet(17+14+17))
		requireFaceAreaChange(t, pocket, out, 10, -2*chainEndCap)
		requireBlendsFaceVoid(t, out, 5)
		requireVerifiedMesh(t, out)
	})

	t.Run("a tongue's top outline fillets with both ends on one face", func(t *testing.T) {
		t.Parallel()
		tongue := tongueBox(t)
		before, err := tongue.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), tongue, xEdgeAt(r3.NewVec(37, 0, 20)), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, chainEndFillet(37+14+37))
		requireFaceAreaChange(t, tongue, out, 0, -2*chainEndCap)
		requireVerifiedMesh(t, out)
	})

	t.Run("an end whose wall wraps the corner is refused", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Fillet(t.Context(), stepBlock(t), xEdgeAt(r3.NewVec(37, 0, 10)), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a chain end's base or wall is reflex at the corner")
	})

	t.Run("an end whose base wraps the corner is refused", func(t *testing.T) {
		t.Parallel()
		// The pocket's mouth: the plate's top is reflex at the sharp corners.
		_, err := decad.Fillet(t.Context(), blindPocket(t), xEdgeAt(r3.NewVec(27, 10, 10)), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a chain end's base or wall is reflex at the corner")
	})

	t.Run("a concave open chain that ends on an arc is refused", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Fillet(t.Context(), arcEndSlot(t), xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a concave open tangent chain ends on an arc; only a straight end is built")
	})

	t.Run("an arc end on a reflex end face is refused", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Fillet(t.Context(), ledgeOnWall(t, 8, 4), xEdgeAt(r3.NewVec(27, 10, 10)), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "an arc end of a tangent chain needs an end face convex at the corner")
	})
}
