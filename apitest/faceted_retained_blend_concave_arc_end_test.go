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

// The fixtures of agent-docs/modify-design.md Part M §5.5 step 5 for a
// concave open chain whose end edge is an arc. A pocket floor outline runs
// arc, line, quarter arc, line, quarter arc, line, arc with r = 3 arcs about
// centres 1 mm from the end face G, so G cuts each end arc 1 mm from its
// centre. A fillet of radius concaveFilletR adds material in the floor's
// corner on the void side; G gives its corner cap up where it is convex there
// (a blind end) and takes it where it is reflex (the floor runs out through
// the face).

const concaveArcEndOffset = 1.0

// concaveArcEndRegion calls f for each cell of the fillet's corner section,
// the part of the square next to the floor and the wall that lies outside the
// fillet's tube. dr is the distance from the wall into the void and dz the
// height above the floor.
func concaveArcEndRegion(fillet bool, f func(dr, dz, cell float64)) {
	const n, r = 1500, concaveFilletR
	for i := range n {
		dr := r * (float64(i) + 0.5) / n
		for j := range n {
			dz := r * (float64(j) + 0.5) / n
			if (fillet && (dr-r)*(dr-r)+(dz-r)*(dz-r) > r*r) || (!fillet && dr+dz < r) {
				f(dr, dz, r*r/(n*n))
			}
		}
	}
}

// concaveArcEndAdded is the volume a fillet adds along an outline of two arcs
// that end on G, two quarter arcs and lines summing to lines. A row at radius
// ρ sweeps asin(offset/ρ) of an end arc.
func concaveArcEndAdded(fillet bool, kappa, lines float64) float64 {
	af, sum := 0.0, 0.0
	concaveArcEndRegion(fillet, func(dr, _, cell float64) {
		rho := concaveR + kappa*dr
		af += cell
		sum += cell * rho * (2*math.Pi/2 + 2*math.Asin(concaveArcEndOffset/rho))
	})
	return af*lines + sum
}

// concaveArcEndCap is the area the fillet's blend covers of the end plane G at
// one end arc: the section of the blend by G.
func concaveArcEndCap(fillet bool, kappa float64) float64 {
	area := 0.0
	concaveArcEndRegion(fillet, func(dr, _, cell float64) {
		rho := concaveR + kappa*dr
		area += cell * rho / math.Sqrt(rho*rho-concaveArcEndOffset*concaveArcEndOffset)
	})
	return area
}

// blindArcEndPocket is the 40x40x10 plate with a pocket floor at z = 5 whose
// outline is the arcEndSlot's, but whose end wall at x = 10 crops the tool:
// G is convex at both corners where the floor's end
// arcs meet it.
func blindArcEndPocket(t *testing.T) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	plate := rectBox(t, w, doc, 0, 0, 40, 40, 10)
	tool, err := decad.Fillet(t.Context(), rectBox(t, w, doc, 8, 10, 30, 30, 20),
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(concaveR))
	require.NoError(t, err)
	cropped, err := decad.Cut(t.Context(), tool, rectBox(t, w, doc, 7, 9, 10, 31, 20))
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), plate, placed(t, cropped, 5))
	require.NoError(t, err)
	return out
}

// bossArcEnd is the 40x40x10 plate against a 10x40x30 wall over x = −10 … 0,
// with a boss over x = −2 … 30, y = 10 … 30 and z = 8 … 20, its vertical
// corners rounded at r = 3. The boss's x < 0 part lies inside the wall. The
// plate's top holds the boss's root as an open chain whose arcs end on the
// wall's x = 0 face, which is convex at both corners (a blind end, the void
// outside the arcs' circle).
func bossArcEnd(t *testing.T) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	plate, err := decad.Union(t.Context(), rectBox(t, w, doc, 0, 0, 40, 40, 10), rectBox(t, w, doc, -10, 0, 0, 40, 30))
	require.NoError(t, err)
	boss, err := decad.Fillet(t.Context(), rectBox(t, w, doc, -2, 10, 30, 30, 12),
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(concaveR))
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), plate, placed(t, boss, 8))
	require.NoError(t, err)
	return out
}

// bossRootAt selects the boss's root edge along y = 10: the 26 mm line near
// the plate's top. The mesh Boolean result holds its vertices within a bound
// of the plate's top, so the edge is found by its own end.
func bossRootAt(t *testing.T, b *decad.Body) decad.EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(decad.Line3); !ok {
			continue
		}
		p, q := e.Start().Position().Value, e.End().Position().Value
		if math.Abs(q.Sub(p).Len()-26) > 0.1 {
			continue
		}
		if math.Abs(p.Z-10) < 0.01 && math.Abs(p.Y-10) < 0.01 {
			return decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(p)).Exactly(1)
		}
	}
	require.Fail(t, "no root edge")
	return nil
}

func TestFacetedRetainedBlendConcaveArcEnd(t *testing.T) {
	t.Parallel()

	t.Run("a slot's floor fillets out through the face its arcs end on", func(t *testing.T) {
		t.Parallel()
		slot := arcEndSlot(t)
		before, err := slot.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), slot, xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveArcEndAdded(true, -1, 26+14+26))
		requireFaceAreaChange(t, slot, out, 0, 2*concaveArcEndCap(true, -1))
		requireBlendsFaceVoid(t, out, 7)
		requireVerifiedMesh(t, out)
	})

	t.Run("a chamfer of a slot's floor runs out through the face its arcs end on", func(t *testing.T) {
		t.Parallel()
		slot := arcEndSlot(t)
		before, err := slot.Volume()
		require.NoError(t, err)
		out, err := decad.Chamfer(t.Context(), slot, xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveArcEndAdded(false, -1, 26+14+26))
		requireFaceAreaChange(t, slot, out, 0, 2*concaveArcEndCap(false, -1))
		requireVerifiedMesh(t, out)
	})

	t.Run("a blind end gives both caps up", func(t *testing.T) {
		t.Parallel()
		pocket := blindArcEndPocket(t)
		before, err := pocket.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), pocket, xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveArcEndAdded(true, -1, 16+14+16))
		requireFaceAreaChange(t, pocket, out, 10, -2*concaveArcEndCap(true, -1))
		requireBlendsFaceVoid(t, out, 7)
		requireVerifiedMesh(t, out)
	})

	t.Run("a boss's root fillets to the wall its arcs end on, which gives up both caps", func(t *testing.T) {
		t.Parallel()
		boss := bossArcEnd(t)
		before, err := boss.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), boss, bossRootAt(t, boss), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -concaveArcEndAdded(true, 1, 26+14+26))
		requireFaceAreaChange(t, boss, out, 0, -2*concaveArcEndCap(true, 1))
		requireBlendsFaceVoid(t, out, 7)
		requireVerifiedMesh(t, out)
	})

	t.Run("a solid crossing the cap the slot's end face takes is refused", func(t *testing.T) {
		t.Parallel()
		// The box floats 0.05 above the floor and crosses x = 0 inside the cap
		// at the slot's corner (0,10.17,5), clear of the blend's strip.
		w, doc := sketch.NewWorld(), decad.New()
		joined, err := decad.Union(t.Context(), arcEndSlotIn(t, w, doc),
			placed(t, rectBox(t, w, doc, -0.9, 10.2, 0.1, 10.3, 0.1), 5.05))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), joined, xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(concaveFilletR),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the cap an end face takes meets another facet of the body at")
	})

	t.Run("a blend circle that misses the end face is refused", func(t *testing.T) {
		t.Parallel()
		// The end face lies 1 mm from the arcs' centres, so the plane's setback
		// circle of radius 3 - 2 = 1 never crosses it.
		_, err := decad.Fillet(t.Context(), arcEndSlot(t), xEdgeAt(r3.NewVec(27, 10, 5)), units.Millimeters(2),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "does not reach an end face")
	})

	t.Run("a convex chain's arc end on a reflex end face is refused", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Fillet(t.Context(), ledgeOnWall(t, 8, 4), xEdgeAt(r3.NewVec(27, 10, 10)),
			units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "an arc end of a tangent chain needs an end face convex at the corner")
	})
}
