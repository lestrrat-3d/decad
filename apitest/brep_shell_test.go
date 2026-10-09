package apitest_test

import (
	"bytes"
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// crossDrilledBar is docs/modify-general-design.md §1's P1 through the
// public API: the 40×20×20 box cut by a Ø6 hole along y through
// (20, ·, 10).
func crossDrilledBar(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)

	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	ds, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := ds.CreatePoint(20, 10)
	ds.Fix(c)
	ds.CreateCircle(c, 3)
	_, err = ds.Solve(t.Context())
	require.NoError(t, err)
	drill, err := doc.Extrude(ds, ds.Profiles()[0], decad.Symmetric{D: units.Millimeters(11)})
	require.NoError(t, err)
	bar, err := decad.Cut(t.Context(), box, drill)
	require.NoError(t, err)
	return doc, bar
}

// TestBrepShellRemovesTheTop pins route S through the public API on P1:
// removing the top at 2 mm builds 13 faces, two of them cylinders (the
// receiver's hole and the cavity's dilated hole), volume 5632 + 220π inside
// its published bound, Verify reads it Sound, and STEP writes it through
// the analytic arm, one ADVANCED_FACE per face. The receiver is retired.
// Removing both x walls instead, no connected run of walls, refuses with
// shell-opening SO6 and leaves the receiver live. Shown to fail with Shell
// ignoring the route's body (the call then read the generic "straight prism"
// refusal).
func TestBrepShellRemovesTheTop(t *testing.T) {
	t.Parallel()
	up := r3.NewVec(0, 0, 1)

	doc, bar := crossDrilledBar(t)
	before := doc.Bodies()
	_, err := bar.Shell(t.Context(), decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))).Exactly(2), units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "shell-opening SO6")
	require.Equal(t, before, doc.Bodies())

	shelled, err := bar.Shell(t.Context(), decad.Faces(decad.Facing(up)).Exactly(1), units.Millimeters(2))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, shelled.Faces(), 13)
	cylinders := 0
	for _, f := range shelled.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cylinders++
		}
	}
	require.Equal(t, 2, cylinders)

	requireShellVolumeSoundAnalytic(t, doc, shelled, 5632, 220, 13)
}

// requireShellVolumeSoundAnalytic requires the shelled body's published
// volume to cover a + b·π, with π enclosed by its two neighbouring floats,
// Verify to read it Sound, and STEP to write it through the analytic arm:
// one ADVANCED_FACE per face, two of them cylinders.
func requireShellVolumeSoundAnalytic(t *testing.T, doc *decad.Document, shelled *decad.Body, a, b int64, faces int) {
	t.Helper()
	vol, err := shelled.Volume()
	require.NoError(t, err)
	piLo, piHi := math.Nextafter(math.Pi, 0), math.Nextafter(math.Pi, 4)
	lo := new(big.Rat).Add(big.NewRat(a, 1), new(big.Rat).Mul(big.NewRat(b, 1), new(big.Rat).SetFloat64(piLo)))
	hi := new(big.Rat).Add(big.NewRat(a, 1), new(big.Rat).Mul(big.NewRat(b, 1), new(big.Rat).SetFloat64(piHi)))
	held := new(big.Rat).SetFloat64(vol.Value.Base())
	bound := new(big.Rat).SetFloat64(vol.Bound.Base())
	require.LessOrEqual(t, new(big.Rat).Sub(held, bound).Cmp(lo), 0)
	require.GreaterOrEqual(t, new(big.Rat).Add(held, bound).Cmp(hi), 0)

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(shelled)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, br.Status)

	var buf bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &buf, shelled, units.Millimeters(0.1),
		export.WithSTEPName("shell"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad")))
	text := buf.String()
	require.Equal(t, faces, strings.Count(text, "=ADVANCED_FACE("))
	require.Equal(t, 2, strings.Count(text, "=CYLINDRICAL_SURFACE("))
}

// TestBrepShellRemovesAWallAndTheTop pins route S with a removed wall run
// through the public API on P1: removing the y = 0 wall and the top at 2 mm
// opens the box along two faces that share an edge. The cavity section is
// [2, 38] × [0, 18] over z ∈ [2, 20], less the hole dilated to radius 5 over
// y ∈ [0, 18], so the volume is 16000 − 180π − (648·18 − 450π) = 4336 + 270π.
// The result holds 13 faces, Verify reads it Sound, and STEP writes it
// through the analytic arm. Shown to fail with the removed wall refused as
// before route S took wall runs (SG5).
func TestBrepShellRemovesAWallAndTheTop(t *testing.T) {
	t.Parallel()
	doc, bar := crossDrilledBar(t)
	sel := decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))).Or(decad.Facing(r3.NewVec(0, 0, 1))).Exactly(2)
	shelled, err := bar.Shell(t.Context(), sel, units.Millimeters(2))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, shelled.Faces(), 13)
	require.Len(t, shelled.Lumps(), 1)
	requireShellVolumeSoundAnalytic(t, doc, shelled, 4336, 270, 13)
}
