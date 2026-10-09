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
// Removing an x wall instead refuses with SG5 and leaves the receiver live.
// Shown to fail with Shell ignoring the route's body (the call then read the
// generic "straight prism" refusal).
func TestBrepShellRemovesTheTop(t *testing.T) {
	t.Parallel()
	up := r3.NewVec(0, 0, 1)

	doc, bar := crossDrilledBar(t)
	before := doc.Bodies()
	_, err := bar.Shell(t.Context(), decad.Faces(decad.Facing(r3.NewVec(-1, 0, 0))).Exactly(1), units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "modify-general SG5")
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

	// 5632 + 220π, with π enclosed by its two neighbouring floats.
	vol, err := shelled.Volume()
	require.NoError(t, err)
	piLo, piHi := math.Nextafter(math.Pi, 0), math.Nextafter(math.Pi, 4)
	lo := new(big.Rat).Add(big.NewRat(5632, 1), new(big.Rat).Mul(big.NewRat(220, 1), new(big.Rat).SetFloat64(piLo)))
	hi := new(big.Rat).Add(big.NewRat(5632, 1), new(big.Rat).Mul(big.NewRat(220, 1), new(big.Rat).SetFloat64(piHi)))
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
	require.Equal(t, 13, strings.Count(text, "=ADVANCED_FACE("))
	require.Equal(t, 2, strings.Count(text, "=CYLINDRICAL_SURFACE("))
}
