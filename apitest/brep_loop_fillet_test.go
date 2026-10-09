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
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin route L's fillet arm of docs/loop-fillet-design.md
// through the public API: Body.Fillet of a complete loop of a planar face
// builds a brep body whose pipe patches are faces of the body, measured by
// the strip model (§5.3), and tessellate, export and take part in booleans.

// filletLoopPatches lists the faces carrying a filletLoop(f,l,p) role.
func filletLoopPatches(body *decad.Body) []*decad.Face {
	var out []*decad.Face
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "filletLoop(") {
				out = append(out, f)
			}
		}
	}
	return out
}

// requireMeasurementHoldsPi asserts the published reading's interval holds
// a + b·π at both ends of a 40-digit π bracket.
func requireMeasurementHoldsPi(t *testing.T, m decad.Measurement, a, b *big.Rat) {
	t.Helper()
	lo, _ := new(big.Rat).SetString("3.141592653589793238462643383279502884197")
	hi, _ := new(big.Rat).SetString("3.141592653589793238462643383279502884198")
	held := new(big.Rat).SetFloat64(m.Value.Base())
	bound := new(big.Rat).SetFloat64(m.Bound.Base())
	for _, p := range []*big.Rat{lo, hi} {
		want := new(big.Rat).Add(a, new(big.Rat).Mul(b, p))
		gap := new(big.Rat).Abs(new(big.Rat).Sub(held, want))
		require.LessOrEqual(t, gap.Cmp(bound), 0, "%s ± %s misses %s", m.Value, m.Bound, want.FloatString(15))
	}
}

// TestBrepLoopFilletPublicPlateTopLoop fillets the pocketed plate's top loop,
// the 40×40 outer loop of its top face, at r = 1.5. The plate is a stacked
// receiver, so the fillet takes route L and builds a closed brep whose four
// quarter cylinders meet along four quarter ellipses with semi-axes 1.5√2
// and 1.5. Table CF's a₁ = 160, a₂ = −4 give the volume
// 15000 − 160·J₁ + 4·J₂ = 29325/2 + 333π/4, Approximate since π enters it;
// every ellipse's published length encloses the quarter-ellipse arc; and the
// mesh closes over every face and encloses the volume.
func TestBrepLoopFilletPublicPlateTopLoop(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 40, 10)
	pocket, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, 10, 15, 30, 25, 5, 5))
	require.NoError(t, err)
	top := planeFacing(t, pocket, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10))
	got, err := pocket.Fillet(t.Context(), loopEdgeQuery(top.Loops()[0]), units.Millimeters(1.5))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1, "the receiver is retired")
	requireEveryEdgeOnTwoFaces(t, got)
	patches := filletLoopPatches(got)
	require.Len(t, patches, 4)
	for _, f := range patches {
		c, ok := f.Surface().(decad.Cylinder)
		require.True(t, ok, "a straight wall's fillet patch is a cylinder")
		require.Equal(t, 1.5, c.Radius.Base())
	}
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	requireMeasurementHoldsPi(t, volume, big.NewRat(29325, 2), big.NewRat(333, 4))

	// The quarter ellipse's length by a fine midpoint quadrature.
	const n = 200000
	var arc float64
	for i := range n {
		s := math.Sin(math.Pi / 2 * (float64(i) + 0.5) / n)
		arc += 1.5 * math.Sqrt(1+s*s) * math.Pi / 2 / n
	}
	ellipses := 0
	for _, e := range got.Edges() {
		el, ok := e.Curve().(decad.Ellipse3)
		if !ok {
			continue
		}
		ellipses++
		require.InDelta(t, 1.5*math.Sqrt2, el.SemiMajor.Base(), 1e-12)
		require.Equal(t, 1.5, el.SemiMinor.Base())
		length, err := e.Length()
		require.NoError(t, err)
		require.InDelta(t, arc, length.Value.Base(), length.Bound.Base())
	}
	require.Equal(t, 4, ellipses)

	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())

	// A fillet-banded body exports to STEP (faceted or analytic: which is
	// export's own concern, loop-fillet DF10).
	var step bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &step, got, units.Millimeters(0.1),
		export.WithSTEPName("fillet"), export.WithSTEPAuthor("test"), export.WithSTEPOrganization("test")))
	require.Contains(t, step.String(), "ISO-10303-21")
}

// TestBrepLoopFilletPublicBoxCaps fillets both cap loops of the 100×60×20
// box, a prism receiver, at r = 5 in one call (loop-fillet RF3): the result
// is a brep body with eight quarter cylinders, each band removing
// 320·J₁ − 4·J₂ = 21500/3 − 1750π, so the volume is 317000/3 + 3500π.
func TestBrepLoopFilletPublicBoxCaps(t *testing.T) {
	t.Parallel()
	_, box := filletBox(t)
	got, err := box.Fillet(t.Context(), bothCapLoops(), units.Millimeters(5))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	require.Len(t, filletLoopPatches(got), 8)
	volume, err := got.Volume()
	require.NoError(t, err)
	requireMeasurementHoldsPi(t, volume, big.NewRat(317000, 3), big.NewRat(3500, 1))
}

func TestVertexBlendPublicBoxAllEdges(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 40, 20, 20)
	got, err := box.Fillet(t.Context(), decad.Edges().Exactly(12), units.Millimeters(2))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	volume, err := got.Volume()
	require.NoError(t, err)
	requireMeasurementHoldsPi(t, volume, big.NewRat(14848, 1), big.NewRat(848, 3))
	var spheres int
	for _, f := range filletLoopPatches(got) {
		if _, ok := f.Surface().(decad.Sphere); ok {
			spheres++
		}
	}
	require.Equal(t, 8, spheres)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(got)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, reading.Status)
	var step bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &step, got, units.Millimeters(0.1),
		export.WithSTEPName("vertex blend"), export.WithSTEPAuthor("test"), export.WithSTEPOrganization("test")))
	require.Contains(t, step.String(), "ISO-10303-21")
}

// TestBrepLoopFilletPublicRevolveCapRefuses pins loop-fillet RF4: a complete
// loop of a partial revolve's planar cap is a loop of cap edges whose walls
// are revolved surfaces, so it stays modify-reach SX5 and leaves the receiver
// live.
func TestBrepLoopFilletPublicRevolveCapRefuses(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	ring, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	var capFace *decad.Face
	for _, f := range ring.Faces() {
		if _, ok := f.Surface().(decad.Plane); ok {
			capFace = f
			break
		}
	}
	require.NotNil(t, capFace)
	_, err = ring.Fillet(t.Context(), loopEdgeQuery(capFace.Loops()[0]), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "not a swept meridian junction")
	require.Equal(t, []*decad.Body{ring}, doc.Bodies())
}
