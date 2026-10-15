package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the CURVE half of the shared-denotation certificate's own
// unit coverage, isolating what an integration test cannot: a placement
// test reaching Stitch's public seam always ALSO fails Table J's own
// bit-identical guard the moment a rigid motion moves a vertex, so it can
// never by itself prove denotation.SameCurve's own xform comparison is load-bearing.
// This file drives curveToken/denotation.SameCurve/compose directly instead.

// TestSameCurveRequiresEqualXform is the direct proof that a placement
// breaks the certificate (docs/surface-design.md §6.2's amendment):
// denotation.SameCurve refuses two otherwise-identical tokens once one carries an
// ADDITIONAL motion the other does not, even though this exact scenario
// can never be observed through Stitch's own public seam — there, a
// placement also moves the held coordinate, so Table J's own bit-identical
// guard (kept as a reject-only narrowing on top of the certificate) already
// refuses the pair regardless of what denotation.SameCurve says about the token alone.
func TestSameCurveRequiresEqualXform(t *testing.T) {
	t.Parallel()
	d := bornDocument(&Document{})
	tok := d.n.MintCurve()
	require.NotZero(t, tok.ID)
	require.True(t, denotation.SameCurve(tok, tok), "a token always denotes the same curve as itself")

	axis, ok := r3.NewVec(3, -1, 2).Normalize()
	require.True(t, ok)
	xf, err := r3.RotationAround(r3.NewVec(41, -17, 9), axis, units.Degrees(37))
	require.NoError(t, err)
	moved := tok.Compose(xf)
	require.Equal(t, tok.ID, moved.ID, "compose restates the SAME identity, never a fresh one")
	require.NotEqual(t, tok.Xform, moved.Xform)
	require.False(t, denotation.SameCurve(tok, moved), "the same curve stated under two different motions is not the same admission")
	require.False(t, denotation.SameCurve(moved, tok))
}

// TestSameCurveDeclinesTheZeroToken is denotation.go's own "no certificate
// declines" rule, driven directly: the zero curveToken never equals
// another zero token, and never equals a freshly minted one.
func TestSameCurveDeclinesTheZeroToken(t *testing.T) {
	t.Parallel()
	require.False(t, denotation.SameCurve(curveToken{}, curveToken{}))
	d := bornDocument(&Document{})
	tok := d.n.MintCurve()
	require.False(t, denotation.SameCurve(curveToken{}, tok))
	require.False(t, denotation.SameCurve(tok, curveToken{}))
}

// TestMintCurveNeverRepeats is the document node's own soundness argument for
// two independently built bodies driven directly: two mints from the SAME
// document counter never collide, exactly as two separate Extrude calls
// never share a curveID (apitest/stitch_test.go's own
// TestStitchRefusesIdenticalBoundedRimsWithNoSharedDenotation).
func TestMintCurveNeverRepeats(t *testing.T) {
	t.Parallel()
	d := bornDocument(&Document{})
	a := d.n.MintCurve()
	b := d.n.MintCurve()
	require.NotEqual(t, a.ID, b.ID)
	require.False(t, denotation.SameCurve(a, b))
}

// TestCurveTokenComposeDeclinesTheZeroToken is compose's own reject-only
// discipline: composing a motion onto "no certificate" stays "no
// certificate" — a decline can never be turned into an admission by
// applying a motion to it.
func TestCurveTokenComposeDeclinesTheZeroToken(t *testing.T) {
	t.Parallel()
	axis, ok := r3.NewVec(1, 2, 3).Normalize()
	require.True(t, ok)
	xf, err := r3.RotationAround(r3.NewVec(5, 6, 7), axis, units.Degrees(37))
	require.NoError(t, err)
	require.Zero(t, curveToken{}.Compose(xf))
}
