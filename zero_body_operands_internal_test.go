package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestCertifyFacetedCopiesRejectsBodiesOutsideTheDocument covers a nil tool and
// a zero Body (no document, so a nil document node) in every position
// certifyFacetedCopies reads. Each is ErrUnsupported with the same message as
// when the body held its document pointer directly, never a panic.
func TestCertifyFacetedCopiesRejectsBodiesOutsideTheDocument(t *testing.T) {
	_, source, _ := neutralArcSeamSource(t)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	image, err := Copy(t.Context(), source, WithCopyPlacement(turn))
	require.NoError(t, err)
	zero := bornBody(&Body{})
	foreign := bornBody(&Body{n: topo.Body{Doc: documentNode(New())}})

	noSource := "copied tools lack a live faceted source"
	notLive := "copied tool 2 is not live in the source document"
	for name, tc := range map[string]struct {
		tools []*Body
		want  string
	}{
		"nil list":          {nil, noSource},
		"one tool":          {[]*Body{source}, noSource},
		"nil first":         {[]*Body{nil, image}, noSource},
		"zero first":        {[]*Body{zero, image}, noSource},
		"zero both":         {[]*Body{zero, bornBody(&Body{})}, noSource},
		"nil second":        {[]*Body{source, nil}, notLive},
		"zero second":       {[]*Body{source, zero}, notLive},
		"other document":    {[]*Body{source, foreign}, notLive},
		"zero after a copy": {[]*Body{source, image, zero}, "copied tool 3 is not live in the source document"},
		"nil after a copy":  {[]*Body{source, image, nil}, "copied tool 3 is not live in the source document"},
	} {
		t.Run(name, func(t *testing.T) {
			require.NotPanics(t, func() {
				set, err := certifyFacetedCopies(t.Context(), tc.tools)
				require.Nil(t, set)
				require.ErrorIs(t, err, ErrUnsupported)
				require.ErrorContains(t, err, tc.want)
			})
		})
	}

	// bindFacetedCopyLineage with a nil operand records nothing and does not
	// panic; a zero image is rejected by the caller's document check before
	// the bind (Transform and Copy answer ErrDegenerate first).
	require.NotPanics(t, func() {
		bindFacetedCopyLineage(nil, source, r3.Identity(), false, nil)
		bindFacetedCopyLineage(image, nil, r3.Identity(), false, nil)
		bindFacetedCopyLineage(zero, source, r3.Identity(), false, nil)
	})
	require.Nil(t, zero.n.Payload)
}

// TestDocumentHandleAndNodeIdentity checks the mapping the document compares
// rely on: both directions map nil to nil, and a node always maps back to the
// one handle that holds it, so == on handles keeps its meaning.
func TestDocumentHandleAndNodeIdentity(t *testing.T) {
	require.Nil(t, documentHandle(nil))
	require.Nil(t, documentNode(nil))
	d, other := New(), New()
	require.Same(t, d, documentHandle(documentNode(d)))
	require.Same(t, documentHandle(documentNode(d)), documentHandle(documentNode(d)))
	require.Same(t, documentNode(d), documentNode(documentHandle(documentNode(d))))
	require.NotSame(t, documentNode(d), documentNode(other))
	require.NotSame(t, d, documentHandle(documentNode(other)))
	b := bornBody(&Body{n: topo.Body{Doc: documentNode(d)}})
	require.Same(t, d, b.Document())
}
